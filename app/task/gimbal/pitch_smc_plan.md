# Pitch 陀螺闭环 + SMC 力矩控制方案

> 本文仅为设计方案与代码草案，**不参与编译，不改动现有代码**。确认后再手动应用到 `gimbal.cc`。
> 核心目标：把 pitch 从"电机编码器位置环"改为**对地稳定**——反馈用 `ins` 姿态角 + 陀螺角速度，控制量用力矩；复用现有 SMC，输出改喂 DM 的扭矩。

---

## 一、思路

### 1. 方案选择
- 反馈：`ins.roll`（姿态角，实机俯仰轴反映在 roll 上）+ `gyro[PITCH_GYRO_AXIS]`（角速度）。
- 控制器：复用 `smc` 类，输入 角度(°)/角速度(°/s)，输出力矩 τ。
- 与 yaw 的三个本质区别：① 输出是**力矩**不是电流；② pitch 受**重力**；③ pitch 有**机械限位**。

### 2. 执行器接口（关键）
DM 用 MIT 模式，**Kp=0、Kd=0**，此时 `T_ff` 即纯力矩（`dm.cc:70` 只在 `Kp!=0 && Kd==0` 时 disable，0/0 合法）：

```cpp
pitch_motor.control(0.f, 0.f, 0.f, 0.f, tau);
```

可选增强：`control(0, 0, 0, Kd_damp, tau)` 只开电机侧速度阻尼，帮压 SMC 抖振。

### 3. SMC 实例与单位
- 单独建 `PitchSMC`，`u_max = 10`（DM `t_max`），`J/C/K/epsilon` 按 pitch 惯量重调，**不要照抄 yaw**。
- `error = angle - target`，隐含假设"输出正 → 角度增大"。
- 无积分器（只有 `target_last`），撞限位不会积分卷绕。
- `slidingmodec.cc:14` 逐拍差分不除 dt，隐含固定 1kHz，别改循环周期。
- `|error| < error_eps` 返回 0；pitch 有重力，建议 `error_eps = 0` 或配重力前馈。

### 4. 符号核对（readme 里的正反馈坑）
必须确认 **DM 力矩正方向 ↔ `ins.roll` 增大方向一致**，不一致就整段取反。手推平台测最快，符号错会直接自激。

### 5. 重力前馈（pitch 特有）
yaw 无恒定负载，pitch 一直对抗重力。切换项虽能压有界扰动，但会抖振/偏移。工程做法：

```
tau = PitchSMC.output + τ_grav(angle)      // τ_grav = A·cos(angle) + B
```

先叠加、再限幅。好处：减小切换增益需求，`error_eps` 附近也不掉力。

### 6. 限位处理（三层）
1. **动态软限位**：把电机机械限位实时映射到 `ins` 空间。
   ```
   base_pitch  = ins.roll - g(motor_pos)      // 底座倾角估计
   angle_max   = 角度上限 + base_pitch - margin
   angle_min   = 角度下限 + base_pitch + margin
   target      = clamp(target, angle_min, angle_max)
   ```
   `g(m)` = 电机角 → 俯仰轴相对底座角，由**水平标定**得到（`roll_to_motor_position` 的反函数）。底座歪时限位跟着平移，云台"顺"着限位走。
2. **虚拟墙**：`pos` 进入 `LIMIT ± margin` 加反向回弹力矩，避免硬顶堵转发热。
3. **硬保护**：`pos` 越过真实 `MIN/MAX` 时，**只允许退出方向**的力矩，禁止继续压。

### 7. 失效与安全
- RC 超时 / `ins` 未 ready / `feedback.timestamp==0` → τ 立即归零（纯力矩下会因重力掉落，需明确是"卸力"还是"切回位置环抱住"）。
- 纯力矩没有自保持，失效逻辑必须显式设计。

### 8. 调参顺序
1. 核符号 + 测 `gyro` 轴对应关系。
2. 标定 `g(m)` 与 `τ_grav(angle)`。
3. 先小 `u_max` 粗调 SMC，再放大；VOFA 看 角度/角速度/力矩。
4. 加动态限位 → 虚拟墙 → 硬保护。
5. 最后接失效逻辑。

### 9. 待确认清单
- [ ] `gyro` 哪个分量是 pitch 轴、符号对不对
- [ ] DM 力矩正方向 ↔ `ins.roll` 方向
- [ ] `g(m)` 是否近似线性、是否直驱（常数偏置）
- [ ] `PITCH_MOTOR_LIMIT_MIN/MAX` 是否就是机械停点
- [ ] IMU 是否装在俯仰平台上

---

## 二、基于当前代码的实现草案

在现有 `gimbal.cc`（yaw 逻辑保持不动）基础上，改动集中在 pitch：删除"IMU 位置修正 + DM 位置环"，换成"SMC 力矩 + 重力前馈 + 三层限位 + 失效保护"。

### 2.1 完整 `gimbal.cc`（草案）

```cpp
#include "slidingmodec.h"
#include <cmath>
#include <cstdio>
#include "ins/ins.h"
#include "motor/dji.h"
#include "motor/dm.h"
#include "utils/msg.h"
#include "utils/os.h"
#include "utils/vofa.h"
#include "rc/dr16.h"

constexpr float fpi = M_PI;
constexpr float PITCH_ZERO_POINT = 1.12f;

// 水平标定：大地角(rad) <-> 电机角(rad) 的对应范围
constexpr float PITCH_ANGLE_LIMIT_UP   = 0.347f;
constexpr float PITCH_ANGLE_LIMIT_DOWN = -0.4f;
constexpr float PITCH_MOTOR_LIMIT_MIN  = 0.74f;
constexpr float PITCH_MOTOR_LIMIT_MAX  = 1.48f;
constexpr float PITCH_LIMIT_MARGIN     = 0.05f;   // 动态限位内缩 (rad)

// 反馈轴：实机俯仰轴落在 ins.roll / gyro[0]（需实测确认）
constexpr int   PITCH_GYRO_AXIS = 0;
constexpr float PITCH_TAU_LIMIT = 10.f;           // DM t_max (Nm)

// SMC：参数为占位值，必须按 pitch 惯量重调
smc PitchSMC(35.f, 130.f, 0.f, 21.f, 27.f, PITCH_TAU_LIMIT, 1.f, 0.5f);

// 重力前馈：tau = A*cos(angle) + B，待标定
constexpr float PITCH_GRAVITY_A = 0.f;
constexpr float PITCH_GRAVITY_B = 0.f;

// 虚拟墙
constexpr float PITCH_WALL_MARGIN = 0.15f;        // rad (电机角)
constexpr float PITCH_WALL_K      = 8.0f;         // Nm/rad

using namespace motor;
static dji yaw_motor("yaw", dji::GM6020, dji::param_t{.id = 1, .port = E_CAN_3, .mode = dji::CURRENT}, -1);
static dm pitch_motor("pitch", {.slave_id = 0x21, .master_id = 0x11, .port = E_CAN_1, .mode = dm::MIT, .p_max = 12.5f, .v_max = 30.0f, .t_max = 10.0f});

// 滑模控制算法
smc YawSMC(35, 130, 0.001, 21, 27, 16384, 1.f, 0.5);

// 水平标定关系的反函数：电机角 -> 俯仰轴相对底座角(rad)
static float motor_to_gimbal(const float motor) {
    if (motor <= PITCH_ZERO_POINT)
        return (PITCH_ZERO_POINT - motor) * PITCH_ANGLE_LIMIT_UP / (PITCH_ZERO_POINT - PITCH_MOTOR_LIMIT_MIN);
    return (PITCH_ZERO_POINT - motor) * (-PITCH_ANGLE_LIMIT_DOWN) / (PITCH_MOTOR_LIMIT_MAX - PITCH_ZERO_POINT);
}

static void gimbal_init();

[[noreturn]] void gimbal_task(void *args) {
    ins::data_t ins_data = ins::state();
    auto rc = rc::dr16::data();

    float yaw_target  = ins_data.yaw_total_angle * 180.f / fpi;
    float yaw_current = ins_data.yaw_total_angle * 180.f / fpi;
    float roll_target = ins_data.roll;                 // 对地目标角 (rad)

    int8_t s_r_last = 0;
    int8_t s_l_last = 0;
    uint32_t scan_start_ms = 0;
    float yaw_scan_start = 0.f;

    gimbal_init();
    pitch_motor.control(PITCH_ZERO_POINT, 0.f, 25.f, 0.5f, 1.f);   // 开机短暂位置保持

    while (true) {
        ins_data = ins::state();
        const bool rc_ok  = bsp_time_get_ms() - rc->timestamp <= 100;
        const bool imu_ok = ins::ready();

        if (!rc_ok) yaw_motor.update(0);

        if (rc_ok) {
            // ---- yaw（保持原样）----
            yaw_current = ins_data.yaw_total_angle * 180.f / fpi;

            if (rc->s_l == -1) {
                if (s_l_last != -1) {
                    scan_start_ms = bsp_time_get_ms();
                    yaw_scan_start = yaw_current;
                }
                const float t = static_cast<float>(bsp_time_get_ms() - scan_start_ms) * 0.001f;
                constexpr float w_yaw = 2.f * fpi * 0.5f;
                constexpr float w_pitch = 2.f * fpi * 0.5f;

                yaw_target = yaw_scan_start + 60.f * t - (35.f / w_yaw) * cosf(w_yaw * t);
                roll_target = -0.3f * sinf(w_pitch * t);           // 对地角度激励
            } else {
                // pitch 目标：对地角度，0 = 水平
                float pitch_stick = static_cast<float>(rc->rc_r[1]) / 660.f;
                pitch_stick = pitch_stick > 1.f ? 1.f : pitch_stick < -1.f ? -1.f : pitch_stick;
                roll_target = pitch_stick >= 0.f ? pitch_stick * PITCH_ANGLE_LIMIT_DOWN
                                                 : -pitch_stick * PITCH_ANGLE_LIMIT_UP;

                // yaw
                if (rc->s_r == 1 && s_r_last != 1) yaw_target += 90.f;
                else if (rc->s_r == -1 && s_r_last != -1) yaw_target -= 90.f;

                yaw_target -= static_cast<float>(rc->rc_r[0]) * 0.5f / 660.f;
            }
            s_l_last = rc->s_l;
            s_r_last = rc->s_r;

            YawSMC.smc_update(yaw_target, yaw_current, ins_data.gyro[2] * 180.f / fpi);
            yaw_motor.update(YawSMC.output);
        }

        // ---- pitch：SMC 力矩闭环 ----
        float tau = 0.f;
        const float pos = pitch_motor.feedback.pos;
        const bool sensors_ok = imu_ok && pitch_motor.feedback.timestamp != 0;

        if (sensors_ok) {
            // 第 1 层：动态软限位（电机机械限位 -> 大地系）
            const float base_pitch = ins_data.roll - motor_to_gimbal(pos);
            const float angle_max  = PITCH_ANGLE_LIMIT_UP   + base_pitch - PITCH_LIMIT_MARGIN;
            const float angle_min  = PITCH_ANGLE_LIMIT_DOWN + base_pitch + PITCH_LIMIT_MARGIN;
            const float target     = roll_target > angle_max ? angle_max :
                                     roll_target < angle_min ? angle_min : roll_target;

            PitchSMC.smc_update(target * 180.f / fpi,
                                ins_data.roll * 180.f / fpi,
                                ins_data.gyro[PITCH_GYRO_AXIS] * 180.f / fpi);
            tau = PitchSMC.output
                + PITCH_GRAVITY_A * cosf(ins_data.roll) + PITCH_GRAVITY_B;

            // 第 2 层：虚拟墙
            if (pos > PITCH_MOTOR_LIMIT_MAX - PITCH_WALL_MARGIN)
                tau -= PITCH_WALL_K * (pos - (PITCH_MOTOR_LIMIT_MAX - PITCH_WALL_MARGIN));
            if (pos < PITCH_MOTOR_LIMIT_MIN + PITCH_WALL_MARGIN)
                tau += PITCH_WALL_K * ((PITCH_MOTOR_LIMIT_MIN + PITCH_WALL_MARGIN) - pos);

            // 第 3 层：硬保护（仅禁止继续压向限位，不挡退出方向）
            // 前提：正力矩使 pos 增大，实测相反则整体取反
            if (pos >= PITCH_MOTOR_LIMIT_MAX && tau > 0.f) tau = 0.f;
            if (pos <= PITCH_MOTOR_LIMIT_MIN && tau < 0.f) tau = 0.f;

            tau = tau > PITCH_TAU_LIMIT ? PITCH_TAU_LIMIT :
                  tau < -PITCH_TAU_LIMIT ? -PITCH_TAU_LIMIT : tau;

            // 失效：卸力（若想抱住可改成 control(pos, 0, Kp, Kd, 0) 位置保持）
            if (!rc_ok) tau = 0.f;
        }

        pitch_motor.control(0.f, 0.f, 0.f, 0.f, tau);

        // debug
        vofa::send(E_UART_1, roll_target, ins_data.roll, tau, ins_data.gyro[PITCH_GYRO_AXIS],
                   pitch_motor.feedback.pos, pitch_motor.feedback.vel, pitch_motor.feedback.torque);
        os::task::sleep(1);
    }
}

void gimbal_init() {
    yaw_motor.init();
    yaw_motor.enable();
    pitch_motor.init();
    pitch_motor.reset();
    os::task::sleep(10);
    pitch_motor.enable();
}
```

### 2.2 相对现有代码的改动点
| 位置 | 现状 | 草案 |
| --- | --- | --- |
| 顶部常量 | `PITCH_IMU_KP/KI/KD/INTEGRAL_LIMIT`、`roll_to_motor_position` | 换成 SMC/重力/限位常量 + `motor_to_gimbal` |
| 控制量 | `pitch_target` 经 DM **位置环** `control(pos,0,35,0.8,2)` | SMC 输出经 **纯力矩** `control(0,0,0,0,tau)` |
| 修正逻辑 | IMU 误差 PI(D) 修正位置设定、含积分 | 移除，SMC 直接闭环角度 |
| 限位 | 静态 ins 限位 + 电机限位夹取 | 动态 ins 限位 + 虚拟墙 + 硬保护 |
| 失效 | 超时只零 yaw，pitch 继续闭环 | 超时 τ 归零（或位置抱住） |

### 2.3 需要填/确认的地方
1. `PitchSMC` 的 `C/K/q/p/epsilon/J`：占位值，必须重调。
2. `PITCH_GRAVITY_A/B`：实测标定，否则留 0 靠 SMC 切换项硬扛（会抖）。
3. `PITCH_GYRO_AXIS`、力矩方向符号：实测确认。
4. `PITCH_WALL_MARGIN/K`：先小后大调，观察撞限位是否平滑。
5. `g(m)`：确认是否直驱线性；有传动/间隙则加 margin。
