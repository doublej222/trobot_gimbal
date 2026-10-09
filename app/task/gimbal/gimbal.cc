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
#define PITCH_ZERO_POINT 1.12f
constexpr float PITCH_ROLL_LIMIT_UP = 0.347f;
constexpr float PITCH_ROLL_LIMIT_DOWN = -0.4f;
constexpr float PITCH_MOTOR_LIMIT_MIN = 0.74f;
constexpr float PITCH_MOTOR_LIMIT_MAX = 1.48f;

// IMU outer loop gains. Motor position remains the inner loop in the DM4310.
constexpr float PITCH_IMU_KP = 0.45f;
constexpr float PITCH_IMU_KI = 0.15f;
constexpr float PITCH_IMU_KD = 0.02f;
constexpr float PITCH_IMU_INTEGRAL_LIMIT = 0.5f;

using namespace motor;
static dji yaw_motor("yaw", dji::GM6020,dji::param_t{.id = 1,.port = E_CAN_3,.mode = dji::CURRENT}, -1);
static dm pitch_motor("pitch", {.slave_id = 0x21,.master_id = 0x11,.port = E_CAN_1,.mode = dm::MIT,.p_max = 12.5f,.v_max = 30.0f,.t_max = 10.0f});

static float roll_to_motor_position(const float roll) {
    if (roll >= 0.f) {
        return PITCH_ZERO_POINT
             - roll * (PITCH_ZERO_POINT - PITCH_MOTOR_LIMIT_MIN) / PITCH_ROLL_LIMIT_UP;
    }
    return PITCH_ZERO_POINT
         - roll * (PITCH_MOTOR_LIMIT_MAX - PITCH_ZERO_POINT) / -PITCH_ROLL_LIMIT_DOWN;
}

// 滑膜控制算法
smc YawSMC(35, 130, 0.001, 21, 27, 16384, 1.f, 0.5);

static void gimbal_init();

[[noreturn]] void gimbal_task(void *args) {
    auto ins = ins::state();
    auto rc = rc::dr16::data();

    float yaw_target = ins.yaw_total_angle * 180.f / fpi;
    float yaw_current = ins.yaw_total_angle * 180.f / fpi;

    float roll_target = ins.roll > PITCH_ROLL_LIMIT_UP ? PITCH_ROLL_LIMIT_UP :
                        ins.roll < PITCH_ROLL_LIMIT_DOWN ? PITCH_ROLL_LIMIT_DOWN : ins.roll;
    float roll_error_integral = 0.f;
    uint32_t control_timestamp = bsp_time_get_ms();

    int8_t s_r_last = 0;
    int8_t s_l_last = 0;
    uint32_t scan_start_ms = 0;
    float yaw_scan_start = 0.f;

    gimbal_init();

    pitch_motor.control(PITCH_ZERO_POINT,0,25,0.5,1);
    while (true) {
        const uint32_t now = bsp_time_get_ms();
        float dt = static_cast<float>(now - control_timestamp) * 0.001f;
        control_timestamp = now;
        dt = dt < 0.0005f ? 0.0005f : dt > 0.01f ? 0.01f : dt;

        ins = ins::state();
        if (bsp_time_get_ms() - rc->timestamp > 100) {
            yaw_motor.update(0);
        } else {
            // yaw
            yaw_current = ins.yaw_total_angle * 180.f / fpi;

            if (rc->s_l == -1) {
                if (s_l_last != -1) {
                    scan_start_ms = bsp_time_get_ms();
                    yaw_scan_start = yaw_current;
                }
                const float t = static_cast<float>(bsp_time_get_ms() - scan_start_ms) * 0.001f;
                constexpr float w_yaw = 2.f * fpi * 0.5f;
                constexpr float w_pitch = 2.f * fpi * 0.5f;

                yaw_target = yaw_scan_start + 60.f * t - (35.f / w_yaw) * (1 - cosf(w_yaw * t));
                roll_target = -0.3f * sinf(w_pitch * t);
            } else {
                // pitch
                float pitch_stick = static_cast<float>(rc->rc_r[1]) / 660.f;

                roll_target = pitch_stick >= 0.f ? pitch_stick * PITCH_ROLL_LIMIT_DOWN
                                                 : -pitch_stick * PITCH_ROLL_LIMIT_UP;

                // yaw
                if (rc->s_r == 1 && s_r_last != 1) yaw_target += 90.f;
                else if (rc->s_r == -1 && s_r_last != -1) yaw_target -= 90.f;

                yaw_target -= static_cast<float>(rc->rc_r[0]) * 0.5f / 660.f;
            }
            s_l_last = rc->s_l;
            s_r_last = rc->s_r;

            YawSMC.smc_update(yaw_target, yaw_current, ins.gyro[2] * 180.f / fpi);
            yaw_motor.update(YawSMC.output);
        }

        const float roll_error = roll_target - ins.roll;
        roll_error_integral += roll_error * dt;
        roll_error_integral = roll_error_integral > PITCH_IMU_INTEGRAL_LIMIT ? PITCH_IMU_INTEGRAL_LIMIT :
                              roll_error_integral < -PITCH_IMU_INTEGRAL_LIMIT ? -PITCH_IMU_INTEGRAL_LIMIT :
                              roll_error_integral;

        // Positive motor position lowers the launcher, opposite to positive roll.
        float pitch_target = roll_to_motor_position(roll_target)
                           - PITCH_IMU_KP * roll_error
                           - PITCH_IMU_KI * roll_error_integral
                           + PITCH_IMU_KD * ins.gyro[0];
        pitch_target = pitch_target > PITCH_MOTOR_LIMIT_MAX ? PITCH_MOTOR_LIMIT_MAX :
                       pitch_target < PITCH_MOTOR_LIMIT_MIN ? PITCH_MOTOR_LIMIT_MIN : pitch_target;
        pitch_motor.control(pitch_target, 0, 45, 0.5f, 5);

        // debug
        // vofa::send(E_UART_1, yaw_current, yaw_target, YawSMC.output);
        vofa::send(E_UART_1, roll_target, ins.roll, pitch_target, ins.gyro[0],
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
