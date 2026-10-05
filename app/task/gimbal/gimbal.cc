#include "slidingmodec.h"
#include <cstdio>
#include "ins/ins.h"
#include "motor/dji.h"
#include "motor/dm.h"
#include "utils/msg.h"
#include "utils/os.h"
#include "utils/vofa.h"
#include "rc/dr16.h"

const float fpi = M_PI;
#define PITCH_ZERO_POINT 1.12f
#define PITCH_LIMIT_MAX 1.49
#define PITCH_LIMIT_MIN 0.77
// static float wrap_deg(float deg) {
//     while (deg > 180.f) deg -= 360.f;
//     while (deg < -180.f) deg += 360.f;
//     return deg;
// }

using namespace motor;
dji yaw_motor("yaw", dji::GM6020,dji::param_t{.id = 1,.port = E_CAN_3,.mode = dji::CURRENT}, -1);
dm pitch_motor("pitch", {.slave_id = 0x21,.master_id = 0x11,.port = E_CAN_1,.mode = dm::MIT,.p_max = 12.5f,.v_max = 30.0f,.t_max = 10.0f});

// 滑膜控制算法
smc YawSMC(35, 130, 0.001, 21, 27, 16384, 1.f, 0.5);

void gimbal_init();

[[noreturn]] void gimbal_task(void *args) {

    auto ins = ins::data();
    auto rc = rc::dr16::data();

    float yaw_target = ins->yaw_total_angle * 180.f / fpi;
    int8_t s_r_last = 0;

    gimbal_init();

    pitch_motor.control(PITCH_ZERO_POINT,0,40,2,0);
    while (true) {
        float yaw_current = ins->yaw_total_angle * 180.f / fpi;
        // float pitch_current = ins->pitch * 180.f / fpi;

        if (bsp_time_get_ms() - rc->timestamp > 100) {
            yaw_motor.update(0);
            pitch_motor.reset();
        } else {
            pitch_motor.control(PITCH_ZERO_POINT, 0, 40, 2, 0);
            if (rc->s_r == 1 && s_r_last != 1) yaw_target = yaw_current + 90.f;
            else if (rc->s_r == -1 && s_r_last != -1) yaw_target = yaw_current - 90.f;
            s_r_last = rc->s_r;
            s_r_last = rc->s_r;

            // delta_yaw_target = static_cast<float>(rc::dr16::data()->rc_r[0]) * 0.0007f;
            YawSMC.smc_update(yaw_target, yaw_current, ins->gyro[2] * 180.f / fpi);

            yaw_motor.update(YawSMC.output);
        }
        // vofa::send(E_UART_1, yaw_current, yaw_target, YawSMC.output);

        vofa::send(E_UART_1, pitch_motor.feedback.timestamp, pitch_motor.feedback.temp_mos, pitch_motor.feedback.pos, pitch_motor.feedback.vel, pitch_motor.feedback.raw.pos, ins->gyro[0]);
        os::task::sleep(1);
    }
}

void gimbal_init() {
    yaw_motor.init();
    yaw_motor.enable();
    pitch_motor.init();
    pitch_motor.enable();
    // while (pitch_motor.feedback.timestamp == 0 or pitch_motor.feedback.err != 1) {
    //     pitch_motor.reset();
    //     os::task::sleep(10);
    //     pitch_motor.enable();
    //     os::task::sleep(10);
    // }
}