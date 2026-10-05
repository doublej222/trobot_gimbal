//
// Created by double_J on 2026/9/29.
//
#include <cmath>
#include <cstdio>
#include "controller/pid.h"
#include "ins/ins.h"
#include "motor/dji.h"
#include "utils/msg.h"
#include "utils/os.h"
#include "utils/vofa.h"
using namespace motor;
using namespace controller;

dji m("test", dji::M3508,dji::param_t{.id = 4,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 15.67);
pid spd_pid(750, 20, 5, 16384, 16384);

float target_speed = 5;

[[noreturn]]void test_task(void *args) {
    m.init();

    // uart callback
    // bsp_uart_set_callback(E_UART_1, [](bsp_uart_e device, const uint8_t *data, size_t len) {
    //     sscanf(reinterpret_cast<const char *>(data), "%f", &target_speed);
    // });

    while (true) {
        // target_speed = 20 * sinf(2 * M_PI * 0.25f * bsp_time_get_ms() / 1000.f);

        // float target = (fmodf(static_cast<float>(bsp_time_get_ms()) / 1000.f, 4.0f) < 1.0f) ? target_speed : -target_speed;
        float out = spd_pid.update(m.feedback.speed, target_speed);


        m.update(out);
        vofa::send(E_UART_1, m.feedback.speed, target_speed, m.output);
        os::task::sleep(1);
    }
}
