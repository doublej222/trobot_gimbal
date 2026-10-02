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

dji m("test", dji::M3508,dji::param_t{.id = 4,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 14);
pid spd_pid(450, 0.27, 1, 3000, 16384);

float target_speed = 25;
float spd = 0;

[[noreturn]]void test_task(void *args) {
    m.init();

    bsp_uart_set_callback(E_UART_1, [](bsp_uart_e device, const uint8_t *data, size_t len) {
        sscanf(reinterpret_cast<const char *>(data), "%f", &spd);
        target_speed = spd;
    });

    while (true) {
        float out = spd_pid.update(m.feedback.speed, target_speed);

        m.update(out);
        vofa::send(E_UART_1, m.feedback.speed, target_speed);
        os::task::sleep(10);
    }
}
