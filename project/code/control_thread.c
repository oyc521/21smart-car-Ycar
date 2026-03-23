#include "hybrid_controller.h"
#include "motor.h"
#include "position.h"
#include <rtthread.h>

extern HybridController g_ctrl;

static void control_thread_entry(void *parameter) {
    float dt = 0.02f;
    uint32_t tick_start = rt_tick_get();
    float current_time = 0.0f;

    // 打印确认线程运行（整数形式避免浮点问题）
    wireless_uart_send_string("control_thread running\r\n");

    while (1) {
        Position_Update();

        float car_x = position.x_m;
        float car_y = position.y_m;
        float car_angle = position.yaw_rad;

        float vx, vy, omega;
        HybridController_ComputeControl(&g_ctrl, car_x, car_y, car_angle,
                                        dt, current_time, &vx, &vy, &omega);

        CarController_SetSpeed(vx, vy, omega);
        CarController_Update();

        current_time += dt;
        rt_thread_delay_until(&tick_start, RT_TICK_PER_SECOND * dt);
    }
}

void control_thread_start(void) {
    rt_thread_t tid = rt_thread_create("control",
                                       control_thread_entry,
                                       RT_NULL,
                                       2048,
                                       RT_THREAD_PRIORITY_MAX / 2,
                                       20);
    if (tid) {
        rt_thread_startup(tid);
        wireless_uart_send_string("control_thread created\r\n");
    } else {
        wireless_uart_send_string("control_thread create failed\r\n");
    }
}