//线程控制器，主要用于线程周期运动控制
#include <rtthread.h>
#include "position.h"
#include "motor.h"
#include "hybrid_controller.h"
#include "encoder.h"   // 添加编码器头文件

extern Position_t position;
extern HybridController g_ctrl;

static void control_thread_entry(void *parameter)
{
    float dt = 0.02f;
    uint32_t tick_start = rt_tick_get();
    float current_time = 0.0f;

    while (1) {
        Position_Update();

        // 获取当前速度
        float current_speed[3];
        EncoderGetSpeeds(current_speed);

        // 可根据需要将 current_speed 用于控制（例如传入 CarController_Update）
        // 或者将速度存储到 g_ctrl 供控制器使用

        float car_x = position.x_m;
        float car_y = position.y_m;
        float car_angle = position.yaw_rad;

        float vx, vy, omega;
        HybridController_ComputeControl(&g_ctrl, car_x, car_y, car_angle,
                                        dt, current_time, &vx, &vy, &omega);

        CarController_SetSpeed(vx, vy, omega);
        CarController_Update();   // 此处内部可使用 current_speed

        current_time += dt;
        rt_thread_delay_until(&tick_start, RT_TICK_PER_SECOND * dt);
    }
}

void control_thread_start(void)
{
    rt_thread_t tid = rt_thread_create("control",
                                       control_thread_entry,
                                       RT_NULL,
                                       2048,
                                       RT_THREAD_PRIORITY_MAX / 2,
                                       20);
    if (tid) rt_thread_startup(tid);
}