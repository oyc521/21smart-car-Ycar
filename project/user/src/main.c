/**
 * ϵͳ�汾��v5.7.0
 * ��Ҫ���£�
 * - �����������ԣ��������ƶ�������������0�㣬����ȫ��ƽ�Ʊ�������ۻ�����
 * - �Ƴ��ƶ�ǰ��ת��׼����Ϊ��ƽ���ƶ� + ���溽����P��Ư��
 * - �޸�ը���ƶ�����ӳ�䣨start_push_bomb ���� 0~3 �������飩
 * - �޸��ƶ�����λ�Ʒ������ƶ�������ͳһ
 * - ͳһ����ӳ��洢��ʶ����д��ȫ�� g_digit_map�����������ͬ����ȡ
 * - ��ͼ�����¼����ƣ�main.c ���յ�ͼ���� TASK_EVENT_MAP_READY �������������
 * - �ƶ�վλ�����ھ�����ֵ��ϸ���������֤
 * - �ڶ���ʶ���ܣ�Ŀ�ĵ�����/����ͼ��������������֧�� KEY3/KEY2 ����
 * - PID �������飨����/����׼/�ƶ�����ǶȻ�ģʽ�л�
 * - �Ӿ�У������������������� MAP_READY �¼���ִ�У�����ʹ����ѭ����ֹ���
 */
#include <rtthread.h>
#include <math.h>
#include "imu660ra_ahrs.h"
#include "position.h"
#include "pid.h"
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "motor.h"
#include "encoder.h"
#include "planner.h"
#include "hybrid_controller.h"
#include "uart_receiver.h"
#include "task_manager.h"
#include "seekfree_assistant.h"
#include "planner.h"
#include "kinematics.h"
#include "filter.h"
#include "uart4_recognition.h"
#include "debug.h"

// ========== ���Ŷ��� ==========
#define KEY_START_PIN       C12
#define LED_CONFIRM_PIN     B9
#define MAP_REQ_INTERVAL_MS   200
#define MAP_REQ_MAX_ATTEMPTS  25
#define BEEP_PIN            B11

// ========== ȫ�ֱ��� ==========
GridMap g_grid_map;
GameState g_game_state;
HybridController g_ctrl;
static uint8_t system_started = 0;
static uint32_t last_map_req_tick = 0;
uint8_t waiting_map = 0;
uint8_t need_map_update = 0;
static uint8_t map_req_attempts = 0;

// ϵͳ����ʱ�����ĺ��򣨶ȣ�������ʶ���ָ�����
float g_locked_yaw = 0.0f;

extern uint8_t g_map_updated;
rt_mutex_t g_map_mutex = RT_NULL; // ���������������ݾ���

// ========== YAW ���Ա��� ==========
static uint8_t yaw_test_active = 0;
static float yaw_test_ref = 0.0f;

static void beep_init(void)
{
    gpio_init(BEEP_PIN, GPO, 0, GPO_PUSH_PULL);   // ��ʼ�͵�ƽ������
}

void BeepOnce(void)
{
    gpio_set_level(BEEP_PIN, 1);   // �ߵ�ƽ����
    rt_thread_mdelay(100);
    gpio_set_level(BEEP_PIN, 0);   // �͵�ƽ�ر�
}

/* LED ��˸һ�� */
static void led_blink_once(void)
{
    gpio_set_level(LED_CONFIRM_PIN, 0);
    rt_thread_mdelay(100);
    gpio_set_level(LED_CONFIRM_PIN, 1);
}

/* �� OpenArt �����µ�ͼ */
static void request_new_map(void)
{
    wireless_uart_send_string("request_new_map called.\r\n");
    uart_write_string(UART_1, "MAP_REQ\n");
    wireless_uart_send_string("MAP_REQ sent.\r\n");
}

/* �����߳���� */
static void control_thread_entry(void *parameter)
{
    wireless_uart_send_string("Control thread started.\r\n");

    uint32_t tick = rt_tick_get();
    char buf[128];
    uint8_t yaw_initialized = 0;
    float current_omega_rad = 0.0f;
    static float last_omega_cmd = 0.0f;  // ���ڵ��Խ��ٶ�ָ��仯

    while (1) {      
        AHRS_Update();  
        Position_Update();

#if DEBUG_ENABLE
        debug_seekfree_loop();
#endif

        // ===== ��ͼ����������������æ�жϣ� =====
        if (need_map_update) {
            if (g_ctrl.mode == CTRL_MODE_PATH_FOLLOWING) {
                // ·�������У��ȴ����к��ٴ���
            } else {
                need_map_update = 0;
                waiting_map = 1;
                map_req_attempts = 0;
                request_new_map();
            }
        }

        key_scanner();

        // ����������KEY4 �̰�����ϵͳ����һ�أ�
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_4);
            if (!system_started) {
                system_started = 1;
                task_manager_set_mode(TASK_MODE_STAGE1);   // ���õ�һ��ģʽ
                led_blink_once();
                //BeepOnce();
                wireless_uart_send_string("Stage1 started. Requesting initial map...\r\n");
                waiting_map = 1;
                map_req_attempts = 0;
                request_new_map();
                AHRS_ResetYaw();
                Position_ResetYaw();               // ͬ���ںϺ���
                g_locked_yaw = AHRS_GetYaw();
                yaw_initialized = 1;
            }
        }

        // ������KEY3 �̰�����ϵͳ���ڶ��أ���ʶ��
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_3);
            if (!system_started) {
                system_started = 1;
                task_manager_set_mode(TASK_MODE_STAGE2);   // ���õڶ���ģʽ
                led_blink_once();
                BeepOnce();
                wireless_uart_send_string("Stage2 started. Requesting initial map...\r\n");
                waiting_map = 1;
                map_req_attempts = 0;
                request_new_map();
                AHRS_ResetYaw();
                Position_ResetYaw();               // ͬ���ںϺ���
                g_locked_yaw = AHRS_GetYaw();
                yaw_initialized = 1;
            }
        }

        // ��ͼ����ʱ����
        if (system_started && waiting_map) {
            uint32_t now = rt_tick_get();
            if (last_map_req_tick == 0 || (now - last_map_req_tick) >= RT_TICK_PER_SECOND * MAP_REQ_INTERVAL_MS / 1000) {
                if (map_req_attempts >= MAP_REQ_MAX_ATTEMPTS) {
                    wireless_uart_send_string("Map request timeout, reset waiting_map.\r\n");
                    waiting_map = 0;
                    map_req_attempts = 0;
                    last_map_req_tick = 0;
                } else {
                    request_new_map();
                    map_req_attempts++;
                    last_map_req_tick = now;
                }
            }
        }

        // ��ͼ�������
        if (waiting_map && g_map_updated) {
            g_map_updated = 0;
            waiting_map = 0;
            map_req_attempts = 0;
            last_map_req_tick = 0;
            wireless_uart_send_string("New map received, sending event to task manager.\r\n");
            rt_event_send(g_task_mgr.event, TASK_EVENT_MAP_READY);
        }

        // ========== �Զ�ģʽ���� ==========
        else if (system_started && yaw_initialized && !waiting_map) {
            float vx_out, vy_out, omega;
            float current_time = (float)rt_tick_get() / 1000.0f;
            HybridController_ComputeControl(&g_ctrl, position.x_m, position.y_m, position.yaw_rad,
                                            0.005f, current_time, &vx_out, &vy_out, &omega);

            if (fabsf(omega - last_omega_cmd) > 0.01f) {
                char dbg[32];
                rt_sprintf(dbg, "CMD_OMEGA:%d\r\n", (int)(omega * 1000));
                wireless_uart_send_string(dbg);
                last_omega_cmd = omega;
            }

            CarController_SetSpeed(vx_out, vy_out, omega);
            CarController_Update();
            current_omega_rad = car_ctrl.current_omega;
        } else {
            // δ������ȴ���ͼʱֹͣ���
            CarController_SetSpeed(0, 0, 0);
            CarController_Update();
            current_omega_rad = 0.0f;
        }

        // ���Դ�ӡ������ģʽ�仯ʱ���һ��
        static int last_mode = -1;
        if (g_ctrl.mode != last_mode) {
            last_mode = g_ctrl.mode;
            char dbg[32];
            rt_sprintf(dbg, "[Mode] changed to %d\r\n", g_ctrl.mode);
            wireless_uart_send_string(dbg);
        }

        // �Ӿ�У������������������� MAP_READY �¼��У��˴����ٵ��� try_vision_reset()
        rt_thread_mdelay(5);   // ��������
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    wireless_uart_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);

    gpio_init(LED_CONFIRM_PIN, GPO, 1, GPO_PUSH_PULL);
    beep_init();    // ��ʼ��������
    key_init(10);   // 10ms��ʱ����

    MotorInit();
    EncoderInit();
    CarController_Init();
    MotorPID_SetGlobalParams(100.0f, 2.5f, 1.0f); // ����pid����

    IMU660RA_AHRS_Init();
    Position_Init();
    task_manager_init();
    task_manager_start();

    memset(&g_game_state, 0, sizeof(g_game_state));
    memset(&g_grid_map, 0, sizeof(g_grid_map));

    HybridController_Init(&g_ctrl, &g_grid_map, &g_game_state);
    g_ctrl.max_speed = 0.10f;
    g_ctrl.path_tolerance = 0.15f;

    debug_module_init();

    uart_receive_init();
    uart4_recognition_init();  

    // ����������
    g_map_mutex = rt_mutex_create("map_mutex", RT_IPC_FLAG_FIFO);
    if (g_map_mutex == RT_NULL) {
        wireless_uart_send_string("Failed to create map mutex.\r\n");
    }

    rt_thread_t uart_thread = rt_thread_create("uart_parse",
                                               parse_uart_data_thread_entry,
                                               RT_NULL,
                                               4096,
                                               5,
                                               20);
    if (uart_thread) rt_thread_startup(uart_thread);
    else wireless_uart_send_string("Failed to create uart thread.\r\n");

    pit_ms_init(PIT_CH0, 10);
    interrupt_global_enable(0);

    rt_thread_t ctrl_thread = rt_thread_create("control", control_thread_entry, NULL,
                                               8192, RT_THREAD_PRIORITY_MAX / 2, 20);
    if (ctrl_thread) {
        rt_thread_startup(ctrl_thread);
        wireless_uart_send_string("Control thread created and started.\r\n");
    } else {
        wireless_uart_send_string("Failed to create control thread.\r\n");
    }

    wireless_uart_send_string("System ready. Press KEY4 to start.\r\n");

    rt_system_scheduler_start();

    return 0;
}