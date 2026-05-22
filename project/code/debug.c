#if DEBUG_ENABLE

#include "debug.h"
#include <string.h>
#include <stdlib.h>
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "seekfree_assistant.h"
#include "motor.h"
#include "encoder.h"
#include "position.h"
#include "hybrid_controller.h"
#include "planner.h"
#include "mapping.h"
#include "uart4_recognition.h"
#include "task_manager.h"
#include "uart_receiver.h"

/* SeekFree 协议走 int32 ×1000 避免浮点传输问题 */
#define DEBUG_PARAM_SCALE 1000.0f

extern HybridController g_ctrl;
extern float pid_kp, pid_ki, pid_kd;
extern PIDParam_t angle_trace_param;
extern PIDParam_t pos_x_param, pos_y_param;
extern rt_mutex_t g_map_mutex;
extern GridMap g_grid_map;
extern GameState g_game_state;
extern uint8_t waiting_map;
extern uint8_t need_map_update;

static char cmd_queue[DEBUG_CMD_QUEUE_SIZE][DEBUG_CMD_LINE_MAX];
static volatile uint8_t cmd_queue_head = 0;
static volatile uint8_t cmd_queue_tail = 0;

static uint8_t cmd_queue_count(void)
{
    return (cmd_queue_head - cmd_queue_tail) % DEBUG_CMD_QUEUE_SIZE;
}

static uint8_t cmd_queue_full(void)
{
    return cmd_queue_count() == (DEBUG_CMD_QUEUE_SIZE - 1);
}

static void cmd_queue_push(const char *line)
{
    uint8_t next = (cmd_queue_head + 1) % DEBUG_CMD_QUEUE_SIZE;
    if (next != cmd_queue_tail) {
        strncpy(cmd_queue[cmd_queue_head], line, DEBUG_CMD_LINE_MAX - 1);
        cmd_queue[cmd_queue_head][DEBUG_CMD_LINE_MAX - 1] = '\0';
        cmd_queue_head = next;
    }
}

static uint8_t cmd_queue_pop(char *line)
{
    if (cmd_queue_head == cmd_queue_tail) return 0;
    strncpy(line, cmd_queue[cmd_queue_tail], DEBUG_CMD_LINE_MAX - 1);
    line[DEBUG_CMD_LINE_MAX - 1] = '\0';
    cmd_queue_tail = (cmd_queue_tail + 1) % DEBUG_CMD_QUEUE_SIZE;
    return 1;
}

/* ========== 三组参数通道映射表 ========== */

static const DebugChannel_t group_speed[DEBUG_CHANNEL_COUNT] = {
    { "INNER_KP",       &pid_kp,                100.0f, 0.0f,  500.0f },
    { "INNER_KI",       &pid_ki,                2.5f,   0.0f,  100.0f },
    { "INNER_KD",       &pid_kd,                1.0f,   0.0f,  50.0f  },
    { "NAV_SPD_MAX",    &g_ctrl.max_speed,      0.10f,  0.01f, 1.0f   },
    { "MIN_SPEED",      &g_ctrl.min_speed,      0.08f,  0.01f, 0.5f   },
    { "PUSH_SPD_MAX",   &g_ctrl.push_speed_max, 0.08f,  0.01f, 1.0f   },
    { NULL,             NULL,                   0.0f,   0.0f,  0.0f    },
    { NULL,             NULL,                   0.0f,   0.0f,  0.0f    },
};

static const DebugChannel_t group_angle[DEBUG_CHANNEL_COUNT] = {
    { "ANGL_KP",        &angle_trace_param.kp,   0.8f,   0.0f,  5.0f   },
    { "ANGL_KD",        &angle_trace_param.kd,   0.2f,   0.0f,  3.0f   },
    { NULL,             NULL,                    0.0f,   0.0f,  0.0f   },
    { NULL,             NULL,                    0.0f,   0.0f,  0.0f   },
    { NULL,             NULL,                    0.0f,   0.0f,  0.0f   },
    { NULL,             NULL,                    0.0f,   0.0f,  0.0f   },
    { NULL,             NULL,                    0.0f,   0.0f,  0.0f   },
    { NULL,             NULL,                    0.0f,   0.0f,  0.0f   },
};

static const DebugChannel_t group_tracking[DEBUG_CHANNEL_COUNT] = {
    { "LOOKAHEAD",      &g_ctrl.lookahead_dist,             0.15f,  0.02f, 0.50f },
    { "PATH_TOL",       &g_ctrl.path_tolerance,             0.03f,  0.005f,0.20f },
    { "MIN_LOOKAHD",    &g_ctrl.min_lookahead,              0.08f,  0.02f, 0.30f },
    { "MAX_LOOKAHD",    &g_ctrl.max_lookahead,              0.50f,  0.10f, 1.00f },
    { "POS_KP",         &pos_x_param.kp,                    2.0f,   0.0f,  10.0f  },
    { "POS_KD",         &pos_x_param.kd,                    0.8f,   0.0f,  5.0f   },
    { NULL,             NULL,                               0.0f,   0.0f,  0.0f   },
    { NULL,             NULL,                               0.0f,   0.0f,  0.0f   },
};

static const DebugChannel_t *group_tables[DEBUG_GROUP_COUNT] = {
    group_speed,
    group_angle,
    group_tracking,
};

static const char *group_names[DEBUG_GROUP_COUNT] = {
    "speed",
    "angle",
    "tracking",
};

static DebugGroup_t current_group = DEBUG_GROUP_SPEED;
static DebugChannel_t channel_map[DEBUG_CHANNEL_COUNT];

static seekfree_assistant_receive_callback_function saved_receive_callback = NULL;

/* ========== 内置空地图 ========== */
static const char empty_map_text[] =
    "----------------\n"
    "----------------\n"
    "----------------\n"
    "----------------\n"
    "----------------\n"
    "----------------\n"
    "----------------\n"
    "----------------\n"
    "----------------\n"
    "----------------\n"
    "----------------\n"
    "----------------\n";

/* ========== 测试函数前向声明 ========== */
static void load_empty_map(void);
static void test_straight(float dist);
static void test_digit_recog(void);
static void test_box_recog(void);
static void test_nav(float x, float y);
static void test_square(float side);

/* ========== 参数组管理函数 ========== */

static void map_group(DebugGroup_t group)
{
    current_group = group;
    const DebugChannel_t *table = group_tables[group];

    for (int i = 0; i < DEBUG_CHANNEL_COUNT; i++) {
        channel_map[i] = table[i];
        if (table[i].name && table[i].param_ptr) {
            int32_t scaled = (int32_t)(*table[i].param_ptr * DEBUG_PARAM_SCALE);
            *(int32_t *)&seekfree_assistant_parameter[i] = scaled;
        } else {
            *(int32_t *)&seekfree_assistant_parameter[i] = 0;
        }
    }
}

void debug_apply_parameter(uint8_t channel, float value)
{
    if (channel >= DEBUG_CHANNEL_COUNT) return;
    DebugChannel_t *ch = &channel_map[channel];
    if (!ch->name || !ch->param_ptr) return;

    if (value < ch->min_val) value = ch->min_val;
    if (value > ch->max_val) value = ch->max_val;
    *(ch->param_ptr) = value;
    channel_map[channel] = *ch;
    {
        int32_t scaled = (int32_t)(value * DEBUG_PARAM_SCALE);
        *(int32_t *)&seekfree_assistant_parameter[channel] = scaled;
    }

    switch (current_group) {
    case DEBUG_GROUP_SPEED:
        if (channel <= 2) {
            MotorPID_SetGlobalParams(pid_kp, pid_ki, pid_kd);
        }
        if (g_ctrl.min_speed > g_ctrl.max_speed) {
            g_ctrl.min_speed = g_ctrl.max_speed;
        }
        break;

    case DEBUG_GROUP_ANGLE:
        if (channel <= 1) {
            AnglePID_SetParams(angle_trace_param.kp,
                               angle_trace_param.ki,
                               angle_trace_param.kd);
        }
        break;

    case DEBUG_GROUP_TRACKING:
        if (channel == 4 || channel == 5) {
            pos_y_param.kp = pos_x_param.kp;
            pos_y_param.kd = pos_x_param.kd;
        }
        break;
    }
}

/* ========== 打印函数 ========== */

static void print_status(void)
{
    char buf[128];
    rt_sprintf(buf, "[DBG] Group: %s\r\n", group_names[current_group]);
    wireless_uart_send_string(buf);

    for (int i = 0; i < DEBUG_CHANNEL_COUNT; i++) {
        if (channel_map[i].name && channel_map[i].param_ptr) {
            rt_sprintf(buf, "  ch%d %-16s = %.4f  [%.3f ~ %.3f]\r\n",
                       i, channel_map[i].name,
                       *(channel_map[i].param_ptr),
                       channel_map[i].min_val,
                       channel_map[i].max_val);
            wireless_uart_send_string(buf);
        }
    }
}

static void print_save(void)
{
    char buf[128];
    rt_sprintf(buf, "// --- %s group ---\r\n", group_names[current_group]);
    wireless_uart_send_string(buf);

    switch (current_group) {
    case DEBUG_GROUP_SPEED:
        rt_sprintf(buf, "MotorPID_SetGlobalParams(%.1ff, %.1ff, %.1ff);\r\n",
                   pid_kp, pid_ki, pid_kd);
        wireless_uart_send_string(buf);
        rt_sprintf(buf, "g_ctrl.max_speed = %.3ff;\r\n", g_ctrl.max_speed);
        wireless_uart_send_string(buf);
        rt_sprintf(buf, "g_ctrl.min_speed = %.3ff;\r\n", g_ctrl.min_speed);
        wireless_uart_send_string(buf);
        rt_sprintf(buf, "g_ctrl.push_speed_max = %.3ff;\r\n", g_ctrl.push_speed_max);
        wireless_uart_send_string(buf);
        break;

    case DEBUG_GROUP_ANGLE:
        rt_sprintf(buf, "AnglePID_SetParams(%.3ff, 0.0f, %.3ff);\r\n",
                   angle_trace_param.kp, angle_trace_param.kd);
        wireless_uart_send_string(buf);
        break;

    case DEBUG_GROUP_TRACKING:
        rt_sprintf(buf, "g_ctrl.lookahead_dist = %.3ff;\r\n", g_ctrl.lookahead_dist);
        wireless_uart_send_string(buf);
        rt_sprintf(buf, "g_ctrl.path_tolerance = %.3ff;\r\n", g_ctrl.path_tolerance);
        wireless_uart_send_string(buf);
        rt_sprintf(buf, "g_ctrl.min_lookahead = %.3ff;\r\n", g_ctrl.min_lookahead);
        wireless_uart_send_string(buf);
        rt_sprintf(buf, "g_ctrl.max_lookahead = %.3ff;\r\n", g_ctrl.max_lookahead);
        wireless_uart_send_string(buf);
        rt_sprintf(buf, "pos_x_param.kp = %.3ff; pos_x_param.kd = %.3ff;\r\n",
                   pos_x_param.kp, pos_x_param.kd);
        wireless_uart_send_string(buf);
        rt_sprintf(buf, "pos_y_param.kp = %.3ff; pos_y_param.kd = %.3ff;\r\n",
                   pos_y_param.kp, pos_y_param.kd);
        wireless_uart_send_string(buf);
        break;
    }
}

static void print_help(void)
{
    wireless_uart_send_string("=== Debug Commands ===\r\n");
    wireless_uart_send_string(" map <group>   - switch channel group: speed/angle/tracking\r\n");
    wireless_uart_send_string(" status        - show current group channels & values\r\n");
    wireless_uart_send_string(" save          - print C init code for current group\r\n");
    wireless_uart_send_string(" test straight <cm> - go straight, arg in cm\r\n");
    wireless_uart_send_string(" test square <cm>   - square path, side in cm\r\n");
    wireless_uart_send_string(" test nav <x_cm> <y_cm> - navigate to coords in cm\r\n");
    wireless_uart_send_string(" test empty   - load built-in empty map\r\n");
    wireless_uart_send_string(" test digit   - UART4 digit recognition test\r\n");
    wireless_uart_send_string(" test box     - UART4 box type recognition test\r\n");
    wireless_uart_send_string(" help         - show this help\r\n");
}

/* ========== 测试函数 ========== */

static void load_empty_map(void)
{
    if (g_map_mutex) rt_mutex_take(g_map_mutex, RT_WAITING_FOREVER);
    load_map_from_text(empty_map_text, &g_grid_map, &g_game_state);
    refresh_grid_map(&g_game_state, &g_grid_map);
    g_digit_map_count = 0;
    g_task_mgr.all_dest_recognized = 0;
    g_task_mgr.all_box_recognized = 0;
    if (g_map_mutex) rt_mutex_release(g_map_mutex);
    wireless_uart_send_string("[TEST] Empty map loaded.\r\n");
}

static void test_straight(float dist)
{
    if (g_ctrl.mode != CTRL_MODE_IDLE) {
        wireless_uart_send_string("[STRAIGHT] Controller busy.\r\n");
        return;
    }

    float start_x = position.x_m;
    float start_y = position.y_m;

    g_ctrl.current_path[0][0] = start_x;
    g_ctrl.current_path[0][1] = start_y;
    g_ctrl.current_path[1][0] = start_x + dist;
    g_ctrl.current_path[1][1] = start_y;
    g_ctrl.path_len = 2;
    g_ctrl.path_following = 1;
    g_ctrl.path_purpose = PATH_PURPOSE_MOVE_ACTION;
    g_ctrl.use_tangent_heading = 0;
    g_ctrl.path_locked_yaw = 0.0f;
    g_ctrl.max_speed = 0.15f;
    g_ctrl.min_speed = 0.08f;
    g_ctrl.path_tolerance = 0.03f;
    g_ctrl.push_smoothed_speed = 0.0f;
    g_ctrl.axial_tracking = 0;
    PID_Reset(&angle_trace_param);

    float vx, vy, omega, dist_to_end;
    int ret;
    const uint32_t timeout_ms = 15000;
    uint32_t start_tick = rt_tick_get();

    char buf[64];
    rt_sprintf(buf, "[STRAIGHT] Start from (%.3f,%.3f) go %.2f m\r\n",
               start_x, start_y, dist);
    wireless_uart_send_string(buf);

    do {
        ret = follow_path(&g_ctrl, position.x_m, position.y_m, position.yaw_rad,
                          &vx, &vy, &omega, &dist_to_end);
        CarController_SetSpeed(vx, vy, 0.0f);
        CarController_Update();
        Position_Update();
        rt_thread_mdelay(5);

        if (rt_tick_get() - start_tick > timeout_ms) {
            wireless_uart_send_string("[STRAIGHT] Timeout!\r\n");
            break;
        }
    } while (!(ret && dist_to_end < g_ctrl.path_tolerance));

    CarController_Stop();
    CarController_Update();

    float actual = position.x_m - start_x;
    rt_sprintf(buf, "[STRAIGHT] target=%.3f actual=%.3f error=%.3f\r\n",
               dist, actual, actual - dist);
    wireless_uart_send_string(buf);

    g_ctrl.mode = CTRL_MODE_IDLE;
    g_ctrl.path_following = 0;
}

static void test_square(float side)
{
    if (g_ctrl.mode != CTRL_MODE_IDLE) {
        wireless_uart_send_string("[SQUARE] Controller busy.\r\n");
        return;
    }
    if (side <= 0.0f) {
        wireless_uart_send_string("[SQUARE] Invalid side length.\r\n");
        return;
    }

    float start_x = position.x_m;
    float start_y = position.y_m;
    float c1_x = start_x,         c1_y = start_y;
    float c2_x = start_x + side,  c2_y = start_y;
    float c3_x = start_x + side,  c3_y = start_y + side;
    float c4_x = start_x,         c4_y = start_y + side;
    float c5_x = start_x,         c5_y = start_y;

    float step = 0.1f;
    int total_points = 0;
    float path[MAX_PATH_POINTS][2];

    #define ADD_POINT(x, y) do { \
        if (total_points < MAX_PATH_POINTS) { \
            path[total_points][0] = (x); \
            path[total_points][1] = (y); \
            total_points++; \
        } \
    } while(0)

    float dx, dy, len;
    int steps;

    dx = c2_x - c1_x; dy = c2_y - c1_y;
    len = sqrtf(dx*dx + dy*dy);
    steps = (int)(len / step); if (steps < 1) steps = 1;
    for (int i = 0; i <= steps; i++) {
        ADD_POINT(c1_x + dx * i / steps, c1_y + dy * i / steps);
    }

    dx = c3_x - c2_x; dy = c3_y - c2_y;
    len = sqrtf(dx*dx + dy*dy);
    steps = (int)(len / step); if (steps < 1) steps = 1;
    for (int i = 1; i <= steps; i++) {
        ADD_POINT(c2_x + dx * i / steps, c2_y + dy * i / steps);
    }

    dx = c4_x - c3_x; dy = c4_y - c3_y;
    len = sqrtf(dx*dx + dy*dy);
    steps = (int)(len / step); if (steps < 1) steps = 1;
    for (int i = 1; i <= steps; i++) {
        ADD_POINT(c3_x + dx * i / steps, c3_y + dy * i / steps);
    }

    dx = c5_x - c4_x; dy = c5_y - c4_y;
    len = sqrtf(dx*dx + dy*dy);
    steps = (int)(len / step); if (steps < 1) steps = 1;
    for (int i = 1; i <= steps; i++) {
        ADD_POINT(c4_x + dx * i / steps, c4_y + dy * i / steps);
    }

    #undef ADD_POINT

    if (total_points < 2) {
        wireless_uart_send_string("[SQUARE] Not enough path points.\r\n");
        return;
    }

    for (int i = 0; i < total_points; i++) {
        g_ctrl.current_path[i][0] = path[i][0];
        g_ctrl.current_path[i][1] = path[i][1];
    }
    g_ctrl.path_len = total_points;
    g_ctrl.path_following = 1;
    g_ctrl.path_purpose = PATH_PURPOSE_MOVE_ACTION;
    g_ctrl.use_tangent_heading = 0;
    g_ctrl.path_locked_yaw = 0.0f;
    g_ctrl.push_smoothed_speed = 0.0f;
    g_ctrl.axial_tracking = 0;
    PID_Reset(&angle_trace_param);

    wireless_uart_send_string("[SQUARE] Start square path...\r\n");

    float vx, vy, omega, dist_to_end;
    int ret;
    const uint32_t timeout_ms = 60000;
    uint32_t start_tick = rt_tick_get();

    do {
        ret = follow_path(&g_ctrl, position.x_m, position.y_m, position.yaw_rad,
                          &vx, &vy, &omega, &dist_to_end);
        CarController_SetSpeed(vx, vy, 0.0f);
        CarController_Update();
        Position_Update();
        rt_thread_mdelay(5);

        if (rt_tick_get() - start_tick > timeout_ms) {
            wireless_uart_send_string("[SQUARE] Timeout!\r\n");
            break;
        }
    } while (!(ret && dist_to_end < g_ctrl.path_tolerance));

    CarController_Stop();
    CarController_Update();

    float end_x = position.x_m;
    float end_y = position.y_m;
    float dx_err = end_x - start_x;
    float dy_err = end_y - start_y;
    float total_err = sqrtf(dx_err*dx_err + dy_err*dy_err);

    char buf[128];
    rt_sprintf(buf, "[SQUARE] Start(%.3f,%.3f) End(%.3f,%.3f) Error=%.3f m\r\n",
               start_x, start_y, end_x, end_y, total_err);
    wireless_uart_send_string(buf);

    g_ctrl.mode = CTRL_MODE_IDLE;
    g_ctrl.path_following = 0;
}

static void test_nav(float x, float y)
{
    if (g_ctrl.mode != CTRL_MODE_IDLE) {
        wireless_uart_send_string("[NAV] Controller busy.\r\n");
        return;
    }

    if (!HybridController_PlanPathToPoint(&g_ctrl, position.x_m, position.y_m, x, y)) {
        wireless_uart_send_string("[NAV] Path planning failed.\r\n");
        return;
    }

    wireless_uart_send_string("[NAV] Navigating...\r\n");

    float vx, vy, omega, dist_to_end;
    int ret;
    const uint32_t timeout_ms = 30000;
    uint32_t start_tick = rt_tick_get();

    do {
        ret = follow_path(&g_ctrl, position.x_m, position.y_m, position.yaw_rad,
                          &vx, &vy, &omega, &dist_to_end);
        CarController_SetSpeed(vx, vy, 0.0f);
        CarController_Update();
        Position_Update();
        rt_thread_mdelay(5);

        if (rt_tick_get() - start_tick > timeout_ms) {
            wireless_uart_send_string("[NAV] Timeout!\r\n");
            break;
        }
    } while (!(ret && dist_to_end < g_ctrl.path_tolerance));

    CarController_Stop();
    CarController_Update();

    char buf[128];
    rt_sprintf(buf, "[NAV] Arrived. Target(%.3f,%.3f) Actual(%.3f,%.3f)\r\n",
               x, y, position.x_m, position.y_m);
    wireless_uart_send_string(buf);

    g_ctrl.mode = CTRL_MODE_IDLE;
    g_ctrl.path_following = 0;
}

static void test_digit_recog(void)
{
    int digit = -1;
    if (uart4_request_digit(&digit)) {
        char buf[64];
        rt_sprintf(buf, "[DIGIT] Recognized: %d\r\n", digit);
        wireless_uart_send_string(buf);
    } else {
        wireless_uart_send_string("[DIGIT] Failed or timeout.\r\n");
    }
}

static void test_box_recog(void)
{
    BoxTypeEnum_t type;
    if (uart4_request_box_type(&type)) {
        char buf[64];
        rt_sprintf(buf, "[BOX] Type: %d\r\n", (int)type);
        wireless_uart_send_string(buf);
    } else {
        wireless_uart_send_string("[BOX] Failed or timeout.\r\n");
    }
}

/* ========== 命令解析 ========== */

static void execute_command(char *line)
{
    char *cmd = strtok(line, " \r\n");
    if (!cmd || cmd[0] == '\0') return;

    if (strcmp(cmd, "map") == 0) {
        char *grp = strtok(NULL, " \r\n");
        if (!grp) {
            wireless_uart_send_string("[DBG] Usage: map <speed|angle|tracking>\r\n");
            return;
        }
        int found = 0;
        for (int g = 0; g < DEBUG_GROUP_COUNT; g++) {
            if (strcmp(grp, group_names[g]) == 0) {
                map_group((DebugGroup_t)g);
                char buf[64];
                rt_sprintf(buf, "[DBG] Switched to group: %s\r\n", group_names[g]);
                wireless_uart_send_string(buf);
                print_status();
                found = 1;
                break;
            }
        }
        if (!found) {
            wireless_uart_send_string("[DBG] Unknown group. Use: speed/angle/tracking\r\n");
        }
    } else if (strcmp(cmd, "status") == 0) {
        print_status();
    } else if (strcmp(cmd, "save") == 0) {
        print_save();
    } else if (strcmp(cmd, "help") == 0) {
        print_help();
    } else if (strcmp(cmd, "test") == 0) {
        char *sub = strtok(NULL, " \r\n");
        if (!sub) {
            wireless_uart_send_string("Usage: test <straight|square|nav|empty|digit|box>\r\n");
            return;
        }
        if (strcmp(sub, "straight") == 0) {
            char *arg = strtok(NULL, " \r\n");
            if (arg) test_straight((float)atoi(arg) / 100.0f);
            else wireless_uart_send_string("Usage: test straight <cm>\r\n");
        } else if (strcmp(sub, "square") == 0) {
            char *arg = strtok(NULL, " \r\n");
            if (arg) test_square((float)atoi(arg) / 100.0f);
            else wireless_uart_send_string("Usage: test square <cm>\r\n");
        } else if (strcmp(sub, "nav") == 0) {
            char *x_str = strtok(NULL, " \r\n");
            char *y_str = strtok(NULL, " \r\n");
            if (x_str && y_str) test_nav((float)atoi(x_str) / 100.0f, (float)atoi(y_str) / 100.0f);
            else wireless_uart_send_string("Usage: test nav <x_cm> <y_cm>\r\n");
        } else if (strcmp(sub, "empty") == 0) {
            load_empty_map();
        } else if (strcmp(sub, "digit") == 0) {
            test_digit_recog();
        } else if (strcmp(sub, "box") == 0) {
            test_box_recog();
        } else {
            wireless_uart_send_string("Unknown test. Available: straight, square, nav, empty, digit, box\r\n");
        }
    } else {
        char buf[64];
        rt_sprintf(buf, "[DBG] Unknown cmd: '%s'. Type 'help'.\r\n", cmd);
        wireless_uart_send_string(buf);
    }
}

/* ========== 回调拦截器：分离文本命令和二进制帧 ========== */

static uint32 debug_filtered_receive(uint8 *buff, uint32 length)
{
    if (!saved_receive_callback) return 0;

    uint32 n = saved_receive_callback(buff, length);
    if (n == 0) return 0;

    static char line[128];
    static uint8 idx = 0;
    uint32 write_pos = 0;

    for (uint32 i = 0; i < n; i++) {
        uint8_t c = buff[i];
        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                line[idx] = '\0';
                if (!cmd_queue_full()) {
                    cmd_queue_push(line);
                }
                idx = 0;
            }
        } else if (c >= 0x20 && c <= 0x7E) {
            if (idx < (int)sizeof(line) - 1) {
                line[idx++] = c;
            }
        } else {
            buff[write_pos++] = c;
        }
    }

    return write_pos;
}

/* ========== 命令处理（调试线程调用） ========== */

void debug_process_commands(void)
{
    char line[DEBUG_CMD_LINE_MAX];
    while (cmd_queue_pop(line)) {
        execute_command(line);
    }
}

/* ========== SeekFree 参数轮询（控制线程调用） ========== */

void debug_seekfree_loop(void)
{
    seekfree_assistant_data_analysis();

    for (uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++) {
        if (seekfree_assistant_parameter_update_flag[i]) {
            seekfree_assistant_parameter_update_flag[i] = 0;
            float val = (float)(*(int32_t *)&seekfree_assistant_parameter[i]) / DEBUG_PARAM_SCALE;
            debug_apply_parameter(i, val);

            char msg[64];
            rt_sprintf(msg, "[DBG] ch%d -> %.4f (%s)\r\n",
                       i, val, channel_map[i].name ? channel_map[i].name : "?");
            wireless_uart_send_string(msg);
        }
    }
}

/* ========== 调试执行线程 ========== */

void debug_thread_entry(void *parameter)
{
    wireless_uart_send_string("[DBG-THREAD] Debug exec thread started.\r\n");
    while (1) {
        debug_process_commands();
        rt_thread_mdelay(10);
    }
}

/* ========== 模块初始化 ========== */

void debug_module_init(void)
{
    saved_receive_callback = seekfree_assistant_receive_callback;
    seekfree_assistant_receive_callback = debug_filtered_receive;

    map_group(DEBUG_GROUP_SPEED);

    rt_thread_t dbg_thread = rt_thread_create("debug_exec",
                                              debug_thread_entry,
                                              RT_NULL,
                                              4096,
                                              RT_THREAD_PRIORITY_MAX - 1,
                                              20);
    if (dbg_thread) {
        rt_thread_startup(dbg_thread);
    }

    char buf[64];
    rt_sprintf(buf, "Debug module ready. Group: %s. Type 'help'\r\n",
               group_names[current_group]);
    wireless_uart_send_string(buf);
    print_status();
}

#else

#include "debug.h"
void debug_module_init(void) {}
void debug_process_commands(void) {}
void debug_apply_parameter(uint8_t channel, float value) {}
void debug_seekfree_loop(void) {}
void debug_thread_entry(void *parameter) {}

#endif
