#include <rtthread.h>
#include "debug_display.h"
#include "zf_device_ips200.h"
#include "planner.h"          // 包含 GridMap, GameState, 以及 CELL_SIZE 等常量
#include "position.h"         // 包含 Position_t 定义
#include "hybrid_controller.h"
// 外部全局变量（由主程序定义）
extern GridMap g_grid_map;
extern GameState g_game_state;
extern Position_t position;

// 屏幕参数（根据实际接线配置）
#define SCREEN_WIDTH  320     // 横屏宽度
#define SCREEN_HEIGHT 240     // 横屏高度

// 地图绘制区域（左上角坐标，预留右侧和底部显示文本）
#define MAP_AREA_X    20
#define MAP_AREA_Y    20
#define MAP_AREA_W    200     // 地图区域宽度（像素）
#define MAP_AREA_H    150     // 地图区域高度（像素）

// 粗网格尺寸
#define MAP_COLS      16
#define MAP_ROWS      12

// 每个网格在屏幕上的像素大小
#define CELL_PIX_W    (MAP_AREA_W / MAP_COLS)
#define CELL_PIX_H    (MAP_AREA_H / MAP_ROWS)

// 颜色定义（RGB565）
#define COLOR_WALL    RGB565_BLACK
#define COLOR_BOX     RGB565_RED
#define COLOR_DEST    RGB565_YELLOW
#define COLOR_BOMB    RGB565_PURPLE
#define COLOR_CAR     RGB565_GREEN
#define COLOR_TEXT    RGB565_WHITE
#define COLOR_BG      RGB565_BLACK

// 显示线程句柄
static rt_thread_t display_thread = RT_NULL;

// 绘制一个填充矩形（使用画点方式，简单但慢，可优化）
static void draw_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    for (uint16_t i = 0; i < w; i++) {
        for (uint16_t j = 0; j < h; j++) {
            ips200_draw_point(x + i, y + j, color);
        }
    }
}

// 绘制一个空心矩形
static void draw_hollow_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // 上下边
    for (uint16_t i = 0; i < w; i++) {
        ips200_draw_point(x + i, y, color);
        ips200_draw_point(x + i, y + h - 1, color);
    }
    // 左右边
    for (uint16_t j = 1; j < h - 1; j++) {
        ips200_draw_point(x, y + j, color);
        ips200_draw_point(x + w - 1, y + j, color);
    }
}

// 绘制地图网格（背景为黑色，墙体为黑色填充，箱子为红色填充，目的地为黄色空心，炸弹为紫色星号）
static void draw_map(void)
{
    // 清空地图区域
    draw_fill_rect(MAP_AREA_X, MAP_AREA_Y, MAP_AREA_W, MAP_AREA_H, COLOR_BG);

    // 绘制网格线（灰色细线）
    ips200_set_color(COLOR_TEXT, COLOR_BG);  // 设置画笔为白色，背景黑
    for (int i = 0; i <= MAP_COLS; i++) {
        uint16_t x = MAP_AREA_X + i * CELL_PIX_W;
        ips200_draw_line(x, MAP_AREA_Y, x, MAP_AREA_Y + MAP_AREA_H - 1, RGB565_GRAY);
    }
    for (int i = 0; i <= MAP_ROWS; i++) {
        uint16_t y = MAP_AREA_Y + i * CELL_PIX_H;
        ips200_draw_line(MAP_AREA_X, y, MAP_AREA_X + MAP_AREA_W - 1, y, RGB565_GRAY);
    }

    // 绘制墙体（直接读取 g_game_state 中的墙体列表）
    for (int i = 0; i < g_game_state.num_walls; i++) {
        Wall *w = &g_game_state.walls[i];
        // 墙体对应一个粗网格，中心坐标已知，需转换为行列
        int r = (int)((w->y1 + w->y2) / 2 / CELL_SIZE);
        int c = (int)((w->x1 + w->x2) / 2 / CELL_SIZE);
        uint16_t x = MAP_AREA_X + c * CELL_PIX_W;
        uint16_t y = MAP_AREA_Y + r * CELL_PIX_H;
        draw_fill_rect(x, y, CELL_PIX_W, CELL_PIX_H, COLOR_WALL);
    }

    // 绘制箱子
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        if (g_game_state.boxes[i].state == 0) {  // 未推动
            int r = (int)(g_game_state.boxes[i].y / CELL_SIZE);
            int c = (int)(g_game_state.boxes[i].x / CELL_SIZE);
            uint16_t x = MAP_AREA_X + c * CELL_PIX_W;
            uint16_t y = MAP_AREA_Y + r * CELL_PIX_H;
            draw_fill_rect(x, y, CELL_PIX_W, CELL_PIX_H, COLOR_BOX);
        }
    }

    // 绘制目的地
    for (int i = 0; i < g_game_state.num_destinations; i++) {
        int r = (int)(g_game_state.destinations[i].y / CELL_SIZE);
        int c = (int)(g_game_state.destinations[i].x / CELL_SIZE);
        uint16_t x = MAP_AREA_X + c * CELL_PIX_W;
        uint16_t y = MAP_AREA_Y + r * CELL_PIX_H;
        draw_hollow_rect(x, y, CELL_PIX_W, CELL_PIX_H, COLOR_DEST);
    }

    // 绘制炸弹（激活状态）
    for (int i = 0; i < g_game_state.num_bombs; i++) {
        if (g_game_state.bombs[i].active) {
            int r = (int)(g_game_state.bombs[i].y / CELL_SIZE);
            int c = (int)(g_game_state.bombs[i].x / CELL_SIZE);
            uint16_t x = MAP_AREA_X + c * CELL_PIX_W + CELL_PIX_W/2;
            uint16_t y = MAP_AREA_Y + r * CELL_PIX_H + CELL_PIX_H/2;
            // 画一个星形标记（简化为画叉）
            ips200_draw_line(x-3, y-3, x+3, y+3, COLOR_BOMB);
            ips200_draw_line(x-3, y+3, x+3, y-3, COLOR_BOMB);
        }
    }
}

// 绘制车辆位置和方向
static void draw_car(void)
{
    // 将世界坐标转换为网格行列
    int r = (int)(position.y_m / CELL_SIZE);
    int c = (int)(position.x_m / CELL_SIZE);
    if (r < 0 || r >= MAP_ROWS || c < 0 || c >= MAP_COLS) return;  // 超出地图

    uint16_t center_x = MAP_AREA_X + c * CELL_PIX_W + CELL_PIX_W/2;
    uint16_t center_y = MAP_AREA_Y + r * CELL_PIX_H + CELL_PIX_H/2;

    // 画车体（绿色圆点）
    for (int dx = -2; dx <= 2; dx++)
        for (int dy = -2; dy <= 2; dy++)
            if (dx*dx + dy*dy <= 4)
                ips200_draw_point(center_x + dx, center_y + dy, COLOR_CAR);

    // 画方向线（沿航向角画一条短线）
    float angle = position.yaw_rad;
    int line_len = 6;
    int end_x = center_x + line_len * cos(angle);
    int end_y = center_y + line_len * sin(angle);
    ips200_draw_line(center_x, center_y, end_x, end_y, COLOR_CAR);
}

// 绘制文本信息（屏幕下方或右侧）
static void draw_info(void)
{
    char buf[64];
    int y = MAP_AREA_Y + MAP_AREA_H + 5;

    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(0, y, "Mode: ");
    // 根据 g_ctrl.mode 显示模式字符串（需要引入 hybrid_controller.h）
    extern HybridController g_ctrl;
    const char *mode_str = "IDLE";
    switch (g_ctrl.mode) {
        case CTRL_MODE_PATH_FOLLOWING: mode_str = "PATH"; break;
        case CTRL_MODE_SOKOBAN_EXECUTING: mode_str = "SOKOBAN"; break;
        case CTRL_MODE_IDLE: default: mode_str = "IDLE"; break;
    }
    ips200_show_string(50, y, mode_str);

    y += 16;
    ips200_show_string(0, y, "Box: ");
    int pushed = 0;
    for (int i = 0; i < g_game_state.num_boxes; i++)
        if (g_game_state.boxes[i].state == 1) pushed++;
    sprintf(buf, "%d/%d", pushed, g_game_state.num_boxes);
    ips200_show_string(40, y, buf);

    y += 16;
    ips200_show_string(0, y, "Pos: ");
    sprintf(buf, "%.2f,%.2f", position.x_m, position.y_m);
    ips200_show_string(40, y, buf);

    y += 16;
    ips200_show_string(0, y, "Yaw: ");
    sprintf(buf, "%.1f", position.yaw_rad * 180.0 / M_PI);
    ips200_show_string(40, y, buf);
}

// 显示线程入口
static void display_thread_entry(void *param)
{
    ips200_init(IPS200_TYPE_SPI);        // 根据实际接口选择 SPI 或并口
    ips200_set_dir(IPS200_CROSSWISE);    // 横屏模式
    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_clear();

    while (1) {
        draw_map();
        draw_car();
        draw_info();

        rt_thread_mdelay(200);   // 5Hz 刷新
    }
}

// 初始化debug显示模块
void debug_display_init(void)
{
#if DEBUG_ENABLE
    if (display_thread == RT_NULL) {
        display_thread = rt_thread_create("debug_disp",
                                          display_thread_entry,
                                          RT_NULL,
                                          2048,          // 栈大小
                                          RT_THREAD_PRIORITY_MAX / 2 + 1,  // 低优先级
                                          20);
        if (display_thread != RT_NULL) {
            rt_thread_startup(display_thread);
            rt_kprintf("Debug display thread started\n");
        } else {
            rt_kprintf("Failed to create debug display thread\n");
        }
    }
#else
    rt_kprintf("Debug display is disabled\n");
#endif
}

// 逐飞助手调试接口（预留）
void debug_send_to_assistant(const uint8_t *data, uint32_t len)
{
    // TODO: 根据逐飞助手协议实现数据发送
    // 例如使用 UART 发送特定格式的数据包
    // uart_write_buffer(DEBUG_UART_INDEX, data, len);
}