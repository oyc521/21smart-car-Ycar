#ifndef __POSITION_H__
#define __POSITION_H__

#include "zf_common_headfile.h"
#include "pid.h"
#include "filter.h"  // 添加滤波器头文件

// 位置控制参数
#define POSITION_TOLERANCE      0.02f    // 位置容差 2cm
#define ANGLE_TOLERANCE         5.0f     // 角度容差 5度
#define SPEED_MAX               1.0f     // 最大速度 1m/s
#define SPEED_MAX_DPS           180.0f   // 最大角速度 180°/s

// 位置结构体
typedef struct {
    float x_dis;            // X方向位移（cm）
    float y_dis;            // Y方向位移（cm）
    float yaw;              // 偏航角（度）
    
    float speed_x;          // X方向速度（cm/s）
    float speed_y;          // Y方向速度（cm/s）
    float speed_z;          // 旋转速度（度/s）
    
    float x_target;         // X方向目标位置（cm）
    float y_target;         // Y方向目标位置（cm）
    float yaw_target;       // 目标角度（度）
	
		float x_m;
		float y_m;
		float yaw_rad;
    
    uint8_t flag_dis_count; // 位移计数标志
    uint8_t position_reached; // 位置到达标志
    uint8_t angle_reached;    // 角度到达标志
} Position_t;

// 全局变量声明
extern Position_t position;
extern uint8_t disaim_flag;
extern float disaim_x_dis, disaim_y_dis;

// PID控制器声明
extern PIDParam_t line_trace_param, angle_trace_param;
extern PIDParam_t disaim_x_param, disaim_y_param;
extern PIDParam_t pos_x_param, pos_y_param;

// 函数声明
void Position_Init(void);                           // 位置控制器初始化
void Position_Update(void);                         // 更新位置信息（需定期调用）
void Position_SetTarget(float x_cm, float y_cm, float angle_deg);  // 设置目标位置
uint8_t Position_IsTargetReached(void);             // 检查是否到达目标位置

// 运动控制函数
void MoveToPosition(float x_cm, float y_cm, float max_speed);      // 移动到指定位置
void RotateToAngle(float target_angle, float max_speed_dps);       // 旋转到指定角度
void MoveStraight(float distance_cm, float speed_cmps);            // 直线移动
void MoveLateral(float distance_cm, float speed_cmps);             // 横向移动

// 循迹函数（兼容旧接口）
/*void TraceCenterLine(float speed);
void TraceLeftLine(float speed, float angle);
void TraceRightLine(float speed, float angle);
void AngleAimTo(float angle);
void DisAimTo(float x_aim, float y_aim);
*/

// 移动到目的地 i 附近
int move_to_destination(int dest_id);
// 移动到箱子 i 附近
int move_to_box(int box_id);
// PID参数初始化
void PositionPIDParamInit(void);

#endif /* __POSITION_H__ */