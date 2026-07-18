#ifndef __POSITION_H__
#define __POSITION_H__

#include "zf_common_headfile.h"
#include "pid.h"
#include "filter.h"

#define POSITION_TOLERANCE      0.02f
#define ANGLE_TOLERANCE         5.0f
#define SPEED_MAX               1.0f
#define SPEED_MAX_DPS           180.0f

typedef struct {
    float x_dis, y_dis, yaw;
    float speed_x, speed_y, speed_z;
    float x_target, y_target, yaw_target;
    float x_m, y_m, yaw_rad;
    uint8_t flag_dis_count;
    uint8_t position_reached;
    uint8_t angle_reached;
} Position_t;

extern Position_t position;
extern PIDParam_t angle_trace_param;
extern PIDParam_t pos_x_param, pos_y_param;
extern PIDParam_t angle_pid_nav;
extern PIDParam_t angle_pid_lateral;
extern float Kp_rot;

void Position_Init(void);
void Position_Update(void);
void Position_Set(float x_m, float y_m, float yaw_rad);
void Position_ResetYaw(void);
void AnglePID_SwitchMode(int mode);
void PositionPIDParamInit(void);
int RotateToAngleIMU(float target_angle);
void MoveToPositionIMU(float x_cm, float y_cm, float max_speed);
void AnglePID_SetParams(float kp, float ki, float kd);

#endif