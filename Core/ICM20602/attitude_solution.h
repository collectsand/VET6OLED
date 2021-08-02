#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "main.h"

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} quaterInfo_t;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
} eulerianAngles_t;

extern quaterInfo_t Q_info;         // 全局四元数
extern eulerianAngles_t eulerAngle; //欧拉角
extern float values[10];
extern uint8_t bia1, bia2, bia3;

void IMU_getValues(float *values);
void IMU_quaterToEulerianAngles(void);

#endif
