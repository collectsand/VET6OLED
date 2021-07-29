#ifndef __SERVOMOTOR_H
#define __SERVOMOTOR_H

#define PWM_TIM_HANDLE htim1
#define PWM_TIM_CHANNLE TIM_CHANNEL_1
#define PID_TIM_HANDLE htim4

#include "main.h"

typedef struct
{
    float target;
    float err, lasterr, preerr;
    float result;
    float integral;
    float Kp, Ki, Kd;
    float beta;
    float epsilon;
    float increment;
    float max, min;
} PID_StructTypedef;

extern PID_StructTypedef pid;
extern float angle;

void PID_Init(float target, float kp, float ki, float kd, float max, float min);
float PID_ControllerPos(float actual);
float PID_ControllerInc(float actual);

void Steering_SetAngle(float angle, uint32_t TIM_CHANNEL_x);
void Steering_SetDutyCycle(float dutycycle, uint32_t TIM_CHANNEL_x);

#endif
