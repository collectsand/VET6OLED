#ifndef __SERVOMOTOR_H
#define __SERVOMOTOR_H

#define PWM_TIM_HANDLE htim1
#define PWM_TIM_CHANNLE TIM_CHANNEL_1
#define PID_TIM_HANDLE htim4

#include "main.h"

typedef struct
{
    float target;
    float lasterr, preerr;
    float result;
    float integral;
    float derivative;
    float lastderivative;
    float Kp, Ki, Kd;
    float alpha;
    float deadband;
    float increment;
    float max, min;
} PID_StructTypedef;

extern PID_StructTypedef pid;
extern float angle;

void PID_Init(PID_StructTypedef *pid, float target, float kp, float ki, float kd);
float PID_ControllerPos(PID_StructTypedef *pid, float actual);
float PID_ControllerInc(PID_StructTypedef *pid, float actual);
float PID_VariableIntegral(float thiserr, float absmax, float absmin);
float PID_ControllerPos2(PID_StructTypedef *pid, float actual);

void Steering_SetAngle(float angle, uint32_t TIM_CHANNEL_x);
void Steering_SetDutyCycle(float dutycycle, uint32_t TIM_CHANNEL_x);

#endif
