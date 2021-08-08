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
    uint8_t Enable;
} PID_StructTypedef;

extern PID_StructTypedef pid;
extern float angle;
extern uint8_t PID_EN;

void PID_Init(PID_StructTypedef *pid, float target, float kp, float ki, float kd);
float PID_ControllerPos(PID_StructTypedef *pid, float actual);
float PID_ControllerInc(PID_StructTypedef *pid, float actual);
void PID_SetPID(PID_StructTypedef *pid, float kp, float ki, float kd);
void PID_Enable(PID_StructTypedef *pid);
void PID_Reset(PID_StructTypedef *pid);

void Steering_SetAngle(float angle, uint32_t TIM_CHANNEL_x);
void Steering_SetDutyCycle(float dutycycle, uint32_t TIM_CHANNEL_x);

#endif
