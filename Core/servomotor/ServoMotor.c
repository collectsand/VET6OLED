#include "ServoMotor.h"
#include "tim.h"
#include "protocol.h"
#include "math.h"

PID_StructTypedef pid;
float angle;

void PID_Init(PID_StructTypedef *pid, float target, float kp, float ki, float kd)
{
    pid->target = target;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->alpha = 0.1;
    pid->deadband = 0.5;
    pid->max = 180.0;
    pid->min = 0.0;

    pid->lasterr = 0.0;
    pid->preerr = 0.0;
    pid->result = 0.0;
    pid->integral = 0.0;
    pid->derivative = 0.0;
    pid->lastderivative = 0.0;
    pid->increment = 0.0;
    pid->Enable = 0;
}

float PID_ControllerPos(PID_StructTypedef *pid, float actual)
{
    float thiserr, factor;

    thiserr = pid->target - actual;

    // 积分分离
    if (thiserr <= 70)
    {
        pid->integral += (thiserr + pid->lasterr) / 2.0;
    }

    // 不完全微分
    pid->derivative = pid->Kd * (1.0 - pid->alpha) * (thiserr - pid->lasterr) + pid->alpha * pid->lastderivative;

    // 计算
    pid->result = pid->Kp * thiserr + pid->Ki * pid->integral + pid->derivative;

    // 赋值
    pid->lasterr = thiserr;
    pid->lastderivative = pid->derivative;

    // 输出限幅
    if (pid->result > pid->max)
    {
        pid->result = pid->max;
    }
    if (pid->result < pid->min)
    {
        pid->result = pid->min;
    }

    return pid->result;
}

float PID_ControllerInc(PID_StructTypedef *pid, float actual)
{
    float thiserr, factor;

    thiserr = pid->target - actual;
    // 积分分离
    if (thiserr < 60)
    {
        pid->integral = thiserr;
    }

    // 不完全微分
    pid->derivative = pid->Kd * (1 - pid->alpha) * (thiserr - 2 * pid->lasterr + pid->preerr) + pid->alpha * pid->lastderivative;

    // 计算
    pid->increment = pid->Kp * (thiserr - pid->lasterr) + pid->Ki * pid->integral + pid->derivative;
    pid->result += pid->increment;

    // 赋值
    pid->preerr = pid->lasterr;
    pid->lasterr = thiserr;
    pid->lastderivative = pid->derivative;

    // 输出限幅
    if (pid->result > pid->max)
        pid->result = pid->max;
    if (pid->result < pid->min)
        pid->result = pid->min;

    return pid->result;
}

void PID_SetPID(PID_StructTypedef *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

void PID_Enable(PID_StructTypedef *pid)
{
    pid->Enable = 1;
}

void PID_Reset(PID_StructTypedef *pid)
{
    pid->Enable = 0;
}

void Steering_SetAngle(float angle, uint32_t TIM_CHANNEL_x)
{
    angle = (0.5 + angle / 180.0 * (2.5 - 0.5)) / 20.0 * PWM_PERIOD_COUNT; // 计算角度对应的占空比

    Steering_SetDutyCycle(angle, TIM_CHANNEL_x); // 设置占空比
}

void Steering_SetDutyCycle(float dutycycle, uint32_t TIM_CHANNEL_x)
{
    /* 对超过范围的占空比进行边界处理 */
    dutycycle = dutycycle < 0.5 / 20.0 * PWM_PERIOD_COUNT ? 0.5 / 20.0 * PWM_PERIOD_COUNT : dutycycle;
    dutycycle = dutycycle > 2.5 / 20.0 * PWM_PERIOD_COUNT ? 2.5 / 20.0 * PWM_PERIOD_COUNT : dutycycle;

    switch (TIM_CHANNEL_x)
    {
    case TIM_CHANNEL_1:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutycycle);
        break;
    case TIM_CHANNEL_2:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutycycle);
        break;
    case TIM_CHANNEL_3:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutycycle);
        break;
    }
}
