#include "ServoMotor.h"
#include "tim.h"
#include "protocol.h"

PID_StructTypedef pid;
float angle;

void PID_Init(float target, float kp, float ki, float kd, float max, float min)
{
    pid.target = target;
    pid.err = 0;
    pid.lasterr = 0;
    pid.preerr = 0;
    pid.result = 0;
    pid.integral = 0;
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    pid.beta = 0;
    pid.epsilon = 0;
    pid.increment = 0;
    pid.max = max;
    pid.min = min;

#if defined(PID_ASSISTANT_EN)
    float pid_temp[3] = {pid.Kp, pid.Ki, pid.Kd};
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3); // 给通道 1 发送 P I D 值
#endif
}

float PID_ControllerPos(float actual)
{
    pid.err = pid.target - actual;

    if (pid.result == pid.max)
    {
        if ((pid.err + pid.lasterr) < 0)
            pid.integral += (pid.err + pid.lasterr) / 2;
    }
    else if (pid.result == pid.min)
    {
        if ((pid.err + pid.lasterr) > 0)
            pid.integral += (pid.err + pid.lasterr) / 2;
    }
    else
        pid.integral += (pid.err + pid.lasterr) / 2;

    pid.result = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.lasterr);
    pid.lasterr = pid.err;
    if (pid.result > pid.max)
        pid.result = pid.max;
    if (pid.result < pid.min)
        pid.result = pid.min;

    return pid.result;
}

float PID_ControllerInc(float actual)
{
    pid.err = pid.target - actual;

    pid.integral = 0;
    if (pid.result > pid.max)
    {
        if ((pid.err + pid.lasterr) < 0)
            pid.integral = (pid.err + pid.lasterr) / 2;
    }
    else if (pid.result < pid.min)
    {
        if ((pid.err + pid.lasterr) > 0)
            pid.integral = (pid.err + pid.lasterr) / 2;
    }
    else
        pid.integral = pid.err;

    pid.increment = pid.Kp * (pid.err - pid.lasterr) + pid.Ki * pid.integral + pid.Kd * (pid.err - 2 * pid.lasterr + pid.preerr);
    pid.preerr = pid.lasterr;
    pid.lasterr = pid.err;
    pid.result += pid.increment;
    if (pid.result > pid.max)
        pid.result = pid.max;
    if (pid.result < pid.min)
        pid.result = pid.min;

    return pid.result;
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
