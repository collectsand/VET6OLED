#include "usmart.h"
#include "usmart_str.h"
////////////////////////////用户配置区///////////////////////////////////////////////
//这下面要包含所用到的函数所申明的头文件(用户自己添加)

#include "ServoMotor.h"
#include "stdio.h"

/*  注意！！！
    参数只能是数字，且不能为小数，不能是工程中的变量名!!!
    参数只能是数字，且不能为小数，不能是工程中的变量名!!!
    参数只能是数字，且不能为小数，不能是工程中的变量名!!!
*/

// 像这种，达咩！！！
// void U_Set_PID(float kp, float ki, float kd)
// {
//     PID_SetPID(&pid, kp, ki, kd);
//     printf("kp%f\nki%f\nkd%f\r\n", pid.Kp, pid.Ki, pid.Kd);
// }

void U_Set_Angle(uint8_t angle)
{
    Steering_SetAngle(angle, TIM_CHANNEL_1);
}
//函数名列表初始化(用户自己添加)
//用户直接在这里输入要执行的函数名及其查找串

struct _m_usmart_nametab usmart_nametab[] =
    {
#if USMART_USE_WRFUNS == 1 //如果使能了读写操作
        (void *)read_addr,
        "uint32_t read_addr(uint32_t addr)",
        (void *)write_addr,
        "void write_addr(uint32_t addr,uint32_t val)",
        (void *)U_Set_Angle,
        "void U_Set_Angle(uint8_t angle)",

#endif
};
///////////////////////////////////END///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//函数控制管理器初始化
//得到各个受控函数的名字
//得到函数总数量
struct _m_usmart_dev usmart_dev =
    {
        usmart_nametab,
        usmart_init,
        usmart_cmd_rec,
        usmart_exe,
        usmart_scan,
        sizeof(usmart_nametab) / sizeof(struct _m_usmart_nametab), //函数数量
        0,                                                         //参数数量
        0,                                                         //函数ID
        1,                                                         //参数显示类型,0,10进制;1,16进制
        0,                                                         //参数类型.bitx:,0,数字;1,字符串
        0,                                                         //每个参数的长度暂存表,需要MAX_PARM个0初始化
        0,                                                         //函数的参数,需要PARM_LEN个0初始化
};
