/********************************S-D舵机驱动**********************************
*作者   ： 沙艺已觉
*文件名 ： Servo_Motor.c
*描述   ： 舵机驱动头文件
*时间   ： 2015/10/30
*使用说明： 
              黑色        GND
              红色        VCC
              白色        PWM
驱动频率：   300HZ    
*   
***************************************************************************/
#ifndef  _SERVO_MOTOR_H_
#define  _SERVO_MOTOR_H_

#include "include.h"



typedef struct servos_status
{
    float direction_p;                         //方向控制P参数
    float direction_d;                         //方向控制D参数
    int16  PID ;
    float direction_last_error;                //方向上次偏差
    float diretion_current_error;             //方向当前偏差
    int16 direction_duty_output;                  //方向控制输出占空比
    
}servos_status;

extern servos_status servos;


//以下参数为FTM_PRECISON 10000u    ]即占空比精度为10000时数据
#define   Servo_freq    50       //c车舵机驱动频率为100HZ b：300
#define   Servo_ftm     FTM1      //FTM模块
#define   Servo_CH      FTM_CH0       //通道 


#define   Servo_mid     727//729
#define   Servo_lmax    647//649
#define   Servo_rmax    807//809



/*

CI车模舵机
驱动频率为50HZ
#define   Servo_mid     612     // 大左 小右
#define   Servo_lmax    515     //
#define   Servo_rmax    715     //

驱动频率为100HZ
#define   Servo_mid     1224     // 大左 小右
#define   Servo_lmax    1030     //
#define   Servo_rmax    1430     //

驱动频率为150HZ
#define   Servo_mid     1930     // 大左 小右
#define   Servo_lmax    1640     //
#define   Servo_rmax    2225     //

C车模舵机
驱动频率为50HZ
#define   Servo_mid     748
#define   Servo_lmax    658
#define   Servo_rmax    828
*/

void Servo_Motor_init(void);
void direction_control();


#endif  