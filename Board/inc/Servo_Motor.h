/********************************S-D�������**********************************
*����   �� ɳ���Ѿ�
*�ļ��� �� Servo_Motor.c
*����   �� �������ͷ�ļ�
*ʱ��   �� 2015/10/30
*ʹ��˵���� 
              ��ɫ        GND
              ��ɫ        VCC
              ��ɫ        PWM
����Ƶ�ʣ�   300HZ    
*   
***************************************************************************/
#ifndef  _SERVO_MOTOR_H_
#define  _SERVO_MOTOR_H_

#include "include.h"



typedef struct servos_status
{
    float direction_p;                         //�������P����
    float direction_d;                         //�������D����
    int16  PID ;
    float direction_last_error;                //�����ϴ�ƫ��
    float diretion_current_error;             //����ǰƫ��
    int16 direction_duty_output;                  //����������ռ�ձ�
    
}servos_status;

extern servos_status servos;


//���²���ΪFTM_PRECISON 10000u    ]��ռ�ձȾ���Ϊ10000ʱ����
#define   Servo_freq    50       //c���������Ƶ��Ϊ100HZ b��300
#define   Servo_ftm     FTM1      //FTMģ��
#define   Servo_CH      FTM_CH0       //ͨ�� 


#define   Servo_mid     727//729
#define   Servo_lmax    647//649
#define   Servo_rmax    807//809



/*

CI��ģ���
����Ƶ��Ϊ50HZ
#define   Servo_mid     612     // ���� С��
#define   Servo_lmax    515     //
#define   Servo_rmax    715     //

����Ƶ��Ϊ100HZ
#define   Servo_mid     1224     // ���� С��
#define   Servo_lmax    1030     //
#define   Servo_rmax    1430     //

����Ƶ��Ϊ150HZ
#define   Servo_mid     1930     // ���� С��
#define   Servo_lmax    1640     //
#define   Servo_rmax    2225     //

C��ģ���
����Ƶ��Ϊ50HZ
#define   Servo_mid     748
#define   Servo_lmax    658
#define   Servo_rmax    828
*/

void Servo_Motor_init(void);
void direction_control();


#endif  