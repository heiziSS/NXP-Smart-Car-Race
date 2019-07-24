/******************************************************************
*
*����   �� ɳ���Ѿ�
*�ļ��� �� extern.h
*����   �� ȫ�ֱ���ͷ�ļ�
*ʱ��   �� 2015/11/2
*
*****************************************************************************/
#ifndef  _EXTERN_H_
#define  _EXTERN_H_

#include "include.h"
extern int8 StartPoint;
extern uint8 host_flag ;                       //ǰ�󳵱�־ 0��ʾǰ��; 1��ʾ��
extern int8  IMG_WAIT_FLAG ;                          //��λ
extern uint8 beep_flag ;
volatile extern uint8 img_switch_flag;                      //ͼ���ַ�л���־ 
extern uint8 img_buffer[CAMERA_SIZE];              //ͼ�񻺴��ַһ
extern uint8 img_buffer2[CAMERA_SIZE];             //ͼ�񻺴��ַ��
extern uint8 img_handle[CAMERA_H][CAMERA_W];       //����ͼ����
extern uint8 srcimg[120][20];
extern int16 Lline[CAMERA_H];   //����������
extern int16 Rline[CAMERA_H];
extern int16 Mline[CAMERA_H];
extern float value;

extern int16  Track_complexity ;                //�������ӳ̶�
extern int8 Prospect_See ;                    //ǰհ
extern int8 dajiao_Prospect_See;	       //���ǰհ
extern int8 Last_Prospect_See ;

extern uint8 Scratch_flag;//������
extern uint8 Obstacle_flag;//�ϰ�
extern int16 Obstacle_offset;
extern float Obstacle_fac;
extern uint8 Ramp_flag;

extern uint8 PID_Control_flag ;         //�ٶȿ���PID���ƿ�����־
extern uint8 Robust_Control_count;      //³�����Ƽ���
extern uint8 Robust_Control_flag;       //³�����Ʊ�־
extern uint8 Robust_Control_Sub_flag ;  
extern uint8 Car_Stop_flag;             //ͣ����־
extern uint8 Car_Run_flag;
extern uint8 Car_start_line;
extern uint8 Blackline_flag;
extern int16 cartime;

extern uint8 LCDRAM[8][80];     //OLED��ʾ

extern uint8 stop ;                     //��������ͣ����־
extern uint8 oled_show_road_flag  ;      //oled�Ƿ�����־
extern uint8 img_send_flag ;            //ͼ���Ƿ��͵���λ��
extern uint8 img_save_flag ;            //ͼ���Ƿ���sd�������־

extern uint8 send_osc_flag ;             //�Ƿ��͵�����ʾ������־
extern uint8 img_send_flag ;            //ͼ���Ƿ��͵���λ��
extern uint8 img_sendmatlab_flag;
extern uint8 TFTShow_img_flag;
extern uint8 OLED_dis_flag;
extern uint8 SUMMM;

extern int16 Time;
//extern float voltage ;

extern uint16 servo_stop_duty ;              //ͣ�����ֹͣ���

extern int8 current_ring_state;
extern int8 last_ring_state;
extern CIRCLE_status circle_flag;
extern int8 fork_flag;
extern uint8 HX_text;
extern uint8 Turn_state;//01010101
extern uint8 Current_Turn_state;
extern uint8 Texting_state;
extern CROSS_status cross_flag;
/************nrf*************/
extern uint8 nrf_rxbuff[DATA_PACKET];
extern uint8 nrf_txbuff[DATA_PACKET];

extern int srcnuma;
extern int srcnumb;
extern float chasu;
extern float var[6];


#endif