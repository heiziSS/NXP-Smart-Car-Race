/**********************************ȫ�ֱ�������****************************
*�ļ�����  define.h
*˵����   ȫ�ֱ�������
*ʱ�䣺    2015/11/21
**************************************************************************/


#ifndef _DEFINE_H_
#define _DEFINE_H_

#include "include.h"


uint8 beep_flag = 1;
uint8 a=0;
int8 StartPoint = 10;
int8  IMG_WAIT_FLAG = 0 ;                           //ͼ��ȴ�
volatile uint8 img_switch_flag = 0;                  //ͼ���ַ�л���־                        
uint8 img_buffer[CAMERA_SIZE];              //����洢����ͼ�������
uint8 img_buffer2[CAMERA_SIZE];             //����洢����ͼ�����һ����    ��ƹ���㷨ʵ�ֽ���洢��
uint8 img_handle[CAMERA_H][CAMERA_W];       //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��
uint8 srcimg[120][20];	
int16 Lline[CAMERA_H];   //����������
int16 Rline[CAMERA_H];
int16 Mline[CAMERA_H];
float value=0;

int16 Track_complexity ;                //�������ӳ̶�
int8 Prospect_See ;                    //ǰհ 
int8 dajiao_Prospect_See;	       //���ǰհ
int8 Last_Prospect_See ;               //��һ����ǰհ

uint8 Scratch_flag = 0;//������
uint8 Obstacle_flag = 0;//�ϰ�
int16 Obstacle_offset = 20;
float Obstacle_fac = 0;
uint8 Ramp_flag = 0;

uint8 Blackline_flag=0;
uint8 Car_start_line=0;
uint8 Car_Stop_flag = 0;                //ͣ����־
uint8 Car_Run_flag = 0;
int16 cartime=0;
uint8 Robust_Control_flag=0;            //³�����Ʊ�־                
uint8 Robust_Control_count=0;

uint16 servo_stop_duty ;              //ͣ�����ֹͣ���
uint8 LCDRAM[8][80];

uint8 stop = 0;                       //����һ��ͣ����־
uint8 oled_show_road_flag = 0 ;       //oled�Ƿ�����־
uint8 img_send_flag = 0 ;             //ͼ���Ƿ��͵���λ��
uint8 img_sendmatlab_flag = 0;
uint8 img_save_flag = 0 ;             //ͼ���Ƿ���sd�������־
uint8 send_osc_flag = 0 ;             //�Ƿ��͵�����ʾ������־
uint8 TFTShow_img_flag = 0 ;
uint8 OLED_dis_flag = 0 ;
uint8 SUMMM;                          //���ϷŴ��С

float var[6];
//float voltage = 7.2;                          

extern IMG_STATUS_e  ov7725_eagle_img_flag;   //ͼ��״̬ 

uint8 nrf_rxbuff[DATA_PACKET];
uint8 nrf_txbuff[DATA_PACKET];

int8 current_ring_state = 0;
int8 last_ring_state = 0;
uint8 HX_text=0;
CIRCLE_status circle_flag=NUCIRCLE;
int8 fork_flag = 0;

CROSS_status cross_flag=NUCROSS;

uint8 Turn_state = L_or_R;//01010101
uint8 Current_Turn_state = 0x00;
uint8 Texting_state = 0x01;

uint8 Current_Startline_state = 0 ;//������״̬��¼
uint8 Last_Startline_state = 0 ;

int srcnuma=0,srcnumb=0;
float chasu;
#endif