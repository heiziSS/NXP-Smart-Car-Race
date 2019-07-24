/**********************************全局变量定义****************************
*文件名：  define.h
*说明：   全局变量定义
*时间：    2015/11/21
**************************************************************************/


#ifndef _DEFINE_H_
#define _DEFINE_H_

#include "include.h"


uint8 beep_flag = 1;
uint8 a=0;
int8 StartPoint = 10;
int8  IMG_WAIT_FLAG = 0 ;                           //图像等待
volatile uint8 img_switch_flag = 0;                  //图像地址切换标志                        
uint8 img_buffer[CAMERA_SIZE];              //定义存储接收图像的数组
uint8 img_buffer2[CAMERA_SIZE];             //定义存储接收图像的另一数组    （乒乓算法实现交替存储）
uint8 img_handle[CAMERA_H][CAMERA_W];       //由于鹰眼摄像头是一字节8个像素，因而需要解压为 1字节1个像素，方便处理
uint8 srcimg[120][20];	
int16 Lline[CAMERA_H];   //左右线数组
int16 Rline[CAMERA_H];
int16 Mline[CAMERA_H];
float value=0;

int16 Track_complexity ;                //赛道复杂程度
int8 Prospect_See ;                    //前瞻 
int8 dajiao_Prospect_See;	       //打角前瞻
int8 Last_Prospect_See ;               //上一场的前瞻

uint8 Scratch_flag = 0;//起跑线
uint8 Obstacle_flag = 0;//障碍
int16 Obstacle_offset = 20;
float Obstacle_fac = 0;
uint8 Ramp_flag = 0;

uint8 Blackline_flag=0;
uint8 Car_start_line=0;
uint8 Car_Stop_flag = 0;                //停车标志
uint8 Car_Run_flag = 0;
int16 cartime=0;
uint8 Robust_Control_flag=0;            //鲁棒控制标志                
uint8 Robust_Control_count=0;

uint16 servo_stop_duty ;              //停车舵机停止打角
uint8 LCDRAM[8][80];

uint8 stop = 0;                       //蓝牙一键停车标志
uint8 oled_show_road_flag = 0 ;       //oled是否开启标志
uint8 img_send_flag = 0 ;             //图像是否发送到上位机
uint8 img_sendmatlab_flag = 0;
uint8 img_save_flag = 0 ;             //图像是否用sd卡保存标志
uint8 send_osc_flag = 0 ;             //是否发送到虚拟示波器标志
uint8 TFTShow_img_flag = 0 ;
uint8 OLED_dis_flag = 0 ;
uint8 SUMMM;                          //避障放大大小

float var[6];
//float voltage = 7.2;                          

extern IMG_STATUS_e  ov7725_eagle_img_flag;   //图像状态 

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

uint8 Current_Startline_state = 0 ;//起跑线状态记录
uint8 Last_Startline_state = 0 ;

int srcnuma=0,srcnumb=0;
float chasu;
#endif