/******************************************************************
*
*作者   ： 沙艺已觉
*文件名 ： extern.h
*描述   ： 全局变量头文件
*时间   ： 2015/11/2
*
*****************************************************************************/
#ifndef  _EXTERN_H_
#define  _EXTERN_H_

#include "include.h"
extern int8 StartPoint;
extern uint8 host_flag ;                       //前后车标志 0表示前车; 1表示后车
extern int8  IMG_WAIT_FLAG ;                          //档位
extern uint8 beep_flag ;
volatile extern uint8 img_switch_flag;                      //图像地址切换标志 
extern uint8 img_buffer[CAMERA_SIZE];              //图像缓存地址一
extern uint8 img_buffer2[CAMERA_SIZE];             //图像缓存地址二
extern uint8 img_handle[CAMERA_H][CAMERA_W];       //用于图像处理
extern uint8 srcimg[120][20];
extern int16 Lline[CAMERA_H];   //左右线数组
extern int16 Rline[CAMERA_H];
extern int16 Mline[CAMERA_H];
extern float value;

extern int16  Track_complexity ;                //赛道复杂程度
extern int8 Prospect_See ;                    //前瞻
extern int8 dajiao_Prospect_See;	       //打角前瞻
extern int8 Last_Prospect_See ;

extern uint8 Scratch_flag;//起跑线
extern uint8 Obstacle_flag;//障碍
extern int16 Obstacle_offset;
extern float Obstacle_fac;
extern uint8 Ramp_flag;

extern uint8 PID_Control_flag ;         //速度控制PID控制开启标志
extern uint8 Robust_Control_count;      //鲁棒控制计数
extern uint8 Robust_Control_flag;       //鲁棒控制标志
extern uint8 Robust_Control_Sub_flag ;  
extern uint8 Car_Stop_flag;             //停车标志
extern uint8 Car_Run_flag;
extern uint8 Car_start_line;
extern uint8 Blackline_flag;
extern int16 cartime;

extern uint8 LCDRAM[8][80];     //OLED显示

extern uint8 stop ;                     //蓝牙控制停车标志
extern uint8 oled_show_road_flag  ;      //oled是否开启标志
extern uint8 img_send_flag ;            //图像是否发送到上位机
extern uint8 img_save_flag ;            //图像是否用sd卡保存标志

extern uint8 send_osc_flag ;             //是否发送到虚拟示波器标志
extern uint8 img_send_flag ;            //图像是否发送到上位机
extern uint8 img_sendmatlab_flag;
extern uint8 TFTShow_img_flag;
extern uint8 OLED_dis_flag;
extern uint8 SUMMM;

extern int16 Time;
//extern float voltage ;

extern uint16 servo_stop_duty ;              //停车舵机停止打角

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