#ifndef __INCLUDE_H__
#define __INCLUDE_H__
#define FIRSTCAR
#define   L_or_R 0x06//0000  0110
#include  "common.h"

/*
 * Include �û��Զ����ͷ�ļ�
 */
#include  "MK60_wdog.h"
#include  "MK60_gpio.h"      //IO�ڲ���
#include  "MK60_uart.h"      //����
#include  "MK60_SysTick.h"
#include  "MK60_lptmr.h"     //�͹��Ķ�ʱ��(��ʱ)
#include  "MK60_pit.h"       //PIT
#include  "MK60_FLASH.h"     //FLASH
#include  "MK60_FTM.h"       //FTM
#include  "MK60_sdhc.h"      //SDHC
#include  "MK60_spi.h"       //SPI
#include  "MK60_adc.h"       //ADC
#include  "MK60_dma.h"       //DMA
#include  "MK60_i2c.h"       //I2C
#include  "MK60_it.h"        //isr


#include  "VCAN_LED.H"                   //LED
#include  "VCAN_KEY.H"          //KEY
#include  "VCAN_SCCB.h"         //OV����ͷ��������SCCB������
#include  "VCAN_camera.h"       
#include  "VCAN_OV7725_REG.h"   
#include  "VCAN_NRF24L0.h"           
#include  "ff.h"                //FatFs
#include  "io.h"



#include  "FuzzySet_Speed.h"  //ģ���㷨�趨�ٶ�
#include  "FuzzySet_Casu.h"  
#include  "OLED.h"           //OLED0.96��LED
#include  "Servo_Motor.h"    //���ͷ�ļ�
#include  "Motor.h"          //�������ͷ�ļ�
#include  "System.h"         //ϵͳ��ʼ��
#include  "overtake.h"
#include  "lcd.h"
#include  "Key.h"
//#include  "Dis.h"
#include "Screen.h"
#include "mm.h"

#include  "extern.h"         //ȫ�ֱ�������            
#include  "VCAN_computer.h"     //�๦�ܵ�������


//#define  RED1 PTB0_IN
//#define  RED2 PTB1_IN
//#define  RED3 PTB2_IN
//#define  RED4 PTB3_IN

//���� ���뿪�� ����
#define BM1 gpio_get(PTB11)
#define BM2 gpio_get(PTB16)
#define BM3 gpio_get(PTB17)
#define BM4 gpio_get(PTB20)
#define BM5 gpio_get(PTB21)
#define BM6 gpio_get(PTB22)
#define BM7 gpio_get(PTB23)
#define BM8 gpio_get(PTC0)



//#define Servo_KP    0.72//0.82//0.56
//#define Servo_KD    0.58//0.58//0.65//0.64
//#define XS_Servo_KP 0.72 
//#define XS_Servo_KD 0.58



#endif  //__INCLUDE_H__
