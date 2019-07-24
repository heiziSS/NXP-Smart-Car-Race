/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       MK60_it.c
 * @brief      山外K60 平台中断服务函数
 * @author     山外科技
 * @version    v5.0
 * @date       2013-06-26
 */

#include    "MK60_it.h"
#include    "include.h"
//#include    "define.h"
#include    "extern.h"

extern reg_s ov7725_eagle_reg[49];   
extern IMG_STATUS_e  ov7725_eagle_img_flag;   //图像状态 
extern int speed;
uint8 relen;
int8  res[3];
int16 Time;
uint16 PIT2_Counter=0;


void  PIT0_IRQHandler(void)  
{
  
    PIT_Flag_Clear(PIT0); 
    DMA_Speed_get();
    
}  
 
//void PIT2_IRQHandler(void)
//{
//    PIT2_Counter++;
//    if(PIT2_Counter==1000 || HX_STATE == HX_NORMAL)
//    {
//    	disable_irq (PIT2_IRQn);
//	HX_STATE = HX_NORMAL;
//	PIT2_Counter = 0;
//    }
//    PIT_Flag_Clear(PIT2); 
//}
/***************************************************
ZigBee串口通信中断
****************************************************/
//void uart5_handler(void)
//{
//    uint8 temp;
//    UARTn_e uratn = UART5;
//    if(UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK)   //接收数据寄存器满
//    {  
////       uart_querystr (UART5,res,3);
////       LED_PrintValueC(80, 0,res[0]);
////       LED_PrintValueC(80, 1,res[1]);
////       LED_PrintValueC(80, 2,res[2]);
////        //用户需要处理接收数据
////       if(res[0]==59 && res[1]==95)
////       {
////         beep();
////         if(res[2]==1)
//           Car_Stop_flag = 1;
////       }
//
//    temp = UART_D_REG(UARTN[uratn]); 
//    temp++;
//    }
//    if(UART_S1_REG(UARTN[uratn]) & UART_S1_TDRE_MASK )  //发送数据寄存器空
//    {
//        //用户需要处理发送数据
//
//    }
//}
/******************************************************
* @author :沙艺已觉
* @function name : PORTC_Handler()
* @ data : 2016/11/15
* @function description : 场中断服务函数
******************************************************/
void DMA0_IRQHandler()
{
    camera_dma();
}

void PORTC_IRQHandler(void)
{
//    static uint8 PTA29_count=0;  
    uint8  n;    //引脚号
    uint32 flag;
    flag = PORTC_ISFR;
    PORTC_ISFR  = ~0;                                   //清中断标志位
//    if(!OLED_dis_flag)
//    {
      n=16;
      if(flag & (1 << n))                                 //PTA29触发中断
      {
	  camera_vsync();
      }
//    }
    //鹰眼直接全速采集，不需要行中断
    /*
    if(OLED_dis_flag)
    {
      n=6;
      if(flag & (1 << n)) 				//PTC6触发中断 加阈值
      {
	DELAY_MS(1);//消抖
	if(gpio_get(PTC6)==0)
	{
	  beep();
	  /////////////////////增加阈值//////////////////////
	  ov7725_eagle_reg[45].val += 1;
	  SCCB_WriteByte(ov7725_eagle_reg[45].addr, ov7725_eagle_reg[45].val);
	}
      }
      n=7;
      if(flag & (1 << n)) 				//PTC7触发中断 减阈值
      {
	DELAY_MS(1);//消抖
	if(gpio_get(PTC7)==0)
	{
	  beep();
	  /////////////////////减少阈值//////////////////////
	  ov7725_eagle_reg[45].val -= 1;
	  SCCB_WriteByte(ov7725_eagle_reg[45].addr, ov7725_eagle_reg[45].val);
	}
      }
    }*/
    /*
    if(flag & (1 << n))                                 //PTD5触发中断
    {
      int8 out=1,out1=1,out2=1;
      int8 a=0;//'>'
      int8 b=5;
      float voltage;
      int16 c=Servo_mid;
      //beep();
      DELAY_MS(1);//消抖
      if(gpio_get(PTC6)==0)
      {
      while(!gpio_get(PTC6));//防止中断退出了
      LED_CLS();
      while(out==1)
      {
        out1=1;
      voltage=Battery_voltage();
      LED_P6x8Str(6,0,"Debug");
      LED_P6x8Str(6,1,"1:senddata");
      LED_P6x8Str(6,2,"2:adjust_Servo");
      LED_P6x8Str(6,3,"3:KP_KI_KD");
      LED_P6x8Str(6,4,"Powervoltage:");
      LED_PrintValueF(84,4,voltage,3);
      LED_P6x8Str(122,4,"V");
        switch(Key_Test())
        {
        case 1:
                LED_CLS();
                LED_P6x8Str(8,0,"Matlabimg    off");
                LED_P6x8Str(8,1,"oscilloscope off");
                LED_P6x8Str(8,2,"TFT_img      off");
                LED_P6x8Str(8,3,"OLED_dispiay off");
                while(out1==1)
                {
                    LED_P6x8Str(0,a,">");
                    switch(Key_Test())
                    {    
                      case 1:
                        if(a==0 && img_sendmatlab_flag==0)
                        {
                          img_sendmatlab_flag=1;
                          LED_P6x8Str(86,0,"on ");
                        }
                        else if(a==0 && img_sendmatlab_flag==1)
                        {
                          img_sendmatlab_flag=0;
                          LED_P6x8Str(86,0,"off");
                        }
                        ///////
                        if(a==1 && send_osc_flag==0)
                        {
                          send_osc_flag=1;
                          LED_P6x8Str(86,1,"on ");
                        }
                        else if(a==1 && send_osc_flag==1)
                        {
                          send_osc_flag=0;
                          LED_P6x8Str(86,1,"off ");
                        }
                        /////////
                        if(a==2 && TFTShow_img_flag==0)
                        {
                          TFTShow_img_flag=1;
                          LED_P6x8Str(86,2,"on ");
                        }
                        else if(a==2 && TFTShow_img_flag==1)
                        {
                          TFTShow_img_flag=0;
                          LED_P6x8Str(86,2,"off");
                        }
                        //////////
                        if(a==3 && OLED_dis_flag==0)
                        {
                          OLED_dis_flag=1;
                          LED_P6x8Str(86,3,"on ");
                        }
                        else if(a==3 && OLED_dis_flag==1)
                        {
                          OLED_dis_flag=0;
                          LED_P6x8Str(86,3,"off");
                        }                         
                        break;
                      case 2:
                        LED_P6x8Str(0,a," ");
                        a--;
                        if(a==-1)a=3;
                        break;
                      case 3:
                        LED_P6x8Str(0,a," ");
                        a++;
                        if(a==4)a=0;
                        break;
                      case 4:out1=0;LED_CLS();break;
                      default:break;
                    }
                }
          break;*/
/**********************************************************************************/
        /* case 2:
                        LED_CLS();
                        LED_P6x8Str(8,0,"Servo_mid");
                        LED_PrintValueI4(80,0,Servo_mid);
                        while(out1==1)
                        {
                           switch(Key_Test())
                           { 
                             case 1:
                             out2=1;
                             LED_CLS();
                             LED_P6x8Str(8,0,"b=");
                             LED_PrintValueI4(16,0,b);
                               while(out2==1)
                               {
                                 switch(Key_Test())
                                 {
                                    case 2:
                                    b++;
                                    LED_P6x8Str(8,0,"b===");
                                    LED_PrintValueI4(16,0,b);
                                    break;
                                    
                                    case 3:
                                    b--;
                                    LED_P6x8Str(8,0,"b=");
                                    LED_PrintValueI4(16,0,b);
                                    break;
                                    case 4:
                                    LED_P6x8Str(8,0,"Servo_mid");
                                    LED_PrintValueI4(80,0,c);
                                    out2=0;break;
                                    default:break;
                                 }
                               }
                             break;
                             case 2:
                               c += b;
                               LED_P6x8Str(8,0,"Servo_mid");
                               LED_PrintValueI4(80,0,c);
                               break;
                             
                             case 3:
                               c -= b;
                               LED_P6x8Str(8,0,"Servo_mid");
                               LED_PrintValueI4(80,0,c);
                               break;
                             
                             case 4:out1=0;LED_CLS();break;
                             default:break;
                           }
                           ftm_pwm_duty(Servo_ftm,Servo_CH,c);
                         }  
                         break;
*/
/**********************************************************************************/
      /*  case 3:
                        LED_CLS();
                        LED_P6x8Str(8,0,"OV7725_CNST:");
                        LED_PrintValueI4(80,0,(int)(ov7725_eagle_reg[45].val));
                        while(out1==1)
                        {
                           switch(Key_Test())
                           { 
                             case 1:
                             out2=1;
                             LED_CLS();
                             LED_P6x8Str(8,0,"b=");
                             LED_PrintValueI4(16,0,b);
                               while(out2==1)
                               {
                                 switch(Key_Test())
                                 {
                                    case 2:
                                    b++;
                                    LED_P6x8Str(8,0,"b===");
                                    LED_PrintValueI4(16,0,b);
                                    break;
                                    
                                    case 3:
                                    b--;
                                    LED_P6x8Str(8,0,"b=");
                                    LED_PrintValueI4(16,0,b);
                                    break;
                                    case 4:
                                    LED_P6x8Str(8,0,"OV7725_CNST:");
                                    LED_PrintValueI4(80,0,(int)(ov7725_eagle_reg[45].val));
                                    out2=0;break;
                                    default:break;
                                 }
                               }
                             break;
                             case 2:
                               (ov7725_eagle_reg[45].val) += b;
                               LED_P6x8Str(8,0,"OV7725_CNST:");
                               LED_PrintValueI4(80,0,(int)(ov7725_eagle_reg[45].val));
                               break;
                             
                             case 3:
                               (ov7725_eagle_reg[45].val) -= b;
                               LED_P6x8Str(8,0,"OV7725_CNST:");
                               LED_PrintValueI4(80,0,(int)(ov7725_eagle_reg[45].val));
                               break;
                             
                             case 4:out1=0;LED_CLS();break;
                             default:break;
                           } 
                         } 
                         SCCB_WriteByte(ov7725_eagle_reg[45].addr, ov7725_eagle_reg[45].val);
//               LED_P6x8Str(6,0,"Servo_KP:");
//               LED_P6x8Str(6,1,"Servo_KD:");
//               LED_P6x8Str(6,2,"Speed_KP:");
//               LED_P6x8Str(6,3,"Speed_KI:");
//               a=0;
//               while(out1==1)
//               {
//                 LED_P6x8Str(0,a,">");
//                 LED_PrintValueF(60,0,servos.direction_p,1);
//                 LED_PrintValueF(60,1,servos.direction_d,1);
//                 LED_PrintValueF(60,2,motor.speed_p,1);
//                 LED_PrintValueF(60,3,motor.speed_i,1);
//                 switch(Key_Test())
//                 { 
//                   case 1:
//                     LED_P6x8Str(0,a," ");
//                     a++;
//                     if(a==4)a=0;
//                     break;
//                   case 2: 
//                     switch(a)
//                     {
//                     case 0:servos.direction_p=servos.direction_p+0.1;break;
//                     case 1:servos.direction_d=servos.direction_d+0.1;break;
//                     case 2:motor.speed_p=motor.speed_p+0.1;break;  
//                     case 3:motor.speed_i=motor.speed_i+0.1;break;
//                     default:break;
//                     }
//                     break;
//                   case 3: 
//                     switch(a)
//                     {
//                     case 0:servos.direction_p=servos.direction_p-0.1;break;
//                     case 1:servos.direction_d=servos.direction_d-0.1;break;
//                     case 2:motor.speed_p=motor.speed_p-0.1;break;  
//                     case 3:motor.speed_i=motor.speed_i-0.1;break;
//                     default:break;
//                     }
//                     break;
//                   case 4:out1=0;LED_CLS();break;  
//                   default:break;
//                 }
//               }
               
          break;*/
/**********************************************************************************/
     /*   case 4:out=0;LED_CLS();break;//跳出中断
        default: break;
        }
      }
    }
    }*/
}