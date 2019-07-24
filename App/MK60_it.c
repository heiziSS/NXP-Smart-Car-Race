/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_it.c
 * @brief      ɽ��K60 ƽ̨�жϷ�����
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-06-26
 */

#include    "MK60_it.h"
#include    "include.h"
//#include    "define.h"
#include    "extern.h"

extern reg_s ov7725_eagle_reg[49];   
extern IMG_STATUS_e  ov7725_eagle_img_flag;   //ͼ��״̬ 
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
ZigBee����ͨ���ж�
****************************************************/
//void uart5_handler(void)
//{
//    uint8 temp;
//    UARTn_e uratn = UART5;
//    if(UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK)   //�������ݼĴ�����
//    {  
////       uart_querystr (UART5,res,3);
////       LED_PrintValueC(80, 0,res[0]);
////       LED_PrintValueC(80, 1,res[1]);
////       LED_PrintValueC(80, 2,res[2]);
////        //�û���Ҫ�����������
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
//    if(UART_S1_REG(UARTN[uratn]) & UART_S1_TDRE_MASK )  //�������ݼĴ�����
//    {
//        //�û���Ҫ����������
//
//    }
//}
/******************************************************
* @author :ɳ���Ѿ�
* @function name : PORTC_Handler()
* @ data : 2016/11/15
* @function description : ���жϷ�����
******************************************************/
void DMA0_IRQHandler()
{
    camera_dma();
}

void PORTC_IRQHandler(void)
{
//    static uint8 PTA29_count=0;  
    uint8  n;    //���ź�
    uint32 flag;
    flag = PORTC_ISFR;
    PORTC_ISFR  = ~0;                                   //���жϱ�־λ
//    if(!OLED_dis_flag)
//    {
      n=16;
      if(flag & (1 << n))                                 //PTA29�����ж�
      {
	  camera_vsync();
      }
//    }
    //ӥ��ֱ��ȫ�ٲɼ�������Ҫ���ж�
    /*
    if(OLED_dis_flag)
    {
      n=6;
      if(flag & (1 << n)) 				//PTC6�����ж� ����ֵ
      {
	DELAY_MS(1);//����
	if(gpio_get(PTC6)==0)
	{
	  beep();
	  /////////////////////������ֵ//////////////////////
	  ov7725_eagle_reg[45].val += 1;
	  SCCB_WriteByte(ov7725_eagle_reg[45].addr, ov7725_eagle_reg[45].val);
	}
      }
      n=7;
      if(flag & (1 << n)) 				//PTC7�����ж� ����ֵ
      {
	DELAY_MS(1);//����
	if(gpio_get(PTC7)==0)
	{
	  beep();
	  /////////////////////������ֵ//////////////////////
	  ov7725_eagle_reg[45].val -= 1;
	  SCCB_WriteByte(ov7725_eagle_reg[45].addr, ov7725_eagle_reg[45].val);
	}
      }
    }*/
    /*
    if(flag & (1 << n))                                 //PTD5�����ж�
    {
      int8 out=1,out1=1,out2=1;
      int8 a=0;//'>'
      int8 b=5;
      float voltage;
      int16 c=Servo_mid;
      //beep();
      DELAY_MS(1);//����
      if(gpio_get(PTC6)==0)
      {
      while(!gpio_get(PTC6));//��ֹ�ж��˳���
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
     /*   case 4:out=0;LED_CLS();break;//�����ж�
        default: break;
        }
      }
    }
    }*/
}