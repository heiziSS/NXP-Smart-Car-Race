/********************************系统初始化**********************************
*
*作者   ： 沙艺已觉
*文件名 ： System.c
*描述   ： 系统初始化
*时间   ： 2015/11/9
说明    ：使用山外V5.3库
*
****************************************************************************/
#include "System.h"
#include "include.h"

void System_init()
{  
  float voltage;
  
        Tft_init();//显示屏初始化
        LCD_Clear(BLUE);//清屏
        OLED_init();
        //初始化摄像头
        camera_init(img_buffer);                   //这里设定  imgbuff 为采集缓冲区！！！！！！

//        NVIC_SetPriorityGrouping(4);             //配置中断服务函数      //设置优先级分组,4bit 抢占优先级,没有亚优先级
//        NVIC_SetPriority(PORTA_IRQn,0);          //配置优先级
//        NVIC_SetPriority(DMA0_IRQn,0);           //配置优先级
//        NVIC_SetPriority(PORTE_IRQn,1);          //配置优先级
//        NVIC_SetPriority(UART0_RX_TX_IRQn,1);    //配置优先级
        
//        NVIC_SetPriority(PIT0_IRQn,2);           //配置优先级
//        NVIC_SetPriority(PIT2_IRQn,3);           //配置优先级
         
        set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler);    //设置PORTA的中断服务函数为 PORTA_IRQHandler 摄像头场中断 不需要使能
        set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //设置DMA0的中断服务函数为 DMA0_IRQHandler  用于摄像头
        
//        uart_init(UART5,115200);                               //     
//        set_vector_handler(UART5_RX_TX_VECTORn,uart5_handler); //
//        uart_rx_irq_en (UART5);                                //
//
////        set_vector_handler(PORTD_VECTORn ,PORTD_IRQHandler);   //按键中断   
          Key_init();                                            //按键初始化 
////        enable_irq(PORTD_IRQn);                                //使能按键中断
//        
//        set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler);    //按键
//        enable_irq(PORTB_IRQn); //使能按键中断
//
        Buzzer_init();           //蜂鸣器初始化      
        Servo_Motor_init();       //舵机初始化
        Motor_init();            //电机初始化	       
        Led_init();
        ADC_init();
        BM_init();               //拨码开关初始化
        
        set_vector_handler(PIT0_VECTORn,PIT0_IRQHandler);
        enable_irq (PIT0_IRQn);    //允许PIT0定时器中断
        pit_init_ms(PIT0,13);     //初始化PIT0,定时时间为： 10ms
//        
////        pit_init_ms(PIT2, 1);              //初始化PIT0，定时时间为： 1ms
////	set_vector_handler(PIT2_VECTORn,PIT2_IRQHandler);//环形计时
//
//        set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);
//        enable_irq(PORTE_IRQn);
////        RED_init();               //光电对管
        Current_Turn_state = Turn_state & Texting_state;//判定Current_Turn_state是否为0来左右转；0左1右 
        Texting_state = Texting_state << 1;
        
        voltage = Battery_voltage();
        if(voltage < 7.4)
          didi();
        else 
          beep();
}

void ADC_init()
{
  adc_init(ADC0_DP1);       //power_adc   
}

float Battery_voltage()
{
        uint16 ad_value;    
        uint16 value[15];
        float  voltage;
        value[0] = adc_once (ADC0_DP1,ADC_12bit);
        value[1] = adc_once (ADC0_DP1,ADC_12bit);
        value[2] = adc_once (ADC0_DP1,ADC_12bit);
        value[3] = adc_once (ADC0_DP1,ADC_12bit);
        value[4] = adc_once (ADC0_DP1,ADC_12bit);
        value[5] = adc_once (ADC0_DP1,ADC_12bit);
        value[6] = adc_once (ADC0_DP1,ADC_12bit);
        value[7] = adc_once (ADC0_DP1,ADC_12bit);
        value[8] = adc_once (ADC0_DP1,ADC_12bit);
        value[9] = adc_once (ADC0_DP1,ADC_12bit);
        value[10] = adc_once(ADC0_DP1,ADC_12bit);
        value[11] = adc_once(ADC0_DP1,ADC_12bit);
        value[12] = adc_once(ADC0_DP1,ADC_12bit);
        value[13] = adc_once(ADC0_DP1,ADC_12bit);
        value[14] = adc_once(ADC0_DP1,ADC_12bit);
        ad_value=(value[0]+value[1]+value[2]+value[3]+value[4]+value[5]+value[6]+value[7]+value[8]+value[9]+value[10]+value[11]+value[12]+value[13]+value[14])/15;
//        LED_PrintValueI4(0,7,ad_value);
        voltage = (float)(ad_value)*0.002445; // 0.002360 = (3.3/2^12)*(300/100) 
        adc_stop(ADC0);
        return(voltage);              
}
void Led_init()
{
  led_init(LED0);
  led_init(LED1);
  led_init(LED2);
  led_init(LED3);
}
void Buzzer_init()
{
  gpio_init(PTE2,GPO,0);
}
void beep()
{
  gpio_set (PTE2, 1);
  DELAY_MS(50);
  gpio_set (PTE2, 0);
}
void didi()
{
  beep();
  DELAY_MS(100);
  beep();
}

void Key_init()
{
  gpio_init (KEY1, GPI,0);
  gpio_init (KEY2, GPI,0);
  gpio_init (KEY3, GPI,0);
  gpio_init (KEY4, GPI,0);
  
//  port_init(KEY1, IRQ_FALLING | PF | ALT1 |PULLUP);
//  port_init(KEY2, IRQ_FALLING | PF | ALT1 |PULLUP);
  port_init(KEY1, ALT1 |PULLUP);
  port_init(KEY2, ALT1 |PULLUP);
  port_init(KEY3, ALT1 |PULLUP);
  port_init(KEY4, ALT1 |PULLUP);
}


void BM_init()
{
  gpio_ddr (PTC0 , GPI); 
  gpio_ddr (PTB16, GPI);
  gpio_ddr (PTB17, GPI);
  gpio_ddr (PTB20, GPI);
  gpio_ddr (PTB21, GPI);
  gpio_ddr (PTB22, GPI);
  gpio_ddr (PTB23, GPI);
  gpio_ddr (PTB11, GPI);
  
  port_init(PTC0 ,ALT1 |PULLUP);
  port_init(PTB16,ALT1 |PULLUP);
  port_init(PTB17,ALT1 |PULLUP);
  port_init(PTB20,ALT1 |PULLUP);
  port_init(PTB21,ALT1 |PULLUP);
  port_init(PTB22,ALT1 |PULLUP);
  port_init(PTB23,ALT1 |PULLUP);
  port_init(PTB11,ALT1 |PULLUP);
}
//void beepms(uint8 ms)//使用中断
//{
//    if(beep_flag==1)
//    { 
//        BEEP = 1;
//      //  stop++;
//        beep_flag=0;   
//        pit_init_ms(PIT3,150);
//        enable_irq (PIT3_IRQn);
//    }
//}
//void RED_init()
//{
//  gpio_init(PTB0,GPI,LOW); //起跑红外
//  gpio_init(PTB1,GPI,LOW);
//  gpio_init(PTB2,GPI,LOW);
//  gpio_init(PTB3,GPI,LOW);
//}
/******************************************************
* @author : WWW
* @function name : OLED_Show_Road
* @ data : 2016/1/28
* @function description : oled显示赛道
******************************************************/
void OLED_Show_Road()
{
    uint8 i,j;
    uint8 left,right;
    uint8 mid;
    uint8 TLline[CAMERA_H];
    uint8 TRline[CAMERA_H];
    uint8 TMline[CAMERA_H]; 
      
    for(i=0;i<60;i++)
    {
        TLline[i]=(uint8)(Lline[i*2]/2);
        TRline[i]=(uint8)(Rline[i*2]/2);
        if( Mline[i*2] < 0)
         TMline[i] = 0 ;
        else if( Mline[i*2] > 159)
          TMline[i] = 79;
        else if( Mline[i*2] > 0 && Mline[i*2] <159 )
          TMline[i] = (uint8)(Mline[i*2]/2);
    }     
    //画出两条边界
   for(i=0;i<8;i++)
   {
     LED_Set_Pos(4,i);	     
     LED_WrDat(0xff);	
   }
   for(i=0;i<8;i++)
   {
     LED_Set_Pos(92,i);	     
     LED_WrDat(0xff);	
   }
   //清空显示区
   for(i=0;i<8;i++)
   {
     for(j=0;j<80;j++)
     {
       LCDRAM[i][j]=0;
     }
   }
 
   for(i=0;i<60;i++)
   {
     left=TLline[i];
     LCDRAM[i/8][left]=LCDRAM[i/8][left]|(0x01<<(i%8));
   }

   for(i=0;i<60;i++)
   {
     right=TRline[i];
     LCDRAM[i/8][right]=LCDRAM[i/8][right]|(0x01<<(i%8));
   }
   //存中线
     for(i=0;i<60;i++)
     {
      mid=TMline[i];
       LCDRAM[i/8][mid]=LCDRAM[i/8][mid]|(0x01<<(i%8));
     }   
   for(i=8;i>0;i--)
   {
     LED_Set_Pos(9,i-1);				
     for(j=0;j<80;j++)
     {      
         LED_WrDat(LCDRAM[i-1][j]);	    	
     }
   }
   LED_PrintValueF(96,0,camer.speed_control_error, 0);
}

void show_img()//120*160
{
    uint8 i,j;
     //清空显示区
   for(i=0;i<8;i++)
   {
     for(j=0;j<80;j++)
     {
       LCDRAM[i][j]=0;
     }
   }
  for(j=0;j<80;j++)
  {
    for(i=0;i<60;i++)
    {
        if(img_handle[i*2][j*2]==255)
           LCDRAM[i/8][j]=LCDRAM[i/8][j]|(0x01<<(i%8));
    }  
  }
     //显示
   for(i=8;i>0;i--)
   {
     LED_Set_Pos(9,i-1);				
     for(j=0;j<80;j++)
     {      
         LED_WrDat(LCDRAM[i-1][j]);	    	
     }
   }

}

