/*!
*     COPYRIGHT NOTICE
*     Copyright (c) 2013,ɽ��Ƽ�
*     All rights reserved.
*     �������ۣ�ɽ����̳ http://www.vcan123.com
*
*     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
*     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
*
* @file       main.c
* @brief      ɽ��K60 ƽ̨������
* @author     ɽ��Ƽ�
* @version    v5.3
* @date       2015-04-07
*/

#include "include.h"
#include "define.h"
//FIRSTCAR
//extern uint8 s;
//int16 vag[1];
//static uint8 start_line_count=0;
float Speed_KP = 12.8;//11.2;//12.8
float Speed_KI = 0.8;//0.38
float HighSpeed_KP = 20.8;//18.2;//20.8
float HighSpeed_KI = 1.2;//0.64
void BM_value()
{//�ռ�������2.5
  if(0==BM1 && 0==BM2 && 0==BM3 && 0==BM4 && 0==BM5 && 0==BM8)
   {
    UFF_NORMAL = UFF0_NORMAL;
    UFF_HX = UFF0_HX;
    dajiao_Prospect_See = 78;
    servos.direction_p = 1.1;//0.68;
    servos.direction_d = 0.22;//0.58;
    Obstacle_fac = 1.2;
    UYY=UYY1;
  }
  if(1==BM1)//������2.9
  {
    UFF_NORMAL = UFF1_NORMAL;
    UFF_HX = UFF1_HX;
    dajiao_Prospect_See = 78;
    servos.direction_p = 1.1;//0.68;
    servos.direction_d = 0.25;//0.58;
    Obstacle_fac = 1.2;
    UYY=UYY1;
  }
  if(1==BM2)//���ȵ�3
  {
    UFF_NORMAL = UFF2_NORMAL;
    UFF_HX = UFF2_HX;
    dajiao_Prospect_See = 85;
    servos.direction_p = 1.1;//0.95;//0.9;
    servos.direction_d = 0.3;//1.1;//1.2;//0.62;
    Obstacle_fac = 1.28;
    UYY=UYY1;
  }
  if(1==BM3)//����3.2
  {
//    UFF_NORMAL = UFF3_NORMAL;
//    UFF_HX = UFF3_HX;
//    dajiao_Prospect_See = 86;
//    servos.direction_p = 1.1;//0.95;//0.9;
//    servos.direction_d = 0.3;//1.1;//0.75;
//    Obstacle_fac = 1.17;
    //�ȵ�
    UFF_NORMAL = UFF3_NORMAL;
    UFF_HX = UFF3_HX;
    dajiao_Prospect_See = 89;
    servos.direction_p = 1.1;//0.95;//0.9;
    servos.direction_d = 0.35;//1.32;//1.38;
    Obstacle_fac = 1.34;
    UYY=UYY1;
  }
  if(1==BM4)//�쵵3.3
  {
    UFF_NORMAL = UFF4_NORMAL;
    UFF_HX = UFF4_HX;
    dajiao_Prospect_See = 92;
    servos.direction_p = 1.1;//0.95;//0.9;
    servos.direction_d = 0.325;//1.32;//1.38;
    Obstacle_fac = 1.32;
    UYY=UYY1;
  }
  if(1==BM5)//���ε�3.5+
  {
    UFF_NORMAL = UFF5_NORMAL;
    UFF_HX = UFF5_HX;
    dajiao_Prospect_See = 94;
    servos.direction_p = 1.1;//0.95;//0.9;
    servos.direction_d = 0.32;//1.32;//1.38;
    Obstacle_fac = 1.35;
    UYY=UYY1;
  }
  if(1==BM8)//���ε�3.5+
  {
    UFF_NORMAL = UFF6_NORMAL;
    UFF_HX = UFF6_HX;
    dajiao_Prospect_See = 96;
    servos.direction_p = 1.1;//0.95;//0.9;
    servos.direction_d = 0.32;//1.32;//1.38;
    Obstacle_fac = 1.4;
    UYY=UYY2;
  }
}

void  main(void) 
{
  Turn_state = 0xD;//0000 1101
  System_init();
  
  motor.speed_p =Speed_KP;
  motor.speed_i =Speed_KI;
  
  motor.speed_filter_error[0] = 0;
  motor.speed_filter_error[1] = 0;
  motor.speed_filter_error[2] = 0;
  
  BM_value();
  
//  TFTShow_img_flag = BM6;
//  OLED_dis_flag = BM7;
//  if(TFTShow_img_flag+img_send_flag)    
//  {
//    motor.speed_p = 0;
//    motor.speed_i = 0;
//  }
  DELAY_MS(200);
  while(1)
 {
   led_turn(LED0);
   TFTShow_img_flag = BM6;
   OLED_dis_flag = BM7;
//    gpio_set(PTC14,1);
    camera_get_img();//��ȡͼ��
    
    if(img_switch_flag == 0)
        img_extract(img_handle,srcimg,img_buffer2,CAMERA_SIZE);
    else if(img_switch_flag == 1)
        img_extract(img_handle,srcimg,img_buffer,CAMERA_SIZE);

//    if(img_send_flag)//1 == send_osc_flag)
//    {
////       var[0] = camer.error;
//       var[1] = camer.error;
//       var[2] = servos.diretion_current_error-servos.direction_last_error;//motor.speed_set_R;
//       var[3] = motor.speed_set_L;//camer.error-camer.last_error;
//       var[4] = motor.speed_L;//HX_STATE;
//       var[5] = motor.speed_R;//motor.speed_set_R;
//       
//       vcan_sendware((uint8_t *)var, sizeof(var));
//    }
    
    Last_Prospect_See=Prospect_See;
    
    Get_Edge();
    
//    servos.direction_p = FuzzySet_steer_P(ABS((int16)camer.error), Prospect_See);
    
    if(TFTShow_img_flag+OLED_dis_flag)
    {
      led (LED3,LED_ON);
      display();
    }
    else
      led (LED3,LED_OFF);
    /*��λ������*/
//    if(BM8)
//    {
//      while(1)
//      {
//        if(img_switch_flag != 0)
//           vcan_sendimg(img_buffer, CAMERA_SIZE);                   //���͵���λ��
//        else
//           vcan_sendimg(img_buffer2, CAMERA_SIZE);                  //���͵���λ��
//      }
//    }
    direction_control();

    speed_control();
    
    while(ov7725_eagle_img_flag != IMG_FINISH)           //�ȴ�ͼ��ɼ����
    {
      if(ov7725_eagle_img_flag == IMG_FAIL)            //����ͼ��ɼ����������¿�ʼ�ɼ�
      {
        ov7725_eagle_img_flag = IMG_START;           //��ʼ�ɼ�ͼ��
        PORTC_ISFR = ~0;                             //д1���жϱ�־λ(����ģ���Ȼ�ص���һ���жϾ����ϴ����ж�)
        enable_irq(PORTC_IRQn);                      //����PTA���ж�
      }
    }
//    gpio_set(PTC14,0);
  }
}
//�������
//    ftm_pwm_duty(L_BACK_FTM,L_BACK_CH,400);
//    ftm_pwm_duty(L_GO_FTM,L_GO_CH,0);
//    ftm_pwm_duty(R_GO_FTM,R_GO_CH,0);
//    ftm_pwm_duty(R_BACK_FTM,R_BACK_CH,400);
//    DELAY_MS(3000);
//    ftm_pwm_duty(L_BACK_FTM,L_BACK_CH,0);
//    ftm_pwm_duty(L_GO_FTM,L_GO_CH,200);
//    ftm_pwm_duty(R_GO_FTM,R_GO_CH,200);
//    ftm_pwm_duty(R_BACK_FTM,R_BACK_CH,0);
//    DELAY_MS(3000);
//        LED_P6x8Str(0,5,"speed_L     =");
//        LED_PrintsignValueI4(80,5,(int)motor.speed_L);                   
//        LED_P6x8Str(0,6,"speed_R     =");
//        LED_PrintsignValueI4(80,6,(int)motor.speed_R);
//void  main(void)
//{
//  Motor_init();            //�����ʼ��
//  Servo_Motor_init();       //�����ʼ��
//  OLED_init();
//  ftm_pwm_duty(Servo_ftm,Servo_CH,Servo_mid-160);
//  while(1)
//  {
//    uint8 L,R;
//    uint32 speed_L,speed_R;
//    L=gpio_get(LEFT_ENCODER_DIRECTION);
//    R=gpio_get(RIGHT_ENCODER_DIRECTION);
//    if(L==0)
//    speed_L= DMA_count_get(LEFT_ENCODER_DMA_CH);
//    else
//    ; 
//    if(R==0)
//    ;
//    else
//    speed_R= DMA_count_get(RIGHT_ENCODER_DMA_CH);
//    if(speed_L >= 30000)
//    {
//      while(1)
//      {
//      LED_PrintValueI5(0,0,(int)speed_L);
//      LED_PrintValueI5(0,1,(int)speed_R);
//      }
//    }
//  }
//}



