#include  "mm.h"
#include "include.h"
extern reg_s ov7725_eagle_reg[49]; 
extern float var[6];
void display_shiliu(int num,int8 pos);
void display()
{
  int image_threshold;
  int8 i;
    if(1 == TFTShow_img_flag)
    { 
      ScreenShow();
    }
    
    if(1 == OLED_dis_flag)
    {
      LED_P6x8Str(0,0,"error:");
      LED_PrintsignValueF4(60,0,camer.error);
      
      LED_P6x8Str(0,1,"CNST:");
      image_threshold = (int)ov7725_eagle_reg[45].val;
      i = image_threshold / 16;
      display_shiliu(i,60);
      i = image_threshold - i * 16;
      display_shiliu(i,66);
      if(gpio_get(PTC6)==0)
      {
      	DELAY_MS(3);//消抖
	if(gpio_get(PTC6)==0)
	{
	  /////////////////////增加阈值//////////////////////
	  ov7725_eagle_reg[45].val += 1;
	  SCCB_WriteByte(ov7725_eagle_reg[45].addr, ov7725_eagle_reg[45].val);
	  while(!gpio_get(PTC6));
	}
      }
      else if(gpio_get(PTC7)==0)
      {
      	DELAY_MS(3);//消抖
	if(gpio_get(PTC7)==0)
	{
	  /////////////////////减少阈值//////////////////////
	  ov7725_eagle_reg[45].val -= 1;
	  SCCB_WriteByte(ov7725_eagle_reg[45].addr, ov7725_eagle_reg[45].val);
	  while(!gpio_get(PTC7));
	}
      }
      
      LED_P6x8Str(0,2,"Cross:");
      switch(cross_flag)
      {
      case NUCROSS:
	LED_P6x8Str(60,2,"N");break;
      case CRO_IN:
	LED_P6x8Str(60,2,"I");break;
      case CRO_BEFORE:
	LED_P6x8Str(60,2,"B");
      }      
      
      LED_P6x8Str(0,3,"Circle:");
      switch(circle_flag)
      {
      case LEFT:
	LED_P6x8Str(60,3,"Lf");break;
      case LCIRCLE:
	LED_P6x8Str(60,3,"Lc");break;
      case LCIRCLE_IN:
	LED_P6x8Str(60,3,"Li");break;
      case LCIRCLE_OUT:
	LED_P6x8Str(60,3,"Lb");break;
      case RIGHT:
	LED_P6x8Str(60,3,"Rf");break;
      case RCIRCLE:
	LED_P6x8Str(60,3,"Rc");break;
      case RCIRCLE_IN:
	LED_P6x8Str(60,3,"Ri");break;
      case RCIRCLE_OUT:
	LED_P6x8Str(60,3,"Rb");break;
      default:
	LED_P6x8Str(60,3,"NU");
      }
    }
      /*
      if(HX_STATE == HX_BEFORE)
      {
        LED_P6x8Str(0,0,"camer.error =");
        LED_PrintsignValueF4(80,0,camer.error); 
        LED_P6x8Str(0,1,"Prospect_See=");
        LED_PrintValueI4(80,1,(int)Prospect_See);
        LED_P6x8Str(0,2,"speed_set   =");
        LED_PrintsignValueI4(80,2,(int)motor.speed_set);  
        LED_P6x8Str(0,3,"l_turn_point=");
        LED_PrintsignValueI4(80,3,(int)l_cross_turn_point); 
        LED_P6x8Str(0,4,"r_turn_point=");
        LED_PrintsignValueI4(80,4,(int)r_cross_turn_point);
        LED_P6x8Str(0,5,"speed_L     =");
        LED_PrintsignValueI4(80,5,(int)motor.speed_L); 
        LED_P6x8Str(0,6,"speed_R     =");
        LED_PrintsignValueI4(80,6,(int)motor.speed_R);
        LED_P6x8Str(0,7,"HX_BEFORE");
//        LED_P6x8Str(0,7,"servo       =");
//        LED_PrintsignValueI4(80,7,(int)(servos.direction_duty_output-Servo_mid));
      }
      if(HX_STATE == HX_ON )
      {
        LED_P6x8Str(0,0,"camer.error =");
        LED_PrintsignValueF4(80,0,camer.error); 
        LED_P6x8Str(0,1,"Prospect_See=");
        LED_PrintValueI4(80,1,(int)Prospect_See);
        LED_P6x8Str(0,2,"speed_set   =");
        LED_PrintsignValueI4(80,2,(int)motor.speed_set);  
        LED_P6x8Str(0,3,"l_turn_point=");
        LED_PrintsignValueI4(80,3,(int)l_cross_turn_point); 
        LED_P6x8Str(0,4,"r_turn_point=");
        LED_PrintsignValueI4(80,4,(int)r_cross_turn_point);
        LED_P6x8Str(0,5,"speed_set_L =");
        LED_PrintsignValueI4(80,5,(int)motor.speed_set_L); 
        LED_P6x8Str(0,6,"speed_set_R =");
        LED_PrintsignValueI4(80,6,(int)motor.speed_set_R);
        LED_P6x8Str(0,7,"HX_ON    ");
//        LED_P6x8Str(0,7,"servo       =");
//        LED_PrintsignValueI4(80,7,(int)(servos.direction_duty_output-Servo_mid));
      }
      if(HX_STATE == HX_ONON )
      {
        LED_PrintsignValueI4(80,6,(int)motor.speed_set_R);
        LED_P6x8Str(0,7,"HX_ONON    ");
      }
      if(HX_STATE == HX_OUT)
      {
        LED_P6x8Str(0,0,"camer.error =");
        LED_PrintsignValueF4(80,0,camer.error); 
        LED_P6x8Str(0,1,"Prospect_See=");
        LED_PrintValueI4(80,1,(int)Prospect_See);
        LED_P6x8Str(0,2,"speed_set   =");
        LED_PrintsignValueI4(80,2,(int)motor.speed_set);  
        LED_P6x8Str(0,3,"speedL_R:");
        LED_PrintsignValueI4(54,3,(int)motor.speed_L);
        LED_PrintsignValueI4(90,3,(int)motor.speed_R);
//        LED_P6x8Str(0,3,"l_turn_point=");
//        LED_PrintsignValueI4(80,3,(int)l_cross_turn_point); 
//        LED_P6x8Str(0,4,"r_turn_point=");
//        LED_PrintsignValueI4(80,4,(int)r_cross_turn_point);
        LED_P6x8Str(0,4,"speed_set_L =");
        LED_PrintsignValueI4(80,4,(int)motor.speed_set_L); 
        LED_P6x8Str(0,5,"speed_set_R =");
        LED_PrintsignValueI4(80,5,(int)motor.speed_set_R);
        LED_P6x8Str(0,6,"SERVE_OUT:");
        LED_PrintsignValueI4(80,6,(int)(servos.direction_duty_output-Servo_mid));
        LED_P6x8Str(0,7,"HX_OUT   ");
      }
      if(HX_STATE == HX_NORMAL)
      {
        LED_P6x8Str(0,0,"camer.error =");
        LED_PrintsignValueF4(80,0,camer.error); 
        LED_P6x8Str(0,1,"Prospect_See=");
        LED_PrintValueI4(80,1,(int)Prospect_See);
        LED_P6x8Str(0,2,"speed_set   =");
        LED_PrintsignValueI4(80,2,(int)motor.speed_set);  
        LED_P6x8Str(0,3,"speedL_R:");
        LED_PrintsignValueI4(54,3,(int)motor.speed_L);
        LED_PrintsignValueI4(90,3,(int)motor.speed_R);
//        LED_P6x8Str(0,3,"l_turn_point=");
//        LED_PrintsignValueI4(80,3,(int)l_cross_turn_point); 
//        LED_P6x8Str(0,4,"r_turn_point=");
//        LED_PrintsignValueI4(80,4,(int)r_cross_turn_point);
        LED_P6x8Str(0,4,"speed_set_L =");
        LED_PrintsignValueI4(80,4,(int)motor.speed_set_L); 
        LED_P6x8Str(0,5,"speed_set_R =");
        LED_PrintsignValueI4(80,5,(int)motor.speed_set_R);
        LED_P6x8Str(0,6,"SERVE_OUT:");
        LED_PrintsignValueI4(80,6,(int)(servos.direction_duty_output-Servo_mid));
        LED_P6x8Str(0,7,"HX_NORMAL");
      }
    }*/
//        if(Car_Stop_flag == 1)
//        {
//          gpio_init (PTB2,GPO, 0);
//          gpio_set (PTB2, 0);
//        }
//        else
//        {
//          gpio_init (PTB2,GPO, 0);
//          gpio_set (PTB2, 1);
//        }
}
void display_shiliu(int num,int8 pos)
{
  switch(num)
  {
  case 0:       LED_P6x8Char(pos,1,'0');break;
  case 1:       LED_P6x8Char(pos,1,'1');break;
  case 2:       LED_P6x8Char(pos,1,'2');break;
  case 3:       LED_P6x8Char(pos,1,'3');break;
  case 4:       LED_P6x8Char(pos,1,'4');break;
  case 5:       LED_P6x8Char(pos,1,'5');break;
  case 6:       LED_P6x8Char(pos,1,'6');break;
  case 7:       LED_P6x8Char(pos,1,'7');break;
  case 8:       LED_P6x8Char(pos,1,'8');break;
  case 9:       LED_P6x8Char(pos,1,'9');break;
  case 10:       LED_P6x8Char(pos,1,'A');break;
  case 11:       LED_P6x8Char(pos,1,'B');break;
  case 12:       LED_P6x8Char(pos,1,'C');break;
  case 13:       LED_P6x8Char(pos,1,'D');break;
  case 14:       LED_P6x8Char(pos,1,'E');break;
  case 15:       LED_P6x8Char(pos,1,'F');break;
  }
}
void send()
{
    /*Matlab接收图像*/
//    if(1)
//    {
//        if(img_switch_flag != 0)
//        vcan_sendmatlabimg(img_buffer,CAMERA_SIZE);
//        else
//        vcan_sendmatlabimg(img_buffer2,CAMERA_SIZE);
//    }
//    if(0)//Matlab 示波器发送控制
//    {
//      vag[0] = motor.avg_speed;
//      vcan_sendmatlabware((uint8_t *)vag, sizeof(vag));
//    }
    /*示波器*/
    if(img_send_flag)//1 == send_osc_flag)
    {
       var[0] = camer.error;
       var[1] = servos.direction_p*100;
       var[2] = motor.speed_set_R;
       var[4] = motor.speed_set_L;//camer.error-camer.last_error;
       var[3] = motor.speed_L;//HX_STATE;
       var[5] = motor.speed_R;//motor.speed_set_R;
       
       vcan_sendware((uint8_t *)var, sizeof(var));
    }
}
