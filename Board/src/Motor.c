/********************************S-D舵机驱动**********************************
*
*作者   ： 沙艺已觉
*文件名 ： Motor.c
*描述   ： 舵机驱动函数
*时间   ： 2015/11/2
*
****************************************************************************/

#include  "Motor.h"
//uint8 s=0;
motor_status motor;
//static uint8 speed_flag=0;
extern float Speed_KP;//12.8//7.4
extern float Speed_KI;//0.38
extern float HighSpeed_KP;//20.8//8.4
extern float HighSpeed_KI;//0.64
void myMotor_Setspeed(int16 PWM_L,int16 PWM_R);
/*************************************************************************
沙艺已觉
函数名称：motor_init()
入口参数：无
出口参数：无
修改时间：2015/11/2
功能说明：电机驱动初始化
其他说明：参考let_it_go源代码
************************************************************************/

//void Motor_init(void)
//{
//  ftm_pwm_init(L_GO_FTM,L_GO_CH,MOTOR_FRE,0);    
//  ftm_pwm_init(L_BACK_FTM,L_BACK_CH,MOTOR_FRE,0);     
//}

/*************************************************************************
沙艺已觉
函数名称： void speed_control(void)
入口参数：无
出口参数：无
修改时间：2015/11/2
功能说明：
其他说明：参考let_it_go源代码 使用山外V5.3库
************************************************************************/
void speed_control(void)
{
//  float chasu;
    gearshift_integral((motor.speed_current_error_L + motor.speed_current_error_R)/2,(motor.speed_current_error_L-motor.speed_last_error_L)/2 + (motor.speed_current_error_R-motor.speed_last_error_R)/2
                       ,HighSpeed_KP,Speed_KP,HighSpeed_KI,Speed_KI);
    filter_Prospect_See();
    motor.avg_speed= (motor.speed_L+ motor.speed_R)/2; //平均速度
    if(Car_Stop_flag==1)//正常速度控制
    {
      motor.speed_set =0;//PID停车
    }
    else 
    {
      if(circle_flag != NUCIRCLE)//环形换慢速表
      {
        UFF=UFF_HX;
      }
      else
      {
        UFF=UFF_NORMAL;
      }
      motor.speed_set = FuzzySet_Speed(Track_complexity, Prospect_See,0);//MAX_DISTANCE-Distance
      if(Obstacle_flag)		motor.speed_set = MIN(motor.speed_set,260);
    }
    motor.speed_filter_error[2] = motor.speed_filter_error[1];
    motor.speed_filter_error[1] = motor.speed_filter_error[0];
    motor.speed_filter_error[0] = motor.speed_set;
    motor.speed_set = (int16)(0.7*motor.speed_filter_error[0]+0.2*motor.speed_filter_error[1]+0.1*motor.speed_filter_error[2]);
//速度滤波左 获得速度中值
    motor.speed_set_L=motor.speed_set;
//速度滤波右 获得速度中值
    motor.speed_set_R=motor.speed_set;

//    chasu = FuzzySet_Casu(ABS((int16)camer.error), Prospect_See)/1000.0;
    if(camer.error<-10 && !Obstacle_flag)
    {
//      var[0] = -chasu;
      chasu = FuzzySet_Casu(ABS((int16)camer.error), Prospect_See)/1000.0;
//      var[0] = chasu * -1000;
      motor.speed_set_R=(int16)(motor.speed_set_R*(1-0.0*chasu));
      motor.speed_set_L=(int16)(motor.speed_set_L*(1-1.0*chasu));
    }
    else if(camer.error>10 && !Obstacle_flag)
    {
//      var[0] = chasu;
      chasu = FuzzySet_Casu(ABS((int16)camer.error), Prospect_See)/1000.0;
//      var[0] = chasu * 1000;
      motor.speed_set_R=(int16)(motor.speed_set_R*(1-1.0*chasu));
      motor.speed_set_L=(int16)(motor.speed_set_L*(1-0.0*chasu));
    }
      
//左
        motor.speed_last_error_L = motor.speed_current_error_L;        //更新每次的差值
        motor.speed_current_error_L = motor.speed_set_L-motor.speed_L;   //速度当前差值
        motor.speed_duty_output_L = motor.speed_duty_output_L+(int16)(motor.speed_p*(motor.speed_current_error_L-motor.speed_last_error_L)+motor.speed_i*motor.speed_current_error_L);
//右       
        motor.speed_last_error_R = motor.speed_current_error_R;        //更新每次的差值
        motor.speed_current_error_R = motor.speed_set_R-motor.speed_R;   //速度当前差值
        motor.speed_duty_output_R = motor.speed_duty_output_R+(int16)(motor.speed_p*(motor.speed_current_error_R-motor.speed_last_error_R)+motor.speed_i*motor.speed_current_error_R);

 //限幅  
    if(motor.speed_duty_output_L>L_Speed_MAX) motor.speed_duty_output_L=L_Speed_MAX;
    if(motor.speed_duty_output_L<L_Speed_MIN) motor.speed_duty_output_L=L_Speed_MIN;
    if(motor.speed_duty_output_R>R_Speed_MAX) motor.speed_duty_output_R=R_Speed_MAX;
    if(motor.speed_duty_output_R<R_Speed_MIN) motor.speed_duty_output_R=R_Speed_MIN;
    
    
//    if(BM8==0)
//    {
//        ftm_pwm_duty(L_BACK_FTM,L_BACK_CH,0);
//        ftm_pwm_duty(L_GO_FTM,L_GO_CH,300);
//        ftm_pwm_duty(R_GO_FTM,R_GO_CH,300);
//        ftm_pwm_duty(R_BACK_FTM,R_BACK_CH,0);
//      if(motor.speed_L<50 || motor.speed_R<50)
//      {
//          ftm_pwm_duty(L_BACK_FTM,L_BACK_CH,0);
//          ftm_pwm_duty(L_GO_FTM,L_GO_CH,0);
//          ftm_pwm_duty(R_BACK_FTM,R_BACK_CH,0);
//          ftm_pwm_duty(R_GO_FTM,R_GO_CH,0);
//      }
//    }
    if(Car_Run_flag == 0 && motor.speed_L>80 && motor.speed_R>80)
    {
      Car_Run_flag = 1;
    }
    if(Car_Run_flag && (motor.speed_L<80 || motor.speed_R<80))
    {
          ftm_pwm_duty(L_BACK_FTM,L_BACK_CH,0);
          ftm_pwm_duty(L_GO_FTM,L_GO_CH,0);
          ftm_pwm_duty(R_BACK_FTM,R_BACK_CH,0);
          ftm_pwm_duty(R_GO_FTM,R_GO_CH,0);
    }
    else 
    {
      myMotor_Setspeed(motor.speed_duty_output_R,motor.speed_duty_output_L);
    }

    
}//速度控制结束

/*************************************************************************
*  函数名称：myMotor_Setspeed
*  功能说明：电机输出、带限幅
*  参数说明：
*  函数返回：无
*  修改时间：2016-4-21    
*  备    注：后轮左右轮的距离为13.7cm，前后轮的距离为19.8cm
*************************************************************************/
void myMotor_Setspeed(int16 PWM_R,int16 PWM_L)//给速度
{
  //左
  if(PWM_L>=0)
    {
      ftm_pwm_duty(L_BACK_FTM,L_BACK_CH,0);
      ftm_pwm_duty(L_GO_FTM,L_GO_CH,(uint32)(PWM_L));
    }
  else
    {
      ftm_pwm_duty(L_GO_FTM,L_GO_CH,0);
      ftm_pwm_duty(L_BACK_FTM,L_BACK_CH,(uint32)(-PWM_L));
    } 
  //右
  if(PWM_R>=0)
    {
      ftm_pwm_duty(R_BACK_FTM,R_BACK_CH,0);
      ftm_pwm_duty(R_GO_FTM,R_GO_CH,(uint32)(PWM_R));
    }
  else
    {
      ftm_pwm_duty(R_GO_FTM,R_GO_CH,0);
      ftm_pwm_duty(R_BACK_FTM,R_BACK_CH,(uint32)(-PWM_R));
    }
  
  
//  if(motor.speed_L<50)
//  {
//      ftm_pwm_duty(L_BACK_FTM,L_BACK_CH,0);
//      ftm_pwm_duty(L_GO_FTM,L_GO_CH,0);
//  }
//  if(motor.speed_R<50)
//  {    
//      ftm_pwm_duty(R_BACK_FTM,R_BACK_CH,0);
//      ftm_pwm_duty(R_GO_FTM,R_GO_CH,0);
//  }
}

/*************************************************************************
*  函数名称：Motor_PID_control
*  功能说明：电机PID算法控制
*  参数说明：uint8 load_type  道路类型
*  函数返回：无
*  修改时间：2016-4-21    
*  备    注：后轮左右轮的距离为13.7cm，前后轮的距离为19.8cm
*************************************************************************/
void Motor_init(void)
{
    ftm_pwm_init(L_GO_FTM,L_GO_CH,MOTOR_FRE,INIT_DUTY);
    ftm_pwm_init(L_BACK_FTM,L_BACK_CH,MOTOR_FRE,INIT_DUTY);
    ftm_pwm_init(R_GO_FTM,R_GO_CH,MOTOR_FRE,INIT_DUTY);
    ftm_pwm_init(R_BACK_FTM,R_BACK_CH,MOTOR_FRE,INIT_DUTY);
    
//    ftm_quad_init(FTM1);//正交解码      
//    ftm_quad_init(FTM2); 
    DMA_count_Init(LEFT_ENCODER_DMA_CH,LEFT_ENCODER_COUNT, 0x7FFF,DMA_rising);//左编码器
    DMA_count_Init(RIGHT_ENCODER_DMA_CH,RIGHT_ENCODER_COUNT, 0x7FFF,DMA_rising);//右编码器
    gpio_init(LEFT_ENCODER_DIRECTION,GPI,1);
    gpio_init(RIGHT_ENCODER_DIRECTION,GPI,1);
}

void DMA_Speed_get()
{
    uint8 L,R;
    L=gpio_get(LEFT_ENCODER_DIRECTION);
    R=gpio_get(RIGHT_ENCODER_DIRECTION);
    if(L==0)
    motor.speed_L=-DMA_count_get(LEFT_ENCODER_DMA_CH);
    else
    motor.speed_L= DMA_count_get(LEFT_ENCODER_DMA_CH); 
    if(R==0)
    motor.speed_R= DMA_count_get(RIGHT_ENCODER_DMA_CH);
    else                                                                                                                                                                                   
    motor.speed_R=-DMA_count_get(RIGHT_ENCODER_DMA_CH); 
    
    DMA_count_reset(LEFT_ENCODER_DMA_CH);
    DMA_count_reset(LEFT_ENCODER_DMA_CH);
    DMA_count_reset(RIGHT_ENCODER_DMA_CH);
    DMA_count_reset(RIGHT_ENCODER_DMA_CH);
    filter_speed_L();
    filter_speed_R();
}

//获取左右电机速度，滤波
#define LAG 70.0 //滞后系数 越小越滞后
void filter_speed_L()
{
    static float a = 0;
    float speed_error ;
 //   motor.speed_L=FTM2_Speed_get();
    speed_error = motor.speed_L - motor.speed_L_save[0];
    a = speed_error/LAG ;                //a是系数
    if(a < 0)
      a = -a ;
    if(a > 0.85)
      a = 0.85 ;
  //速度滤波，一阶滞后滤波算法
  motor.speed_L = (int16)(motor.speed_L * (1-a) + (motor.speed_L_save[0]+motor.speed_L_save[1]+motor.speed_L_save[2])*a/3);//(motor.speed_L * (1-a) + (motor.speed_L_save[0]+motor.speed_L_save[1]+motor.speed_L_save[2]+motor.speed_L_save[3]+motor.speed_L_save[4]+motor.speed_L_save[5]+motor.speed_L_save[6]+motor.speed_L_save[7]+motor.speed_L_save[8]+motor.speed_L_save[9])*a/10);// car.speed_new * 0.1 +  
  
//  motor.speed_L_save[9] = motor.speed_L_save[8] ;
//  motor.speed_L_save[8] = motor.speed_L_save[7] ;
//  motor.speed_L_save[7] = motor.speed_L_save[6] ;
//  motor.speed_L_save[6] = motor.speed_L_save[5] ;
//  motor.speed_L_save[5] = motor.speed_L_save[4] ;
//  motor.speed_L_save[4] = motor.speed_L_save[3] ;
//  motor.speed_L_save[3] = motor.speed_L_save[2] ;
  motor.speed_L_save[2] = motor.speed_L_save[1] ;
  motor.speed_L_save[1] = motor.speed_L_save[0] ;
  motor.speed_L_save[0] = motor.speed_L ;
//  car.speed_error = car.speed_new - car.speed_old;
//  car.speed_old = car.speed_new ; 
}

void filter_speed_R()
{
    static float a = 0;
    float speed_error ;
 //   motor.speed_L=FTM2_Speed_get();
    speed_error = motor.speed_R - motor.speed_R_save[0];
    a = speed_error/LAG ;                //a是系数
    if(a < 0)
      a = -a ;
    if(a > 0.85)
      a = 0.85 ;
  //速度滤波，一阶滞后滤波算法
  motor.speed_R = (int16)(motor.speed_R * (1-a) + (motor.speed_R_save[0]+motor.speed_R_save[1]+motor.speed_R_save[2])*a/3);//(motor.speed_R * (1-a) + (motor.speed_R_save[0]+motor.speed_R_save[1]+motor.speed_R_save[2]+motor.speed_R_save[3]+motor.speed_R_save[4]+motor.speed_R_save[5]+motor.speed_R_save[6]+motor.speed_R_save[7]+motor.speed_R_save[8]+motor.speed_R_save[9])*a/10);// car.speed_new * 0.1 +  
  
//  motor.speed_R_save[9] = motor.speed_R_save[8] ;
//  motor.speed_R_save[8] = motor.speed_R_save[7] ;
//  motor.speed_R_save[7] = motor.speed_R_save[6] ;
//  motor.speed_R_save[6] = motor.speed_R_save[5] ;
//  motor.speed_R_save[5] = motor.speed_R_save[4] ;
//  motor.speed_R_save[4] = motor.speed_R_save[3] ;
//  motor.speed_R_save[3] = motor.speed_R_save[2] ;
  motor.speed_R_save[2] = motor.speed_R_save[1] ;
  motor.speed_R_save[1] = motor.speed_R_save[0] ;
  motor.speed_R_save[0] = motor.speed_R ;
//  car.speed_error = car.speed_new - car.speed_old;
//  car.speed_old = car.speed_new ; 
}
void filter_Prospect_See()
{
    static float a = 0;
    float Prospect_error;
    Prospect_error = Prospect_See - Last_Prospect_See;
    a = Prospect_error/20 ;                //分母越小越滞后
    if(a < 0)
      a = -a;
    if(a > 0.85)
      a = 0.85 ;
    
    if(a>0.25)
    Prospect_See = (int16)((1-a)*Prospect_See+a*Last_Prospect_See);
}