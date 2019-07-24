/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       VCAN_camera.h
 * @brief      摄像头函数接口重定向
 * @author     山外科技
 * @version    v5.2.1
 * @date       2015-04-01
 */


#ifndef _VCAN_CAMERA_H_
#define _VCAN_CAMERA_H_


#define CAMERA_OV7725_EAGLE         2       //山外鹰眼
#define CAMERA_OV7725_WOLF          3       //山外狼眼


#define USE_CAMERA      CAMERA_OV7725_EAGLE   //选择使用的 摄像头

typedef struct
{
    uint8 addr;                 /*寄存器地址*/
    uint8 val;                   /*寄存器值*/
} reg_s;

//定义图像采集状态
typedef enum
{
    IMG_NOTINIT = 0,
    IMG_FINISH,             //图像采集完毕
    IMG_FAIL,               //图像采集失败(采集行数少了)
    IMG_GATHER,             //图像采集中
    IMG_START,              //开始采集图像
    IMG_STOP,               //禁止图像采集
} IMG_STATUS_e;

#define EMPTY               160
#define VARIANCE_THRESHOLD  4    //跳变点方差阈值
#define TRAOFF              64//54
#define OBS_hang            40//40  //障碍提前几行打角
//#define OBSTACLE_OFFSET     20

#define UKBLACK_HEIGHT      4
#define UKBLACK_WIDTH       10
#define LINIT               0
#define RINIT               159
#define MINIT               80    

#include  "VCAN_SCCB.h"
#include  "VCAN_OV7725_Eagle.h"

typedef struct camera_status
{
     float error;     //偏差
     float last_error;//上次偏差
     float curvature; //曲率
     float speed_control_error;   //速度控制偏差    
}camer_status;

extern camer_status camer;

/*跳变点结构体*/
typedef struct TRACK_INFM
{    
    float First_deriva[118];   //一阶导
    float average[116];        //平均值
    int8 jump_point[3];         
    int8 jump_point_state[3];   //0上黑下白 1上白下黑
    int8 jump_count;
    int8 inf_point;
    int8 inf_point_state;       //0上黑下白 1上白下黑
    int8 inf_count;
}TRACK_INFM;

/*枚举赛道类型*/
typedef enum
{
    TRACK_COMMON = 0,
    TRACK_OBLI_CRO_L, //左斜十字
    TRACK_OBLI_CRO_R, //右斜十字
    TRACK_CROSS,
    TRACK_CIRCLE,     //环形
    TRACK_OBSTACLE_L, //左障碍物
    TRACK_OBSTACLE_R, //右障碍物
    TRACK_SCRATCH,    //起跑线
} track_state_e;

/*枚举环形状态*/
typedef enum
{
    NUCIRCLE = 0,   //无环形
    LEFT,           //左环形
    LCIRCLE_IN,     //进左环形
    LCIRCLE_OUT,    //出左环形
    LCIRCLE,        //左环形中
    RIGHT,          //右环形
    RCIRCLE_IN,     //进右环形
    RCIRCLE_OUT,    //出右环形
    RCIRCLE,        //右环形中
}CIRCLE_status; 

typedef enum
{
    NUCROSS = 0,
    CRO_BEFORE,
    CRO_IN,
}CROSS_status;

void camer_error_calculation();
void Get_Edge();
void img_extract(void *dst1, void *dst2, void *src, uint32_t srclen);
void CopyToSrcimg(void *dst, void *src, uint32_t srclen);extern void Get_Edge();
void TrackFilter(int16 start, int16 end, int16 * line);
uint8 ConvertToBorder(uint8 pixnum,uint8 pixel,char flag,uint8 last_border,uint8 last_pixnum);
int   Edge_Tracking();
int8 Track_Scanning(uint8* fork_head,uint8* fork_tail,uint8* maxw_row,int track_end);
uint8 AcquireRealBorder(uint8* fork_head,uint8* fork_tail,uint8* maxw_row,int8 fork_flag,int track_end);//得到赛道边界
void CalFirstDeriva(TRACK_INFM* track, int16* line,int end);
void AcquireInflecPoint(TRACK_INFM* track, int end, int16* line, char flag);
void AcquireJumpPoint(TRACK_INFM* track, int end, char flag);
void matchline(int16 start, int16 startp, int16 end, int16 endp, int16* line);
int8 ExtendTheTrack(uint8 start, uint8 end, int16* line);
float calculate_variance(float a, float b, float c, float average);
uint8 MaxMin(uint8 start,uint8 end,int16* line,char choose,char type);
track_state_e CircleJudge(int track_end);
track_state_e ObliqueCrossJudge(int8 infpoint,int8 maxw_row,uint8 fork_tail,uint8 fork_head,char flag);
track_state_e ScratchObstacleJudge(int8 maxw_row,uint8 fork_head,uint8 fork_tail);
uint8 LostLineCount(int16 start, int16 end, int16* line, char flag);
void aquire_Mline(int16* Lline,int16* Rline,int16* Mline);
void fork_select(int16* line,int8 start,int8 fork_head,int8 end,char flag);
void data_init();
track_state_e ComCross_handle(TRACK_INFM* right,TRACK_INFM* left,int track_end);
float error_filter(float a,float b,float c);
float MMN_deriva(uint8 start,uint8 end,float* line,char type);
int16 MMN(int16 *line,uint8 s,uint8 e,uint8 choose);
track_state_e Obstacle_handle(uint8 fork_flag,uint8 fork_head,uint8 fork_tail,int track_end);
void Circle_handle(int track_end,uint8 fork_flag,uint8 fork_head,uint8 fork_tail);
void Circle_flag_handle(int8 fork_flag,uint8 fork_head,uint8 fork_tail);
uint8 sratch_handle();
uint8 Ramp_handle(TRACK_INFM* right,TRACK_INFM* left,int8 H,int8 L,float ramp_V,uint8 flag);
float f_MMN(float *line,uint8 s,uint8 e,uint8 choose);
float camer_error_calculation_jihang(int8 x,int8 y);
void cross_flag_handle();
void cross_handle(TRACK_INFM* right,TRACK_INFM* left,int track_end,uint8 fork_flag,uint8 fork_head,uint8 fork_tail);

extern void img_extract(void *dst1, void *dst2, void *src, uint32_t srclen);
extern void CopyToSrcimg(void *dst, void *src, uint32_t srclen);
extern int border[120][4];

// 摄像头 接口统一改成 如下模式

//  camera_init(imgaddr);
//  camera_get_img();
//  camera_cfg(rag,val)


//  camera_vsync()  //场中断
//  camera_href()   //行中断
//  camera_dma()    //DMA中断

// 需要 提供 如下 宏定义
// #define  CAMERA_USE_HREF    1     //是否使用 行中断 (0 为 不使用，1为使用)
// #define  CAMERA_COLOR       1     //摄像头输出颜色 ， 0 为 黑白二值化图像 ，1 为 灰度 图像 ，2 为 RGB565 图像
// #define  CAMERA_POWER       0     //摄像头 电源选择， 0 为 3.3V ,1 为 5V



#endif


