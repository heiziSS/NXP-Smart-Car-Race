/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       VCAN_camera.h
 * @brief      ����ͷ�����ӿ��ض���
 * @author     ɽ��Ƽ�
 * @version    v5.2.1
 * @date       2015-04-01
 */


#ifndef _VCAN_CAMERA_H_
#define _VCAN_CAMERA_H_


#define CAMERA_OV7725_EAGLE         2       //ɽ��ӥ��
#define CAMERA_OV7725_WOLF          3       //ɽ������


#define USE_CAMERA      CAMERA_OV7725_EAGLE   //ѡ��ʹ�õ� ����ͷ

typedef struct
{
    uint8 addr;                 /*�Ĵ�����ַ*/
    uint8 val;                   /*�Ĵ���ֵ*/
} reg_s;

//����ͼ��ɼ�״̬
typedef enum
{
    IMG_NOTINIT = 0,
    IMG_FINISH,             //ͼ��ɼ����
    IMG_FAIL,               //ͼ��ɼ�ʧ��(�ɼ���������)
    IMG_GATHER,             //ͼ��ɼ���
    IMG_START,              //��ʼ�ɼ�ͼ��
    IMG_STOP,               //��ֹͼ��ɼ�
} IMG_STATUS_e;

#define EMPTY               160
#define VARIANCE_THRESHOLD  4    //����㷽����ֵ
#define TRAOFF              64//54
#define OBS_hang            40//40  //�ϰ���ǰ���д��
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
     float error;     //ƫ��
     float last_error;//�ϴ�ƫ��
     float curvature; //����
     float speed_control_error;   //�ٶȿ���ƫ��    
}camer_status;

extern camer_status camer;

/*�����ṹ��*/
typedef struct TRACK_INFM
{    
    float First_deriva[118];   //һ�׵�
    float average[116];        //ƽ��ֵ
    int8 jump_point[3];         
    int8 jump_point_state[3];   //0�Ϻ��°� 1�ϰ��º�
    int8 jump_count;
    int8 inf_point;
    int8 inf_point_state;       //0�Ϻ��°� 1�ϰ��º�
    int8 inf_count;
}TRACK_INFM;

/*ö����������*/
typedef enum
{
    TRACK_COMMON = 0,
    TRACK_OBLI_CRO_L, //��бʮ��
    TRACK_OBLI_CRO_R, //��бʮ��
    TRACK_CROSS,
    TRACK_CIRCLE,     //����
    TRACK_OBSTACLE_L, //���ϰ���
    TRACK_OBSTACLE_R, //���ϰ���
    TRACK_SCRATCH,    //������
} track_state_e;

/*ö�ٻ���״̬*/
typedef enum
{
    NUCIRCLE = 0,   //�޻���
    LEFT,           //����
    LCIRCLE_IN,     //������
    LCIRCLE_OUT,    //������
    LCIRCLE,        //������
    RIGHT,          //�һ���
    RCIRCLE_IN,     //���һ���
    RCIRCLE_OUT,    //���һ���
    RCIRCLE,        //�һ�����
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
uint8 AcquireRealBorder(uint8* fork_head,uint8* fork_tail,uint8* maxw_row,int8 fork_flag,int track_end);//�õ������߽�
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

// ����ͷ �ӿ�ͳһ�ĳ� ����ģʽ

//  camera_init(imgaddr);
//  camera_get_img();
//  camera_cfg(rag,val)


//  camera_vsync()  //���ж�
//  camera_href()   //���ж�
//  camera_dma()    //DMA�ж�

// ��Ҫ �ṩ ���� �궨��
// #define  CAMERA_USE_HREF    1     //�Ƿ�ʹ�� ���ж� (0 Ϊ ��ʹ�ã�1Ϊʹ��)
// #define  CAMERA_COLOR       1     //����ͷ�����ɫ �� 0 Ϊ �ڰ׶�ֵ��ͼ�� ��1 Ϊ �Ҷ� ͼ�� ��2 Ϊ RGB565 ͼ��
// #define  CAMERA_POWER       0     //����ͷ ��Դѡ�� 0 Ϊ 3.3V ,1 Ϊ 5V



#endif


