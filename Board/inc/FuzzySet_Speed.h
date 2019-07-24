/********************************电机驱动**********************************
*
*作者   ： 沙艺已觉
*文件名 ： FuzzySet_Speed.h
*描述   ： 模糊算法设定速度头文件
*时间   ： 2016/4/14
说明    ：参考FreeCars模糊控制例程  使用山外V5.3库
*
****************************************************************************/
#ifndef _FUZZYSET_SPEED_H_
#define _FUZZYSET_SPEED_H_

#include "include.h" 

int16 FuzzySet_Speed(int16 P, int16 D,int16 E);  /*模糊运算引擎*/
void gearshift_integral(int16 SP,int16 SD,float PS,float PX,float IS,float IX);

extern int16 *UFF ;
extern int16 *UFF_HX;
extern int16 *UFF_NORMAL;

extern int16 UFF0_NORMAL[7] ;
extern int16 UFF1_NORMAL[7] ;
extern int16 UFF2_NORMAL[7] ;
extern int16 UFF3_NORMAL[7] ;
extern int16 UFF4_NORMAL[7] ;
extern int16 UFF5_NORMAL[7] ;
extern int16 UFF6_NORMAL[7] ;

extern int16 UFF0_HX[7] ;
extern int16 UFF1_HX[7] ;
extern int16 UFF2_HX[7] ;
extern int16 UFF3_HX[7] ;
extern int16 UFF4_HX[7] ;
extern int16 UFF5_HX[7] ;
extern int16 UFF6_HX[7] ;
#endif