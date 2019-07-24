/********************************�������**********************************
*
*����   �� ɳ���Ѿ�
*�ļ��� �� FuzzySet_Speed.h
*����   �� ģ���㷨�趨�ٶ�ͷ�ļ�
*ʱ��   �� 2016/4/14
˵��    ���ο�FreeCarsģ����������  ʹ��ɽ��V5.3��
*
****************************************************************************/
#ifndef _FUZZYSET_SPEED_H_
#define _FUZZYSET_SPEED_H_

#include "include.h" 

int16 FuzzySet_Speed(int16 P, int16 D,int16 E);  /*ģ����������*/
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