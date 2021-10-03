#ifndef __contral_H
#define __contral_H
#include "sys.h"

typedef struct 
{
    float LastP;//�ϴι���Э���� ��ʼ��ֵΪ0.02
    float Now_P;//��ǰ����Э���� ��ʼ��ֵΪ0
    float out;//�������˲������ ��ʼ��ֵΪ0
    float Kg;//���������� ��ʼ��ֵΪ0
    float Q;//��������Э���� ��ʼ��ֵΪ0.001
    float R;//�۲�����Э���� ��ʼ��ֵΪ0.543
}KFP;//Kalman Filter parameter
void Keman_Init();
float kalmanFilter(KFP *kfp,float input);
void TIM5_Init_Init(u16 arr,u16 psc);

extern KFP kfp;
#endif


