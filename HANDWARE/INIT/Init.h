#ifndef __Init_H
#define __Init_H
#include "sys.h"

void TIM1_PWM_Init(u16 per,u16 psc);
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
void Servo_Init_TIM4(uint32_t psc, uint32_t arr);
void motor(int motorID, int speed);
void GoodMoto(int lspeed,int rspeed);
int abs_my(int a,int b);
float Read_EncoderB(void);
float Read_EncoderA(void);
int velocityPID(int Encoder,int Target);
#endif



