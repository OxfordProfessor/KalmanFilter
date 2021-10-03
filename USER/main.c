#include "stm32f10x.h"
#include "Init.h"
#include "contral.h"
#include "delay.h"
#include "usart.h"
/************************************************
δ�������
   1. ���Ķ�ʱ������
   2. ������Ƶֵ
   3. �ж����ȼ��趨
   4. ���Ա���������ֵ��С���ٶȵĹ�ϵ
************************************************/



 int main(void)
 {	
	 extern KFP kfp;
	 KFP KFP_height={200,0,0,100,10,543};
	 float a=54.14;
	 u16 t;  
	 u16 len;	
	 u16 times=0;
	 float b;
//     TIM1_PWM_Init(7199,0);
	 Encoder_Init_TIM2();
//	 Encoder_Init_TIM3();
//	 Servo_Init_TIM4(720, 2000);
	 TIM5_Init_Init(99,7199);
	 delay_init();	    	 //��ʱ������ʼ��	  
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	 uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
	 while(1){
        if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			printf("%f",Read_EncoderA());
			for(t=0;t<len;t++)
			{
				USART_SendData(USART1, USART_RX_BUF[t]);//�򴮿�1��������
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			}
			printf("\r\n\r\n");//���뻻��
			USART_RX_STA=0;
		}else
		{
			times++;
			if(times%5000==0)
			{
				printf("\r\nս��STM32������ ����ʵ��\r\n");
				printf("����ԭ��@ALIENTEK\r\n\r\n");
			}
			if(times%10==0){
				printf("%lf",Read_EncoderA()); 
				printf("\t");
				b=kalmanFilter(&KFP_height,Read_EncoderA());
				printf("%lf",b);
                printf("\r\n\r\n");
			}				
			delay_ms(10);   
		}
	 }
 }

