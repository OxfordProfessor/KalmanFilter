#include "stm32f10x.h"
#include "Init.h"
#include "contral.h"
#include "delay.h"
#include "usart.h"
/************************************************
未完成任务：
   1. 更改定时器串口
   2. 调整分频值
   3. 中断优先级设定
   4. 测试编码器读数值与小车速度的关系
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
	 delay_init();	    	 //延时函数初始化	  
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	 uart_init(115200);	 //串口初始化为115200
	 while(1){
        if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			printf("%f",Read_EncoderA());
			for(t=0;t<len;t++)
			{
				USART_SendData(USART1, USART_RX_BUF[t]);//向串口1发送数据
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
			}
			printf("\r\n\r\n");//插入换行
			USART_RX_STA=0;
		}else
		{
			times++;
			if(times%5000==0)
			{
				printf("\r\n战舰STM32开发板 串口实验\r\n");
				printf("正点原子@ALIENTEK\r\n\r\n");
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

