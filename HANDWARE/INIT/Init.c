#include "Init.h"
#include "contral.h"
#define ENCODER_TIM_PERIOD (u16)(65535)   //���ɴ���65535 ��ΪF103�Ķ�ʱ����16λ�ġ�
#define maxspeed 600
#define limit 800
/*
  ��������Ҫռ������ͨ������Ҫ������ʱ�������м������ܵĶ�ʱ��1��2��3��4��5��8����1��2Ϊ
  �߼���ʱ����6��7Ϊ������ʱ����ֻ�����ϼ������ܣ�����֮���ã���
  ������ѡ��ʱ��2��3Ϊ������������������ɼ���ʱ����ѡ��ʱ��1��8Ϊ���������PWM��ʱ����
  ǰ��ֻ������ת�򣬶��PWMʹ�ö�ʱ��4���жϿ��ƺ����ö�ʱ��5��
  
  ����ʹ�ø߼���ʱ�����Ƶ��
*/


void TIM1_PWM_Init(u16 per,u16 psc)        //���PWM��ʼ��
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  //need to reset the pin of the righr motor
	                                                                     	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;      //need to reset the pin of the righr motor,this is the four channels of TIM1.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = per; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

	//�˴��ṹ�����ȫ�����ã������������PWM����������
	TIM_OCInitStructure.TIM_Pulse = 0; 		//��װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //TIM1�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//ʹ������Ƚ�״̬
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//ʹ������Ƚ�N״̬
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM1 ����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//TIM1 ����Ƚ�N���Ը�
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//�� MOE=0 ���� TIM1 ����ȽϿ���״̬
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//�� MOE=0 ���� TIM1 ����Ƚ� N ����״̬
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); //����Ƚ�ͨ��1��ʼ��  
    
	//�˴��ṹ�����ȫ�����ã������������PWM����������
	TIM_OCInitStructure.TIM_Pulse = 0; 		//��װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //TIM1�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//ʹ������Ƚ�״̬
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//ʹ������Ƚ�N״̬
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM1 ����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//TIM1 ����Ƚ�N���Ը�
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//�� MOE=0 ���� TIM1 ����ȽϿ���״̬
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//�� MOE=0 ���� TIM1 ����Ƚ� N ����״̬
	TIM_OC2Init(TIM1, &TIM_OCInitStructure); //����Ƚ�ͨ��2��ʼ�� 
	
	//�˴��ṹ�����ȫ�����ã������������PWM����������
	TIM_OCInitStructure.TIM_Pulse = 0; 		//��װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //TIM1�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//ʹ������Ƚ�״̬
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//ʹ������Ƚ�N״̬
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM1 ����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//TIM1 ����Ƚ�N���Ը�
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//�� MOE=0 ���� TIM1 ����ȽϿ���״̬
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//�� MOE=0 ���� TIM1 ����Ƚ� N ����״̬
	TIM_OC3Init(TIM1, &TIM_OCInitStructure); //����Ƚ�ͨ��3��ʼ��
	
	//�˴��ṹ�����ȫ�����ã������������PWM����������
	TIM_OCInitStructure.TIM_Pulse = 0; 		//��װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //TIM1�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//ʹ������Ƚ�״̬
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//ʹ������Ƚ�N״̬
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM1 ����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//TIM1 ����Ƚ�N���Ը�
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//�� MOE=0 ���� TIM1 ����ȽϿ���״̬
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//�� MOE=0 ���� TIM1 ����Ƚ� N ����״̬
	TIM_OC4Init(TIM1, &TIM_OCInitStructure); //����Ƚ�ͨ��4��ʼ��
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);//ʹ��TIMx�� CCR1 �ϵ�Ԥװ 
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);//ʹ��TIMx�� CCR2 �ϵ�Ԥװ
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);//ʹ��TIMx�� CCR3 �ϵ�Ԥװ
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);//ʹ��TIMx�� CCR4 �ϵ�Ԥװ
	TIM_ARRPreloadConfig(TIM1, ENABLE);//ʹ��Ԥװ�ؼĴ���
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	TIM_Cmd(TIM1, ENABLE);  //ʹ�ܶ�ʱ��	
}


/**************************************************************************
�������ܣ���TIM2��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//ʹ�ܶ�ʱ��2��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//need to reset the pin of the two channels of Enoder of left motor
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	//need to reset the pin of the two channels of Enoder of right motor
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);		//need to reset the pin of the two channels of Enoder of right motor
 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ؼ���ģʽ 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);  //��ʼ����ʱ��2
	
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	
	TIM_ICStructInit(&TIM_ICInitStructure); //��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
	TIM_ICInitStructure.TIM_ICFilter = 10;  //�����˲�������
	TIM_ICInit(TIM2, &TIM_ICInitStructure);//���� TIM_ICInitStruct �Ĳ�����ʼ������	TIMx
 
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//ʹ�ܶ�ʱ���ж�
	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��2
}

/**************************************************************************
�������ܣ���TIM3��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM3(void)
{
  	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  	TIM_ICInitTypeDef TIM_ICInitStructure;  
  	GPIO_InitTypeDef GPIO_InitStructure;
	
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//ʹ�ܶ�ʱ��3��ʱ��
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //need to reset the pin of the two channels of Enoder of right motor
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//need to reset the pin of the two channels of Enoder of right motor
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
  	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA
  	
  	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
 	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ؼ���ģʽ 
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  //��ʼ����ʱ��3
	
  	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3(TIM_ICPolarity_Rising����TIM_ICPolarity_FallingЧ����ͬ������4��Ƶ)
	
  	TIM_ICStructInit(&TIM_ICInitStructure); //��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
  	TIM_ICInitStructure.TIM_ICFilter = 10;  //�����˲�������
  	TIM_ICInit(TIM3, &TIM_ICInitStructure);//���� TIM_ICInitStruct �Ĳ�����ʼ������	TIMx
 
  	TIM_ClearFlag(TIM3, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
  	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//ʹ�ܶ�ʱ���ж�
  	TIM_SetCounter(TIM3,0);
  	TIM_Cmd(TIM3, ENABLE); //ʹ�ܶ�ʱ��
}

void Servo_Init_TIM4(uint32_t psc, uint32_t arr)     //�����ʱ����ʼ��
{
    GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��ʹ��
	                                                                         	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //need to reset,TIM4_CH2,��ʱ��ͨ��2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/****************** 72000000Hz / 2000 / 720=50Hz*******************/
	TIM_TimeBaseStructure.TIM_Period = arr - 1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = psc - 1; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4����	
}

void TIM5_Init_Init(u16 arr,u16 psc)        //�ж��ö�ʱ����ʼ��
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  //�ж����ȼ��Ľṹ�嶨��

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //TIM3��ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx����Ӧ��ģʽ����
 
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);  //ʹ��TIM5�ж�
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM5�жϷ�����������
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ��IRQ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM5, ENABLE);  //����TIM3
}

/*
  С������ֲ�
              ��ͷ

    ��                      ��
            0      1
              ��β
*/
void motor(int motorID, int speed)
{
	if(speed>maxspeed) speed=maxspeed;
	if(speed<-maxspeed) speed=-maxspeed;
	
	if(motorID == 0) 
	{
		if(speed >= 0)
		{
			TIM_SetCompare1(TIM1,velocityPID(Read_EncoderA(),speed));	//�޸ıȽ�ֵ���޸�ռ�ձ�
			TIM_SetCompare2(TIM1,0);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		}
		else
		{
			TIM_SetCompare2(TIM1,-velocityPID(Read_EncoderA(),speed));	//�޸ıȽ�ֵ���޸�ռ�ձ�
			TIM_SetCompare1(TIM1,0);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		}
	}
	else if(motorID == 1)
	{
		if(speed >= 0)
		{
			TIM_SetCompare3(TIM1,velocityPID(Read_EncoderB(),speed));	//�޸ıȽ�ֵ���޸�ռ�ձ�
			TIM_SetCompare4(TIM1,0);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		}
		else
		{
			TIM_SetCompare4(TIM1,-velocityPID(Read_EncoderB(),speed));	//�޸ıȽ�ֵ���޸�ռ�ձ�
			TIM_SetCompare3(TIM1,0);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		}
	}
}

void GoodMoto(int lspeed,int rspeed)//һ·���,��ʱ����˫ͨ������
{
	motor(0,lspeed);
	motor(1,rspeed);	
}

void TIM5_IRQHandler(void)
{ 		    		  			    
	if(TIM5->SR&0X0001)//����ж�
	{ 
        	
	}				   
	TIM5->SR&=~(1<<0);//����жϱ�־λ 	    
}

float Read_EncoderA(void)
{
	int Encoder_TIM = 0;  
	float Speed = 0.0;  
	Encoder_TIM= (short)TIM2 -> CNT;   
	TIM2 -> CNT=0;
	return -Encoder_TIM;
}

float Read_EncoderB(void)
{
	int Encoder_TIM = 0;  
	float Speed = 0.0;  
	Encoder_TIM= (short)TIM3 -> CNT;   
	TIM3 -> CNT=0;
	return Encoder_TIM;
}


void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//����жϱ�־λ 	    
}


int abs_my(int a,int b)//��ľ���ֵ
{
	if((a-b)>=0)
		return a-b;
	else
		return b-a;
}

int velocityPID(int Encoder,int Target){         //�ٶȵ��ڣ�Ϊ��ȷ��С���������ͬ�ٶ���ʵ���ߵ��ٶ�Ҳһ��
	/*��һ����PID����
   float Position_KP=35,Position_KD=440;//������������Ҫ�����ʺ��Լ�С������ֵ��С  //��Ϊû���õ�KI(����)����������ȥ���˺�KI��ص�ʽ��
   static float Bias,Pwm,Last_Bias;
   Encoder=Encoder*20;                       //��Ҫ���ԣ���������������ֵת��Ϊ�ٶ�ֵ��
   Bias=Encoder-Target; //ƫ��=ʵ��ֵ-Ŀ��ֵ
   Pwm=7200-Position_KP*Bias+Position_KD*(Bias-Last_Bias);//λ��ʽPID������//����7200��С�����PWM�����ֵ��Ҳ����������TIM2�Ĵ��δ����������errorΪ��ʱ����ʾС�����ں��ߵ����룬û��ƫ�ƣ�������ʱa=0��ʹ����󷵻ص�PWMֵΪ7200��ʹ����ﵽ����ٶ�
   Last_Bias=Bias;                                       //������һ��ƫ�� 
   if(Pwm>limit){                 //�޷�
	  Pwm=limit;
   }
   else if(Pwm<limit){
	  Pwm=-limit;
   }
   return Pwm;
	*/
	//�������򵥴ֱ��ĵ���
	int Pwm;
	Encoder=Encoder*20;
	while(abs_my(Encoder,Target)>100){
		if(Target>Encoder)   Pwm++;
		if(Encoder>Target)   Pwm--;
	}
}


