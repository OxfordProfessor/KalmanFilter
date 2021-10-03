#include "Init.h"
#include "contral.h"
#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为F103的定时器是16位的。
#define maxspeed 600
#define limit 800
/*
  编码器需要占用两个通道，需要两个定时器（具有计数功能的定时器1，2，3，4，5和8），1，2为
  高级定时器，6，7为基本定时器（只有向上计数功能，故弃之不用）。
  本例中选择定时器2，3为编码器（两个电机）采集定时器，选择定时器1，8为后驱电机用PWM定时器，
  前轮只负责舵机转向，舵机PWM使用定时器4。中断控制函数用定时器5。
  
  尝试使用高级定时器控制电机
*/


void TIM1_PWM_Init(u16 per,u16 psc)        //电机PWM初始化
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

	//此处结构体必须全部配置，否则输出不了PWM波！！！！
	TIM_OCInitStructure.TIM_Pulse = 0; 		//待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //TIM1脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//使能输出比较状态
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//使能输出比较N状态
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM1 输出比较极性高
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//TIM1 输出比较N极性高
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//当 MOE=0 重置 TIM1 输出比较空闲状态
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//当 MOE=0 重置 TIM1 输出比较 N 空闲状态
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); //输出比较通道1初始化  
    
	//此处结构体必须全部配置，否则输出不了PWM波！！！！
	TIM_OCInitStructure.TIM_Pulse = 0; 		//待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //TIM1脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//使能输出比较状态
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//使能输出比较N状态
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM1 输出比较极性高
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//TIM1 输出比较N极性高
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//当 MOE=0 重置 TIM1 输出比较空闲状态
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//当 MOE=0 重置 TIM1 输出比较 N 空闲状态
	TIM_OC2Init(TIM1, &TIM_OCInitStructure); //输出比较通道2初始化 
	
	//此处结构体必须全部配置，否则输出不了PWM波！！！！
	TIM_OCInitStructure.TIM_Pulse = 0; 		//待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //TIM1脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//使能输出比较状态
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//使能输出比较N状态
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM1 输出比较极性高
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//TIM1 输出比较N极性高
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//当 MOE=0 重置 TIM1 输出比较空闲状态
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//当 MOE=0 重置 TIM1 输出比较 N 空闲状态
	TIM_OC3Init(TIM1, &TIM_OCInitStructure); //输出比较通道3初始化
	
	//此处结构体必须全部配置，否则输出不了PWM波！！！！
	TIM_OCInitStructure.TIM_Pulse = 0; 		//待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //TIM1脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//使能输出比较状态
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//使能输出比较N状态
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM1 输出比较极性高
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//TIM1 输出比较N极性高
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//当 MOE=0 重置 TIM1 输出比较空闲状态
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//当 MOE=0 重置 TIM1 输出比较 N 空闲状态
	TIM_OC4Init(TIM1, &TIM_OCInitStructure); //输出比较通道4初始化
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能TIMx在 CCR1 上的预装 
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能TIMx在 CCR2 上的预装
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能TIMx在 CCR3 上的预装
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能TIMx在 CCR4 上的预装
	TIM_ARRPreloadConfig(TIM1, ENABLE);//使能预装载寄存器
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	TIM_Cmd(TIM1, ENABLE);  //使能定时器	
}


/**************************************************************************
函数功能：把TIM2初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//need to reset the pin of the two channels of Enoder of left motor
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	//need to reset the pin of the two channels of Enoder of right motor
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);		//need to reset the pin of the two channels of Enoder of right motor
 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//边沿计数模式 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);  //初始化定时器2
	
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	
	TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
	TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
	TIM_ICInit(TIM2, &TIM_ICInitStructure);//根据 TIM_ICInitStruct 的参数初始化外设	TIMx
 
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//使能定时器中断
	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE); //使能定时器2
}

/**************************************************************************
函数功能：把TIM3初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM3(void)
{
  	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  	TIM_ICInitTypeDef TIM_ICInitStructure;  
  	GPIO_InitTypeDef GPIO_InitStructure;
	
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//使能定时器3的时钟
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //need to reset the pin of the two channels of Enoder of right motor
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//need to reset the pin of the two channels of Enoder of right motor
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
  	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA
  	
  	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
 	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//边沿计数模式 
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  //初始化定时器3
	
  	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3(TIM_ICPolarity_Rising或者TIM_ICPolarity_Falling效果相同，都是4倍频)
	
  	TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
  	TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
  	TIM_ICInit(TIM3, &TIM_ICInitStructure);//根据 TIM_ICInitStruct 的参数初始化外设	TIMx
 
  	TIM_ClearFlag(TIM3, TIM_FLAG_Update);//清除TIM的更新标志位
  	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//使能定时器中断
  	TIM_SetCounter(TIM3,0);
  	TIM_Cmd(TIM3, ENABLE); //使能定时器
}

void Servo_Init_TIM4(uint32_t psc, uint32_t arr)     //舵机定时器初始化
{
    GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟使能
	                                                                         	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //need to reset,TIM4_CH2,定时器通道2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/****************** 72000000Hz / 2000 / 720=50Hz*******************/
	TIM_TimeBaseStructure.TIM_Period = arr - 1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler = psc - 1; //设置用来作为TIMx时钟频率除数的预分频值  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx

	TIM_Cmd(TIM4, ENABLE);  //使能TIM4外设	
}

void TIM5_Init_Init(u16 arr,u16 psc)        //中断用定时器初始化
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  //中断优先级的结构体定义

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //TIM3的时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的相应的模式配置
 
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);  //使能TIM5中断
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM5中断服务函数的配置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;  //主优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级2级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能IRQ中断通道
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化NVIC寄存器

	TIM_Cmd(TIM5, ENABLE);  //开启TIM3
}

/*
  小车电机分布
              车头

    左                      右
            0      1
              车尾
*/
void motor(int motorID, int speed)
{
	if(speed>maxspeed) speed=maxspeed;
	if(speed<-maxspeed) speed=-maxspeed;
	
	if(motorID == 0) 
	{
		if(speed >= 0)
		{
			TIM_SetCompare1(TIM1,velocityPID(Read_EncoderA(),speed));	//修改比较值，修改占空比
			TIM_SetCompare2(TIM1,0);	//修改比较值，修改占空比
		}
		else
		{
			TIM_SetCompare2(TIM1,-velocityPID(Read_EncoderA(),speed));	//修改比较值，修改占空比
			TIM_SetCompare1(TIM1,0);	//修改比较值，修改占空比
		}
	}
	else if(motorID == 1)
	{
		if(speed >= 0)
		{
			TIM_SetCompare3(TIM1,velocityPID(Read_EncoderB(),speed));	//修改比较值，修改占空比
			TIM_SetCompare4(TIM1,0);	//修改比较值，修改占空比
		}
		else
		{
			TIM_SetCompare4(TIM1,-velocityPID(Read_EncoderB(),speed));	//修改比较值，修改占空比
			TIM_SetCompare3(TIM1,0);	//修改比较值，修改占空比
		}
	}
}

void GoodMoto(int lspeed,int rspeed)//一路电机,定时器的双通道控制
{
	motor(0,lspeed);
	motor(1,rspeed);	
}

void TIM5_IRQHandler(void)
{ 		    		  			    
	if(TIM5->SR&0X0001)//溢出中断
	{ 
        	
	}				   
	TIM5->SR&=~(1<<0);//清除中断标志位 	    
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
	if(TIM2->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//清除中断标志位 	    
}


int abs_my(int a,int b)//差的绝对值
{
	if((a-b)>=0)
		return a-b;
	else
		return b-a;
}

int velocityPID(int Encoder,int Target){         //速度调节，为了确保小车在软件相同速度下实际走的速度也一致
	/*法一：类PID调节
   float Position_KP=35,Position_KD=440;//参数整定，需要调到适合自己小车的数值大小  //因为没有用到KI(积分)，所以这里去除了和KI相关的式子
   static float Bias,Pwm,Last_Bias;
   Encoder=Encoder*20;                       //需要测试，将编码器读出的值转换为速度值。
   Bias=Encoder-Target; //偏差=实际值-目标值
   Pwm=7200-Position_KP*Bias+Position_KD*(Bias-Last_Bias);//位置式PID控制器//这里7200是小车电机PWM的最大值，也就是主函数TIM2的传参传入的数，当error为零时，表示小车处于黑线的中央，没有偏移，所以这时a=0，使得最后返回的PWM值为7200，使电机达到最大速度
   Last_Bias=Bias;                                       //保存上一次偏差 
   if(Pwm>limit){                 //限幅
	  Pwm=limit;
   }
   else if(Pwm<limit){
	  Pwm=-limit;
   }
   return Pwm;
	*/
	//法二：简单粗暴的调节
	int Pwm;
	Encoder=Encoder*20;
	while(abs_my(Encoder,Target)>100){
		if(Target>Encoder)   Pwm++;
		if(Encoder>Target)   Pwm--;
	}
}


