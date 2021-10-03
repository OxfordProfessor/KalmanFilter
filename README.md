本次实验采用了卡尔曼滤波，对相关数据进行滤波处理

同时，本次尝试使用编码器对车辆速度进行采集，对采集数据进行串口传输到电脑，然后使用Origin数据处理工具对数据进行处理

并尝试使用高级定时器（TIM1\TIM8）输出PWM波形，然后采用位置式PID算法结合速度采集对小车速度进行校准

本次实验设计的内容：

    1. 高级定时器(TIM1\TIM8)的使用（包括时钟分割与采集频率）
    
    2. PID算法调节速度输出与函数封装
    
    3. 串口调节使用基础
    
    4. 定时器的计数模式的使用
    
    5. 卡尔曼滤波的参数整定
    
    6. 修改系列芯片需要修改的设置
CSDN：

    https://blog.csdn.net/qq_42680785/article/details/101946624?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522163324344816780357278719%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=163324344816780357278719&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-101946624.first_rank_v2_pc_rank_v29&utm_term=%E6%9B%B4%E6%94%B9%E8%8A%AF%E7%89%87&spm=1018.2226.3001.4187）

卡尔曼滤波的效果：
   ![image](https://github.com/OxfordProfessor/KalmanFilter/blob/main/Graph2.png)
   
   ![image](https://github.com/OxfordProfessor/KalmanFilter/blob/main/Graph3.png)
   
编码器的使用：

   ![image](https://github.com/OxfordProfessor/KalmanFilter/blob/main/20210119214056467.png)
   
    void Encoder_Init_TIM2(void){
   
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
   
    float Read_EncoderA(void){
   
	int Encoder_TIM = 0;         //初始化
	float Speed = 0.0;  
	Encoder_TIM= (short)TIM2 -> CNT;      //重装初始值，CNT寄存器的值赋予Encoder_TIM.
	TIM2 -> CNT=0;                  //寄存器清零
	return -Encoder_TIM;
    
    }
   
高级定时器的使用：

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
   
占空比与采样控制----------------------------------------------------------------------
      
    根据定时器时钟bai的频率，比如时钟的频率du是72MHZ，可以理解为一秒钟STM32会自己zhi数72M次，dao预分频系数就是将频率分割，比如分频系数是72，则该时钟的频率会变成72MHZ/72=1MHZ，但是在设置的时候要注意，数值应该是72-1。假定分频系数是72-1，那么频率变成1MHZ，也就意味着STM32在一秒钟会数1M次，即1us数一次。好了，接下来就是确定预装载值，比如需要定时1ms，由于1ms=1us*1000,那么预装载值就是1000-1；如此类推，在预分频系数确定的情况下，定时的时长就由预装载值确定了。至于要把值减一的原因，是计数是从0开始，所以要减一。而这时如果要改变占空比比为50％，那么比较定时器值就要设为500.时钟分割是指几次采样有效，我们默认一次.

串口传输注意事项-----------------------------------------------------------------------
    
    首先，串口该部分为接收数据部分
    
![image](https://github.com/OxfordProfessor/KalmanFilter/blob/main/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202021-10-03%20143938.png)
    
    进而，接受数据部分为
    
![image](https://github.com/OxfordProfessor/KalmanFilter/blob/main/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202021-10-03%20144014.png)
    
    初始化过程中，一定要保证初始化的波特率与软件一致，其次，如果PA9与PA10通过跳线帽与UART1 TX与RX连接，一定要带上跳线帽，CH340两个通信引脚设计时不用接在这两个引脚上
    
![image](https://github.com/OxfordProfessor/KalmanFilter/blob/main/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202021-10-03%20144036.png)
    
    程序可以直接移植正点原子程序或本程序

程序的卡尔曼滤波部分---------------------------------------------------------------------

    卡尔曼滤波的结构体定义部分放在h文件中，并且一定要在尾部extern，其余文件引用时也要extern
    
![image](https://github.com/OxfordProfessor/KalmanFilter/blob/main/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202021-10-03%20144435.png)
    
![image](https://github.com/OxfordProfessor/KalmanFilter/blob/main/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202021-10-03%20144506.png)
    
PID速度调节部分----------------------------------------------------------------------

    int velocityPID(int Encoder,int Target){         //速度调节，为了确保小车在软件相同速度下实际走的速度也一致
	  int Pwm;
	  Encoder=Encoder*20;
	  while(abs_my(Encoder,Target)>100){
		if(Target>Encoder)   Pwm++;
		if(Encoder>Target)   Pwm--;
	  }
    }
    
    void GoodMoto(int lspeed,int rspeed)//一路电机,定时器的双通道控制
    {
	  motor(0,lspeed);
	  motor(1,rspeed);	
    }
    
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

    
