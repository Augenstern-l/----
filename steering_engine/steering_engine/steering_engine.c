//#include "steering_engine.h"

///***
//	*控制舵机转动角度
//	*
//	*
//	*/
//void Servo_Control(uint16_t angle)
//{
//	float temp;
//	temp = ((3.0 / 10.0) * angle + 5.0);//寄存器值 = 1/9 * 角度 + 5
//	TIM_SetCompare4(TIM4, (uint16_t)temp);//将角度值写入比较寄存器
//}

///***
//	*定时器4初始化
//	*生成50hzPWM驱动舵机
//	*
//	*/
//void Timer4_Init(void)
//{	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
// 	RCC_APB2PeriphClockCmd(duoji_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);//使能PB外设和AFIO复用功能模块时钟使能
//	                                                                         	
//   //设置该引脚为复用输出功能,输出TIM4的CH4的PWM脉冲波形
//	GPIO_InitStructure.GPIO_Pin = duoji_GPIO_PIN; 		//TIM4_CH4
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	//复用推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(duoji_GPIO_PORT, &GPIO_InitStructure);
//	/****************** 72MHz / 200 / 7200 = 50Hz*******************/
//	TIM_TimeBaseStructure.TIM_Period = 800; 		//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
//	TIM_TimeBaseStructure.TIM_Prescaler = 1800; 	//设置用来作为TIMx时钟频率除数的预分频值  
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 		//设置时钟分割:TDTS = Tck_tim
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
//	 
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 	//选择定时器模式:TIM脉冲宽度调制模式2
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
//    TIM_OCInitStructure.TIM_Pulse = 0;					//初始状态占空比为0
//	TIM_OC4Init(TIM4, &TIM_OCInitStructure); 			//根据TIM_OCInitStruct中指定的参数初始化外设TIMx
//	
//	TIM_Cmd(TIM4, ENABLE);  //使能TIM4外设	
//}

