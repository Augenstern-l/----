#ifndef __steering_engine_H
#define __steering_engine_H

/***头文件***/
#include "stm32f10x.h"

/***宏定义***/
#define duoji_GPIO_PIN			GPIO_Pin_9     		//引脚
#define duoji_GPIO_PORT		GPIOB             		//端口
#define duoji_GPIO_CLK			RCC_APB2Periph_GPIOB    //端口H时钟

/***函数声明***/
void Servo_Control(uint16_t angle);
void Timer4_Init(void);

#endif /*__steering_engine_H*/

