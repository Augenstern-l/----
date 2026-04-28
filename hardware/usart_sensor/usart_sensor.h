#ifndef __USART_SENSOR_H
#define __USART_SENSOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// ===================== 串口DMA/帧缓存参数 =====================
#define UART_DMA_BUF_LEN            128
#define UART_FRAME_BUF_LEN          64
#define MAX_PROCESS_CHAR_PER_POLL   128

// ===================== 击中判定参数（STM32侧） =====================
// OpenMV 会发送 dist(像素距离)。STM32 侧根据阈值判定是否“击中”。
// 建议：阈值需根据实际靶面大小/距离进行标定。
#define HIT_THRESHOLD               4     // dist <= HIT_THRESHOLD 判定为“命中候选”
#define HIT_CONFIRM_FRAMES          15       // 连续命中候选帧数达到该值才置 hit_state=1（抗抖）
#define HIT_MISS_FRAMES             30      // 连续非命中/无效帧达到该值才清 hit_state（防闪烁）

// 声光提示保持时间（ms）
#define HIT_ALARM_HOLD_MS           800

// ===================== 全局视觉数据（供电机/显示调用） =====================
extern int cx, cy;                   // 靶心坐标
extern int lx, ly;                   // 激光坐标
extern int last_lx, last_ly;         // 激光坐标记忆（用于激光丢失时继续追踪）
extern int dist;                     // 靶心-激光距离
extern uint8_t hit_state;            // 击中状态：1=击中，0=未击中
extern uint8_t g_hit_lock;           // 命中锁定：1=锁定停机，0=未锁定
extern uint8_t vision_data_valid;    // 数据有效标记：1=有效，0=无效

// ===================== 接口函数声明 =====================
uint8_t UART1_Send_String(uint8_t *str, uint16_t len);
void USART_Sensor_Init(void);
void USART_Sensor_Poll(void);

// 命中声光提示轮询（在主循环周期调用，用于非阻塞关闭蜂鸣器/LED）
void Hit_Alarm_Poll(void);

#endif
