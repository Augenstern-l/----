#include "usart_sensor.h"
#include "usart.h"
#include "dma.h"
#include "main.h"

#include <string.h>
#include <stdio.h>

// DMA接收缓冲区（F103要求4字节对齐）
uint8_t uart_dma_buf[UART_DMA_BUF_LEN] __attribute__((aligned(4)));
static uint8_t frame_buf[UART_FRAME_BUF_LEN] = {0};

// 帧解析内部变量
static uint16_t dma_old_pos = 0;    // 上一次处理的DMA位置
static uint16_t frame_idx = 0;      // 帧缓冲区索引
static uint8_t frame_started = 0;   // 帧起始符&标记
static uint8_t check_sum_recv = 0;  // 接收的校验和
static uint8_t check_sum_calc = 0;  // 计算的校验和

// 全局视觉数据（供外部访问）
int cx = 999, cy = 999;             // 靶心坐标（999=无效）
int lx = 999, ly = 999;             // 激光坐标（OpenMV发送的是激光点）
int last_lx = 999, last_ly = 999;   // 激光坐标记忆（用于激光丢失时继续追踪）
int dist = 0;                       // 靶心-激光距离
uint8_t hit_state = 0;              // 击中状态
uint8_t vision_data_valid = 0;      // 数据有效标记（电机控制依赖）
uint8_t g_hit_lock = 0;             // 命中锁定

// 命中判定与声光提示内部状态
static uint8_t s_hit_confirm_cnt = 0;
static uint8_t s_miss_cnt = 0;
static uint32_t s_alarm_until_tick = 0;

static void Hit_Alarm_On(void)
{
    // 注：PB13 蜂鸣器、PB14 红灯（见 Core/Inc/main.h）
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
}

static void Hit_Alarm_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
}

static void Hit_Miss_Count_Update(void)
{
    if (s_miss_cnt < 0xFF)
    {
        s_miss_cnt++;
    }
    if (hit_state == 1U && s_miss_cnt >= HIT_MISS_FRAMES)
    {
        hit_state = 0U;
        g_hit_lock = 0U; // 脱靶后自动恢复瞄准
    }
}

void Hit_Alarm_Poll(void)
{
    if (s_alarm_until_tick != 0U)
    {
        // 处理 HAL_GetTick() 回绕：用有符号差值判断
        if ((int32_t)(HAL_GetTick() - s_alarm_until_tick) >= 0)
        {
            Hit_Alarm_Off();
            s_alarm_until_tick = 0U;
        }
    }
}

// 非阻塞串口发送函数（调试用）
uint8_t UART1_Send_String(uint8_t *str, uint16_t len)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET)
        return 0;
    return (HAL_UART_Transmit(&huart1, str, len, 10) == HAL_OK) ? 1 : 0;
}

// HAL库DMA接收完成回调（F103循环模式无需重启）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

// HAL库DMA接收半完成回调（可选）
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

static void Vision_Data_Reset_Invalid(void)
{
    vision_data_valid = 0;
    cx = 999; cy = 999;
    lx = 999; ly = 999;
    dist = 0;
}

// 帧解析函数（解析 OpenMV 坐标帧）
// OpenMV: &cx,cy,gx,gy,dist,checksum#
static void Parse_Vision_Frame(void)
{
    frame_buf[frame_idx] = '\0'; // 确保字符串结束

    if (frame_idx < 10)
    {
        // 太短直接丢
        Vision_Data_Reset_Invalid();
        frame_idx = 0;
        frame_started = 0;
        memset(frame_buf, 0, UART_FRAME_BUF_LEN);
        return;
    }

    // 解析 6 个字段（cx,cy,lx,ly,dist,checksum）
    int parse_cnt = sscanf((char *)frame_buf, "%d,%d,%d,%d,%d,%hhu",
                           &cx, &cy, &lx, &ly, &dist, &check_sum_recv);
    if (parse_cnt == 6)
    {
        check_sum_calc = (uint8_t)((cx + cy + lx + ly + dist) & 0xFF);

        if (check_sum_calc == check_sum_recv)
        {
            vision_data_valid = 1;

            if (lx != 0 && ly != 0 && lx != 999 && ly != 999)
            {
                last_lx = lx;
                last_ly = ly;
            }

            // ========== STM32侧“击中”判定（抗抖 + 防闪烁） ==========
            // 激光坐标突然变为 (0,0) 时，忽略该坐标，不参与击中/脱靶计数
            uint8_t ignore_zero_laser = (lx == 0 && ly == 0);
            if (!ignore_zero_laser)
            {
                uint8_t coords_valid = (cx != 0 && cy != 0 && lx != 0 && ly != 0 &&
                                        cx != 999 && cy != 999 && lx != 999 && ly != 999);
                uint8_t hit_candidate = (coords_valid && dist >= 0 && dist <= HIT_THRESHOLD);

                if (hit_candidate)
                {
                    if (s_hit_confirm_cnt < 0xFF) s_hit_confirm_cnt++;
                    s_miss_cnt = 0;
                }
                else
                {
                    s_hit_confirm_cnt = 0;
                    Hit_Miss_Count_Update();
                }

                // 命中确认：只在 0->1 触发一次声光
                if (hit_state == 0U && s_hit_confirm_cnt >= HIT_CONFIRM_FRAMES)
                {
                    hit_state = 1U;
                    g_hit_lock = 1U;
                    s_alarm_until_tick = HAL_GetTick() + HIT_ALARM_HOLD_MS;
                    Hit_Alarm_On();
                }

                // 脱靶确认：连续 miss 才清零
                if (hit_state == 1U && s_miss_cnt >= HIT_MISS_FRAMES)
                {
                    hit_state = 0U;
                    g_hit_lock = 0U; // 脱靶后自动恢复瞄准
                }
            }
        }
        else
        {
            // 校验失败
            Vision_Data_Reset_Invalid();

            s_hit_confirm_cnt = 0;
            Hit_Miss_Count_Update();
        }
    }
    else
    {
        // 解析失败
        Vision_Data_Reset_Invalid();

        s_hit_confirm_cnt = 0;
        Hit_Miss_Count_Update();
    }

    // 重置帧解析状态
    frame_idx = 0;
    frame_started = 0;
    memset(frame_buf, 0, UART_FRAME_BUF_LEN);
}

// 主循环轮询函数（处理DMA接收数据）
void USART_Sensor_Poll(void)
{
    // 原子操作读取DMA当前接收位置
    uint16_t dma_cnt = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    uint16_t dma_pos = UART_DMA_BUF_LEN - dma_cnt;
    uint16_t process_cnt = 0;

    while ((dma_old_pos != dma_pos) && (process_cnt < MAX_PROCESS_CHAR_PER_POLL))
    {
        process_cnt++;
        uint8_t ch = uart_dma_buf[dma_old_pos];
        dma_old_pos = (uint16_t)((dma_old_pos + 1) % UART_DMA_BUF_LEN);

        // 帧起始符
        if (ch == '&')
        {
            frame_idx = 0;
            frame_started = 1;
            memset(frame_buf, 0, UART_FRAME_BUF_LEN);
            continue;
        }

        if (frame_started && (frame_idx < (UART_FRAME_BUF_LEN - 1)))
        {
            frame_buf[frame_idx++] = ch;
        }

        // 帧结束符
        if (ch == '#' && frame_started)
        {
            Parse_Vision_Frame();
        }
    }

    // 非阻塞关闭蜂鸣器/LED
    Hit_Alarm_Poll();
}

// 传感器串口初始化（DMA循环接收）
void USART_Sensor_Init(void)
{
    HAL_UART_DMAStop(&huart1);

    // 启用DMA循环模式（F103：CCR.CIRC）
    huart1.hdmarx->Instance->CCR |= DMA_CCR_CIRC;

    if (HAL_UART_Receive_DMA(&huart1, uart_dma_buf, UART_DMA_BUF_LEN) != HAL_OK)
    {
        Error_Handler();
    }
}
