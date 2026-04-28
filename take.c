#include "take.h"
#include "oled.h"
#include "usart_sensor.h"
#include "Electrical_regulation.h"
#include "key.h"
#include "main.h"

static uint32_t g_aim_countdown_start_tick = 0;
static uint8_t g_aim_countdown_active = 0;
static uint8_t g_hit_display_lock = 0;
static uint8_t g_countdown_display_sec = 0;
static uint32_t g_pc13_blink_tick = 0;

static void PC13_Led_Blink_1Hz(uint32_t now_tick)
{
    if ((now_tick - g_pc13_blink_tick) >= 500U)
    {
        g_pc13_blink_tick = now_tick;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

void Aim_Motor_Countdown_Reset(void)
{
    g_aim_countdown_start_tick = HAL_GetTick();
    g_aim_countdown_active = 1;
    g_hit_display_lock = 0;
    g_countdown_display_sec = 10;
}

// 主循环（核心逻辑）
void loop(void)
{
    uint32_t now_tick = HAL_GetTick();

    PC13_Led_Blink_1Hz(now_tick);

    /* 1. 按键扫描（检测一键启动） */
    KEY_Scan();


    /* 2. 未启动→提示长按进入瞄准 */
    if (key_start_flag == 0)
    {
        g_aim_countdown_active = 0;
        g_hit_display_lock = 0;
        g_countdown_display_sec = 0;
        OLED_operate_gram(PEN_CLEAR);
        OLED_show_string(0, 0, "System Ready", 12);
        OLED_show_string(2, 0, "Hold PA2", 12);
        OLED_show_string(2, 9, "to Aim", 12);
        OLED_refresh_gram();
        return;
    }    
    // 始终轮询传感器，保证采样坐标更新
    USART_Sensor_Poll();

    /* 3. 已启动→执行工作逻辑 */
    Gimbal_Align_Target();

    /* 4. OLED显示坐标/距离/击中状态 */
    OLED_operate_gram(PEN_CLEAR);

    oled_show();

    if (g_hit_lock)
    {
        g_hit_display_lock = 1;
    }

    if (g_hit_display_lock)
    {
        OLED_show_string(3, 7, "HIT", 12);
    }
    else
    {
        OLED_show_string(3, 7, "MISS", 12);
    }

    // 显示距离（用于标定 HIT_THRESHOLD）
    OLED_show_string(4, 6, "D:", 12);
    OLED_show_num(4, 9, dist, 0, 3, 12);

    if (g_hit_display_lock)
    {
        g_aim_countdown_active = 0;
    }

    {
        if (g_aim_countdown_active)
        {
            uint32_t elapsed_ms = now_tick - g_aim_countdown_start_tick;
            if (elapsed_ms < 10000U)
            {
                g_countdown_display_sec = (10000U - elapsed_ms + 999U) / 1000U;
            }
            else
            {
                g_countdown_display_sec = 0;
            }
        }

        OLED_show_string(4, 0, "T:", 12);
        OLED_show_num(4, 2, g_countdown_display_sec, 0, 2, 12);
    }

    OLED_refresh_gram();
}

// 初始化函数
void loop_init(void)
{
    /* 1. 传感器串口初始化 */
    USART_Sensor_Init();

    g_pc13_blink_tick = HAL_GetTick();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /* 2. OLED初始化 */
    OLED_init();
    
    /* 3. 激光初始关闭 */
    Ray_on;

    /* 4. OLED显示就绪提示 */
    OLED_operate_gram(PEN_CLEAR);
    OLED_show_string(0, 0, "System Ready", 12);
    OLED_refresh_gram();
    HAL_Delay(500);
}


void oled_show(void){
   
    OLED_show_chinese_16x16(0, 0, 2); // 坐
    OLED_show_chinese_16x16(0, 1,3); // 标


    OLED_show_string(0, 6, "CX:", 12);
    OLED_show_num(0, 9, cx, 0, 3, 12);

    OLED_show_string(0, 13, "CY:", 12);
    OLED_show_num(0, 16, cy, 0, 3, 12);

    OLED_show_string(1, 6, "LX:", 12);
    OLED_show_num(1, 9, lx, 0, 3, 12);

    OLED_show_string(1, 13, "LY:", 12);
    OLED_show_num(1, 16, ly, 0, 3, 12);

    OLED_show_chinese_16x16(2, 0, 4); // 状
    OLED_show_chinese_16x16(2, 1, 5); // 态
    OLED_show_chinese_16x16(2, 2, 6); // :
}