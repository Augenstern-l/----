#include "key.h"



// ???????????????????
uint8_t key_start_flag = 0;
static uint8_t allow_start_after_rect = 1; // 去掉四点采样，允许长按直接进入瞄准

/**
 * @brief  按键扫描：长按进入瞄准（已去掉四点采样）
 */
void KEY_Scan(void)
{
    typedef enum
    {
        KEY_STATE_IDLE = 0,
        KEY_STATE_DEBOUNCE_PRESS,
        KEY_STATE_PRESSED,
        KEY_STATE_LONG_HOLD,
        KEY_STATE_DEBOUNCE_RELEASE
    } key_state_t;

    static key_state_t state = KEY_STATE_IDLE;
    static uint32_t tick_mark = 0;
    static uint32_t press_start = 0;

    uint8_t pressed = (HAL_GPIO_ReadPin(KEY_START_GPIO_PORT, KEY_START_PIN) == KEY_PRESSED_LEVEL);
    uint32_t now = HAL_GetTick();

    switch (state)
    {
    case KEY_STATE_IDLE:
        if (pressed)
        {
            tick_mark = now;
            state = KEY_STATE_DEBOUNCE_PRESS;
        }
        break;

    case KEY_STATE_DEBOUNCE_PRESS:
        if (!pressed)
        {
            state = KEY_STATE_IDLE;
        }
        else if ((uint32_t)(now - tick_mark) >= KEY_DEBOUNCE_MS)
        {
            press_start = now;
            state = KEY_STATE_PRESSED;
        }
        break;

    case KEY_STATE_PRESSED:
        if (!pressed)
        {
            tick_mark = now;
            state = KEY_STATE_DEBOUNCE_RELEASE;
        }
        else if ((uint32_t)(now - press_start) >= KEY_LONG_PRESS_MS)
        {
            if (allow_start_after_rect)
            {
                key_start_flag = 1;
                state = KEY_STATE_LONG_HOLD;
            }
        }

        if (key_start_flag == 1)
        {
            g_hit_lock = 0;
            close_yaw();
            close_pitch();
        }
        break;

    case KEY_STATE_LONG_HOLD:
        if (!pressed)
        {
            tick_mark = now;
            state = KEY_STATE_DEBOUNCE_RELEASE;
        }

        if (key_start_flag == 1)
        {
            g_hit_lock = 0;
            close_yaw();
            close_pitch();
        }
        break;

    case KEY_STATE_DEBOUNCE_RELEASE:
        if (pressed)
        {
            state = (key_start_flag == 1) ? KEY_STATE_LONG_HOLD : KEY_STATE_PRESSED;
        }
        else if ((uint32_t)(now - tick_mark) >= KEY_DEBOUNCE_MS)
        {
            // 短按不再采样，留空

            press_start = 0;
            state = KEY_STATE_IDLE;
        }
        break;

    default:
        state = KEY_STATE_IDLE;
        press_start = 0;
        break;
    }
}