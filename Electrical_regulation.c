#include "Electrical_regulation.h"
#include "tim.h"



// 初始化PID实例
PID_TypeDef yaw_pid = {
    .Kp = YAW_KP,
    .Ki = YAW_KI,
    .Kd = YAW_KD,
    .err = 0.0f,
    .err_last = 0.0f,
    .integral = 0.0f,
    .output = 0.0f,
    .output_max = PID_OUTPUT_MAX,
    .output_min = PID_OUTPUT_MIN,
    .d_filter = 0.0f
};

PID_TypeDef pitch_pid = {
    .Kp = PITCH_KP,
    .Ki = PITCH_KI,
    .Kd = PITCH_KD,
    .err = 0.0f,
    .err_last = 0.0f,
    .integral = 0.0f,
    .output = 0.0f,
    .output_max = PID_OUTPUT_MAX,
    .output_min = PID_OUTPUT_MIN,
    .d_filter = 0.0f
};

float current_yaw_angle = 0.0f;   // 当前水平角度
float current_pitch_angle = 0.0f; // 当前俯仰角度

// ===================== PID核心计算函数 =====================
float PID_Calculate(PID_TypeDef *pid, float set_val, float actual_val)
{
    // 1. 计算当前偏差（设定值-实际值）
    pid->err = set_val - actual_val;

    // 2. 积分项（带积分饱和限制，防止积分溢出）
    if (pid->Ki > 0.0f)
    {
        pid->integral += pid->err;
        // 积分上限：避免积分累积过大导致抖动
        float integral_max = pid->output_max / pid->Ki;
        float integral_min = pid->output_min / pid->Ki;
        if (pid->integral > integral_max)
        {
            pid->integral = integral_max;
        }
        else if (pid->integral < integral_min)
        {
            pid->integral = integral_min;
        }
    }
    else
    {
        pid->integral = 0.0f;
    }

    // 3. 微分项（不使用滤波与采样周期）
    float d_term = pid->err - pid->err_last;

    // 4. PID输出计算（位置式）
    pid->output = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * d_term;

    // 5. 输出限幅
    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    else if (pid->output < pid->output_min)
    {
        pid->output = pid->output_min;
    }

    // 6. 更新上一次偏差
    pid->err_last = pid->err;

    return pid->output;
}

// ===================== PID重置（消除积分累积） =====================
void PID_Reset(PID_TypeDef *pid)
{
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->d_filter = 0.0f;
}

// ===================== 角度计算 =====================
// dx/dy 为像素误差，输出为偏差角度（度）与合成距离偏差
static void calc_angle_from_pixel(int dx, int dy, float *yaw_deg, float *pitch_deg, float *dz)
{
    float dx1 = (CAM_VIEW_W / CAM_RES_W) * (float)dx;  // Δx1 = (A/m) * Δx
    float dy1 = (CAM_VIEW_H / CAM_RES_H) * (float)dy;  // Δy1 = (B/n) * Δy

    if (yaw_deg != NULL)
    {
        *yaw_deg = atanf(dx1 / CAM_DISTANCE_L) * RAD_TO_DEG;  // θ1
    }
    if (pitch_deg != NULL)
    {
        *pitch_deg = atanf(dy1 / CAM_DISTANCE_L) * RAD_TO_DEG; // θ2
    }
    if (dz != NULL)
    {
        *dz = sqrtf(dx1 * dx1 + dy1 * dy1); // ΔZ
    }
}

// ===================== 原有电机驱动/关闭/PWM函数（完全保留） =====================
static void Aim_Countdown_OnDrive(void)
{
    static uint32_t s_last_drive_tick = 0;
    const uint32_t restart_gap_ms = 200U;
    uint32_t now = HAL_GetTick();

    if (s_last_drive_tick == 0U || (uint32_t)(now - s_last_drive_tick) > restart_gap_ms)
    {
        extern void Aim_Motor_Countdown_Reset(void);
        Aim_Motor_Countdown_Reset();
    }

    s_last_drive_tick = now;
}

void yaw_drive(uint8_t DIR, uint32_t Freq)
{
    Aim_Countdown_OnDrive();
    float angle_step = STEP_ANGLE_PER_PULSE * 1.0f;
#if LIMIT_PROTECTION_ENABLE
    // 角度限制判断（改进：只阻止继续越界方向，允许回到范围内）
    if (current_yaw_angle >= YAW_MAX_ANGLE && DIR == 1)
    {
        close_yaw();
        return;
    }
    if (current_yaw_angle <= YAW_MIN_ANGLE && DIR == 0)
    {
        close_yaw();
        return;
    }
#endif

    // 原有驱动逻辑
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_TIM_Base_Start(&htim4);
        PWM_SetDuty_yaw(50);
        if (HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1) != HAL_OK)
        {
            Error_Handler();
        }
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, DIR ? GPIO_PIN_SET : GPIO_PIN_RESET);
    PWM_SetFreq_yaw(Freq);
    float target_angle = current_yaw_angle + (DIR ? angle_step : -angle_step);
#if LIMIT_PROTECTION_ENABLE
    if (target_angle > YAW_MAX_ANGLE) target_angle = YAW_MAX_ANGLE;
    if (target_angle < YAW_MIN_ANGLE) target_angle = YAW_MIN_ANGLE;
#endif
    current_yaw_angle = target_angle;
}

void close_yaw()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_TIM_Base_Stop(&htim4);
    HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
}

void pitch_drive(uint8_t DIR, uint32_t Freq)
{
    Aim_Countdown_OnDrive();
    float angle_step = STEP_ANGLE_PER_PULSE * 1.0f;
#if LIMIT_PROTECTION_ENABLE
    // 角度限制判断（改进：只阻止继续越界方向，允许回到范围内）
    if (current_pitch_angle >= PITCH_MAX_ANGLE && DIR == 1)
    {
        close_pitch();
        return;
    }
    if (current_pitch_angle <= PITCH_MIN_ANGLE && DIR == 0)
    {
        close_pitch();
        return;
    }
#endif

    // 原有驱动逻辑
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_TIM_Base_Start(&htim3);
        PWM_SetDuty_pitch(50);
        if (HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
        {
            Error_Handler();
        }
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, DIR ? GPIO_PIN_SET : GPIO_PIN_RESET);
    PWM_SetFreq_pitch(Freq);
    float target_angle = current_pitch_angle + (DIR ? angle_step : -angle_step);
#if LIMIT_PROTECTION_ENABLE
    if (target_angle > PITCH_MAX_ANGLE) target_angle = PITCH_MAX_ANGLE;
    if (target_angle < PITCH_MIN_ANGLE) target_angle = PITCH_MIN_ANGLE;
#endif
    current_pitch_angle = target_angle;
}

void close_pitch()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_TIM_Base_Stop(&htim3);
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
}

void PWM_SetFreq_yaw(uint32_t frequency)
{
    uint32_t timerClock = HAL_RCC_GetPCLK1Freq();
    uint32_t prescaler = (timerClock / frequency / 0xFFFF) + 1;
    uint32_t period = (timerClock / (prescaler * frequency)) - 1;

    __HAL_TIM_SET_PRESCALER(&htim4, prescaler - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim4, period);
    TIM4->EGR = TIM_EGR_UG;
}

void PWM_SetDuty_yaw(uint8_t dutyCycle)
{
    if (dutyCycle <= 100)
    {
        uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim4);
        uint32_t pulse = (period + 1) * dutyCycle / 100;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);
    }
}

void PWM_SetFreq_pitch(uint32_t frequency)
{
    uint32_t timerClock = HAL_RCC_GetPCLK1Freq();
    uint32_t prescaler = (timerClock / frequency / 0xFFFF) + 1;
    uint32_t period = (timerClock / (prescaler * frequency)) - 1;

    __HAL_TIM_SET_PRESCALER(&htim3, prescaler - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim3, period);
    TIM3->EGR = TIM_EGR_UG;
}

void PWM_SetDuty_pitch(uint8_t dutyCycle)
{
    if (dutyCycle <= 100)
    {
        uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);
        uint32_t pulse = (period + 1) * dutyCycle / 100;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
    }
}

// 角度归零
void angle_calibration(void)
{
    current_yaw_angle = 0.0f;
    current_pitch_angle = 0.0f;
    PID_Reset(&yaw_pid);   // 归零同时重置PID
    PID_Reset(&pitch_pid);
}

void Gimbal_Align_Target(void)
{
    // 靶心丢失计时（静态保持）
    static uint32_t s_last_target_valid_tick = 0;
    static uint32_t s_all_invalid_start_tick = 0;

    // 1) 未启动：关电机、关激光、重置PID
    if (key_start_flag == 0)
    {
        close_yaw();
        close_pitch();
        Ray_off;

        s_last_target_valid_tick = 0U;
        s_all_invalid_start_tick = 0U;

        PID_Reset(&yaw_pid);
        PID_Reset(&pitch_pid);
        return;
    }

    // 命中锁定：优先停止，避免后续逻辑驱动电机
    if (g_hit_lock == 1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
        PID_Reset(&yaw_pid);
        PID_Reset(&pitch_pid);
        return;
    }

    // 启动后打开激光
    Ray_on;

    // 2) 靶心/激光有效性判断
    uint8_t target_valid = (vision_data_valid == 1 && cx != 999 && cy != 999 && cx != 0 && cy != 0);
    uint8_t laser_valid = (lx != 999 && ly != 999 && lx != 0 && ly != 0);

    // 靶心有效：刷新时间戳
    if (target_valid)
    {
        s_last_target_valid_tick = HAL_GetTick();
    }

    if (target_valid || laser_valid)
    {
        s_all_invalid_start_tick = 0;
    }
    else if (s_all_invalid_start_tick == 0U)
    {
        s_all_invalid_start_tick = HAL_GetTick();
    }

    // 靶心无效：
    // - 短暂丢失：只停住不回中
    // - 长时间丢失：才执行回中
    if (!target_valid)
    {
        uint32_t now = HAL_GetTick();
        if (!target_valid && !laser_valid && s_all_invalid_start_tick != 0U &&
            (uint32_t)(now - s_all_invalid_start_tick) > LOST_ALL_TIMEOUT_MS)
        {
            close_yaw();
            close_pitch();
            return;
        }

        close_yaw();
        close_pitch();

        PID_Reset(&yaw_pid);
        PID_Reset(&pitch_pid);

        // 若从未获得过靶心，或已丢失超过阈值，保持停止不回中
        return;
    }

    // 3) 激光无效：使用上一次有效激光坐标继续追靶
    if (lx == 999 || ly == 999 || lx == 0 || ly == 0)
    {
        if (last_lx != 999 && last_ly != 999)
        {
            lx = last_lx;
            ly = last_ly;
        }
        else
        {
            close_yaw();
            close_pitch();

            PID_Reset(&yaw_pid);
            PID_Reset(&pitch_pid);
            return;
        }
    }

    // 4) 计算像素误差（靶心 - 激光）
    int dx = cx - lx;
    int dy = cy - ly;
    int adx = (dx >= 0) ? dx : -dx;
    int ady = (dy >= 0) ? dy : -dy;

    // 5) 近距离点动微调已禁用，继续使用PID驱动

    // 7) 远距离：正常PID速度控制（PID只算幅值，方向由 dx/dy 决定）
    float yaw_angle = 0.0f;
    float pitch_angle = 0.0f;
    calc_angle_from_pixel(dx, dy, &yaw_angle, &pitch_angle, NULL);

    float yaw_err_mag   = fabsf(yaw_angle);
    float pitch_err_mag = fabsf(pitch_angle);

    // 设定值必须是 0.0f（之前写 1.0f 会导致永远对不准）
    float yaw_pid_out   = PID_Calculate(&yaw_pid, 0.0f, yaw_err_mag);
    float pitch_pid_out = PID_Calculate(&pitch_pid, 0.0f, pitch_err_mag);

    // 远快近慢：根据像素误差选择基础频率
    uint32_t yaw_base = (adx > SPEED_SWITCH_PIX) ? YAW_BASE_FREQ_FAR : YAW_BASE_FREQ_NEAR;
    uint32_t pitch_base = (ady > SPEED_SWITCH_PIX) ? PITCH_BASE_FREQ_FAR : PITCH_BASE_FREQ_NEAR;

    uint32_t yaw_freq   = yaw_base + (uint32_t)yaw_pid_out;
    uint32_t pitch_freq = pitch_base + (uint32_t)pitch_pid_out;

    // 限幅
    if (yaw_freq > MAX_FREQ) yaw_freq = MAX_FREQ;
    if (pitch_freq > MAX_FREQ) pitch_freq = MAX_FREQ;

    // 方向（支持反转宏）
    uint8_t yaw_dir_raw = (dx > 0) ? 0 : 1;
    uint8_t pitch_dir_raw = (dy > 0) ? 0 : 1;

    uint8_t yaw_dir = YAW_DIR_INVERT ? (!yaw_dir_raw) : yaw_dir_raw;
    uint8_t pitch_dir = PITCH_DIR_INVERT ? (!pitch_dir_raw) : pitch_dir_raw;

    // 8) 执行驱动（死区可选）
#if DEADZONE_ENABLE
    static uint8_t yaw_in_deadzone = 0;
    static uint8_t pitch_in_deadzone = 0;
    static uint8_t yaw_on_cnt = 0;
    static uint8_t yaw_off_cnt = 0;
    static uint8_t pitch_on_cnt = 0;
    static uint8_t pitch_off_cnt = 0;

    if (!yaw_in_deadzone)
    {
        if (adx <= DEVIATION_THRESHOLD)
        {
            if (yaw_on_cnt < 0xFF) yaw_on_cnt++;
            if (yaw_on_cnt >= DEVIATION_HYSTERESIS_ON_FRAMES)
            {
                yaw_in_deadzone = 1;
                yaw_on_cnt = 0;
            }
        }
        else
        {
            yaw_on_cnt = 0;
        }
    }
    else
    {
        if (adx > DEVIATION_HYSTERESIS_OFF_THRESH)
        {
            if (yaw_off_cnt < 0xFF) yaw_off_cnt++;
            if (yaw_off_cnt >= DEVIATION_HYSTERESIS_OFF_FRAMES)
            {
                yaw_in_deadzone = 0;
                yaw_off_cnt = 0;
            }
        }
        else
        {
            yaw_off_cnt = 0;
        }
    }

    if (!pitch_in_deadzone)
    {
        if (ady <= DEVIATION_THRESHOLD)
        {
            if (pitch_on_cnt < 0xFF) pitch_on_cnt++;
            if (pitch_on_cnt >= DEVIATION_HYSTERESIS_ON_FRAMES)
            {
                pitch_in_deadzone = 1;
                pitch_on_cnt = 0;
            }
        }
        else
        {
            pitch_on_cnt = 0;
        }
    }
    else
    {
        if (ady > DEVIATION_HYSTERESIS_OFF_THRESH)
        {
            if (pitch_off_cnt < 0xFF) pitch_off_cnt++;
            if (pitch_off_cnt >= DEVIATION_HYSTERESIS_OFF_FRAMES)
            {
                pitch_in_deadzone = 0;
                pitch_off_cnt = 0;
            }
        }
        else
        {
            pitch_off_cnt = 0;
        }
    }

    if (!yaw_in_deadzone)
    {
        yaw_drive(yaw_dir, yaw_freq);
    }
    else
    {
        close_yaw();
    }

    if (!pitch_in_deadzone)
    {
        pitch_drive(pitch_dir, pitch_freq);
    }
    else
    {
        close_pitch();
    }
#else
    yaw_drive(yaw_dir, yaw_freq);
    pitch_drive(pitch_dir, pitch_freq);
#endif
}
















