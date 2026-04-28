#ifndef __ELECTRICAL_REGULATION_H
#define __ELECTRICAL_REGULATION_H

#include "main.h"
#include "usart_sensor.h"
#include "key.h"
#include <math.h>

// ===================== 电机参数（32细分） ====aun============
#define STEP_ANGLE_BASE     1.8f        // 电机基础步距角
#define MOTOR_SUB_DIVISION  32          // 32细分
#define STEP_ANGLE_PER_PULSE (STEP_ANGLE_BASE / MOTOR_SUB_DIVISION) // 每步角度 0.05625°

// ===================== 相机参数（需实测校准） =====================
#define CAM_RES_W           320         // 水平分辨率 QVGA/240*320 QQVGA/160*120
#define CAM_RES_H           240         // 垂直分辨率 QVGA
#define CAM_FOV_W           70.8f      // 水平视场角 
#define CAM_FOV_H           55.6f      // 垂直视场角  
#define PIXEL_TO_ANGLE_W    (CAM_FOV_W / CAM_RES_W) // 水平1像素对应角度
#define PIXEL_TO_ANGLE_H    (CAM_FOV_H / CAM_RES_H) // 垂直1像素对应角度

// ===================== 角度计算（论文式3-10~3-14） =====================
// A/B 为实际视场平面长度/宽度，L 为目标与摄像头距离
#define CAM_VIEW_W          170.00f     // 实际视场平面长度 A
#define CAM_VIEW_H          126.00f     // 实际视场平面宽度 B
#define CAM_DISTANCE_L      120.0f      // 目标与摄像头距离 L
#define RAD_TO_DEG          (57.29577951308232f) // 180 / PI

// ===================== 控制参数 =====================
// ===================== 死区开关（0=禁用，1=启用） =====================
#define DEADZONE_ENABLE       0
#define DEVIATION_THRESHOLD   2          // 像素偏差死区（<1像素视为对准）
#define DEVIATION_HYSTERESIS_ON_FRAMES  2  // 连续进入死区帧数阈值
#define DEVIATION_HYSTERESIS_OFF_THRESH 2  // 退出死区的像素阈值（>该值才恢复运动）
#define DEVIATION_HYSTERESIS_OFF_FRAMES 10  // 连续超出阈值帧数

#define YAW_BASE_FREQ       YAW_BASE_FREQ_NEAR // PID输出上限计算基准（按近距离基础频率）
#define YAW_BASE_FREQ_NEAR  7           // 近距离基础频率（慢）
#define YAW_BASE_FREQ_FAR   60         // 远距离基础频率（快）
#define PITCH_BASE_FREQ_NEAR 7
#define PITCH_BASE_FREQ_FAR  60
#define SPEED_SWITCH_PIX     30          // 速度切换像素阈值（|dx|或|dy|超过则用远距离频率）
#define MAX_FREQ            100         // 电机最大安全频率
#define PID_OUTPUT_MAX      (MAX_FREQ - YAW_BASE_FREQ) // PID输出上限
#define PID_OUTPUT_MIN      10          // PID输出下限（无反向输出）

// ===================== 方向反转开关（按需置1） =====================
// 若出现“越调越偏”，把对应轴改为1
#define YAW_DIR_INVERT            0
#define PITCH_DIR_INVERT          0

// ===================== 软限位开关（0=禁用，1=启用） =====================
#define LIMIT_PROTECTION_ENABLE   0

// ===================== 目标丢失处理（防“回中往复”振荡） =====================
// OpenMV 视觉在某些帧可能输出 0/无效坐标或出现短暂校验失败。
// 若每次无效都立即回中，会出现“对准后又回到最初位置”的往复振荡。
// 这里设置：短暂丢失仅停止不回中；长时间丢失才回中。
#define TARGET_LOST_TIMEOUT_MS  2000U    // 连续无有效靶心超过该时间仍保持停止
#define LOST_ALL_TIMEOUT_MS     3000U    // 靶心+激光均无效超过该时间保持停止

// ===================== PID参数 =====================
#define YAW_KP              0.12f        // 水平轴比例系数
#define YAW_KI              0.00f        // 水平轴积分系数
#define YAW_KD              0.02f        // 水平轴微分系数
#define PITCH_KP            0.04f        // 俯仰轴比例系数
#define PITCH_KI            0.00f       // 俯仰轴积分系数
#define PITCH_KD            0.015f       // 俯仰轴微分系数

// PID采样周期与微分滤波参数已移除（当前PID不依赖采样周期与滤波）

// ===================== 角度限制参数 =====================
#define YAW_MAX_ANGLE       90.0f       // 水平轴最大角度
#define YAW_MIN_ANGLE      -90.0f
#define PITCH_MAX_ANGLE     45.0f       // 俯仰轴最大角度
#define PITCH_MIN_ANGLE    -45.0f
extern float current_yaw_angle;   // 当前水平角度
extern float current_pitch_angle; // 当前俯仰角度

// ===================== 激光引脚定义 =====================
#define Ray_on        HAL_GPIO_WritePin(ray_GPIO_Port, ray_Pin, GPIO_PIN_SET)
#define Ray_off       HAL_GPIO_WritePin(ray_GPIO_Port, ray_Pin, GPIO_PIN_RESET)

// ===================== PID结构体定义 =====================
typedef struct
{
    float Kp;            // 比例系数
    float Ki;            // 积分系数
    float Kd;            // 微分系数
    float err;           // 当前偏差
    float err_last;      // 上一次偏差
    float integral;      // 积分项
    float output;        // PID输出
    float output_max;    // 输出上限
    float output_min;    // 输出下限
    float d_filter;      // 微分滤波输出
} PID_TypeDef;

// 声明PID实例
extern PID_TypeDef yaw_pid;    // 水平轴PID
extern PID_TypeDef pitch_pid;  // 俯仰轴PID

// 函数声明
void yaw_drive(uint8_t DIR, uint32_t Freq);
void pitch_drive(uint8_t DIR, uint32_t Freq);
void close_yaw(void);
void close_pitch(void);
void PWM_SetFreq_yaw(uint32_t frequency);
void PWM_SetFreq_pitch(uint32_t frequency);
void PWM_SetDuty_yaw(uint8_t dutyCycle);
void PWM_SetDuty_pitch(uint8_t dutyCycle);
void Gimbal_Align_Target(void);
void angle_calibration(void);
float PID_Calculate(PID_TypeDef *pid, float set_val, float actual_val);
void PID_Reset(PID_TypeDef *pid); // PID重置

void Gimbal_Fine_Adjust(int dx, int dy);
#endif
