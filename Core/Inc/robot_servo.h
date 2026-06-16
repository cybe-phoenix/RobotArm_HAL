/**
  ******************************************************************************
  * @file    robot_servo.h
  * @brief   六自由度机械臂舵机控制模块头文件
  *
  * @details
  *          本模块负责机械臂五个关节舵机的 PWM 输出控制，
  *          包括舵机 PWM 启动、中位初始化、ADC 到 PWM 映射、
  *          以及五路舵机 PWM 同步设置等功能。
  *
  ******************************************************************************
  */

#ifndef __ROBOT_SERVO_H
#define __ROBOT_SERVO_H

#include "main.h"

/* ========================== 舵机参数配置 ========================== */

/* 五个关节舵机的中位 PWM 值，对应 1.5ms 脉宽 */
#define SERVO1_MID 150   // PA6  -> TIM3_CH1
#define SERVO2_MID 150   // PA7  -> TIM3_CH2
#define SERVO3_MID 150   // PB0  -> TIM3_CH3
#define SERVO4_MID 150   // PB1  -> TIM3_CH4
#define SERVO5_MID 150   // PB6  -> TIM4_CH1

/* 12 位 ADC 最大值 */
#define ADC_MAX_VALUE 4095

/* 舵机安全 PWM 范围，避免机械结构顶死 */
#define SERVO_MIN 80
#define SERVO_MAX 220

/* ========================== 函数声明 ========================== */

/**
  * @brief  限制 PWM 值在指定范围内
  * @param  pwm 原始 PWM 值
  * @param  min 最小允许 PWM 值
  * @param  max 最大允许 PWM 值
  * @retval 限幅后的 PWM 值
  */
uint16_t PWM_Limit(int32_t pwm, uint16_t min, uint16_t max);

/**
  * @brief  将 ADC 采样值映射为舵机 PWM 值
  * @param  adc ADC 采样值，范围 0~4095
  * @param  min PWM 最小值
  * @param  max PWM 最大值
  * @param  reverse 是否反向映射，0 为正向，1 为反向
  * @retval 映射并限幅后的 PWM 值
  */
uint16_t ADC_To_PWM(uint16_t adc, uint16_t min, uint16_t max, uint8_t reverse);

/**
  * @brief  启动五个关节舵机 PWM 输出
  * @note   夹爪舵机 TIM4_CH2 由 gripper 模块单独控制
  */
void Servo_PWM_Start_All(void);

/**
  * @brief  将五个关节舵机设置到中位
  */
void Servo_All_Mid(void);

/**
  * @brief  根据五路 ADC 采样值实时更新五个关节舵机位置
  * @param  adc_values 五路 ADC 采样数组
  */
void Servo_Update_From_ADC(uint16_t adc_values[5]);

/**
  * @brief  同步设置五个关节舵机的 PWM 值
  * @param  pwm1 舵机 1 的 PWM 值
  * @param  pwm2 舵机 2 的 PWM 值
  * @param  pwm3 舵机 3 的 PWM 值
  * @param  pwm4 舵机 4 的 PWM 值
  * @param  pwm5 舵机 5 的 PWM 值
  */
void Servo_Set_All_PWM(uint16_t pwm1,
                       uint16_t pwm2,
                       uint16_t pwm3,
                       uint16_t pwm4,
                       uint16_t pwm5);

#endif
