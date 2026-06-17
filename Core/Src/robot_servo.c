/**
  ******************************************************************************
  * @file    robot_servo.c
  * @brief   六自由度机械臂舵机控制模块源文件
  *
  * @details
  *          本文件实现五个关节舵机的底层 PWM 控制逻辑。
  *          通过 TIM3_CH1~CH4 和 TIM4_CH1 输出 PWM，
  *          并将 ADC 采样值映射为舵机 PWM 占空比。
  *
  ******************************************************************************
  */

#include "robot_servo.h"
#include "tim.h"

// 舵机 PWM 死区
#define SERVO_PWM_DEADBAND  2 // 声音稍微大一点，但小幅控制更顺滑。
// #define SERVO_PWM_DEADBAND  3 // 声音更小，但小幅转动会有顿挫感。

/**
  * @brief  限制 PWM 值在指定范围内
  * @param  pwm 原始 PWM 值
  * @param  min 最小允许 PWM 值
  * @param  max 最大允许 PWM 值
  * @retval 限幅后的 PWM 值
  */
uint16_t PWM_Limit(int32_t pwm, uint16_t min, uint16_t max)
{
    if (pwm < min) pwm = min;
    if (pwm > max) pwm = max;
    return (uint16_t)pwm;
}

/**
  * @brief  将 ADC 采样值映射为舵机 PWM 值
  * @param  adc ADC 采样值，范围 0~4095
  * @param  min PWM 最小值
  * @param  max PWM 最大值
  * @param  reverse 映射方向，0 表示正向，1 表示反向
  * @retval 映射并限幅后的 PWM 值
  */
uint16_t ADC_To_PWM(uint16_t adc, uint16_t min, uint16_t max, uint8_t reverse)
{
    int32_t pwm;

    if (adc > ADC_MAX_VALUE)
    {
        adc = ADC_MAX_VALUE;
    }

    pwm = min + (int32_t)((uint32_t)adc * (max - min) / ADC_MAX_VALUE);

    if (reverse)
    {
        pwm = min + max - pwm;
    }

    return PWM_Limit(pwm, min, max);
}

/**
  * @brief  启动五个关节舵机的 PWM 输出
  * @note   夹爪舵机使用 TIM4_CH2，由 robot_gripper 模块单独控制
  */
void Servo_PWM_Start_All(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

/**
  * @brief  将五个关节舵机设置到机械中位
  */
void Servo_All_Mid(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO1_MID);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO2_MID);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, SERVO3_MID);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, SERVO4_MID);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO5_MID);
}

/**
  * @brief  根据五路 ADC 采样值实时更新五个关节舵机位置
  * @param  adc_values 五路 ADC 采样值数组
  */
// void Servo_Update_From_ADC(uint16_t adc_values[5])
// {
//     __HAL_TIM_SET_COMPARE(
//         &htim3,
//         TIM_CHANNEL_1,
//         ADC_To_PWM(adc_values[0], SERVO_MIN, SERVO_MAX, 1)
//     );

//     __HAL_TIM_SET_COMPARE(
//         &htim3,
//         TIM_CHANNEL_2,
//         ADC_To_PWM(adc_values[1], SERVO_MIN, SERVO_MAX, 1)
//     );

//     __HAL_TIM_SET_COMPARE(
//         &htim3,
//         TIM_CHANNEL_3,
//         ADC_To_PWM(adc_values[2], SERVO_MIN, SERVO_MAX, 1)
//     );

//     __HAL_TIM_SET_COMPARE(
//         &htim3,
//         TIM_CHANNEL_4,
//         ADC_To_PWM(adc_values[3], SERVO_MIN, SERVO_MAX, 1)
//     );

//     __HAL_TIM_SET_COMPARE(
//         &htim4,
//         TIM_CHANNEL_1,
//         ADC_To_PWM(adc_values[4], SERVO_MIN, SERVO_MAX, 1)
//     );
// }

/**
  * @brief  同步设置五个关节舵机的 PWM 值
  * @note   主要用于动作回放过程中的插值输出
  */
void Servo_Set_All_PWM(uint16_t pwm1,
                       uint16_t pwm2,
                       uint16_t pwm3,
                       uint16_t pwm4,
                       uint16_t pwm5)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm4);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm5);
}

static uint16_t Servo_Apply_Deadband(uint16_t new_pwm, uint16_t old_pwm)
{
    if (new_pwm > old_pwm)
    {
        if ((new_pwm - old_pwm) < SERVO_PWM_DEADBAND)
        {
            return old_pwm;
        }
    }
    else
    {
        if ((old_pwm - new_pwm) < SERVO_PWM_DEADBAND)
        {
            return old_pwm;
        }
    }

    return new_pwm;
}

void Servo_Update_From_ADC(uint16_t adc_values[5])
{
    static uint16_t last_pwm1 = SERVO1_MID;
    static uint16_t last_pwm2 = SERVO2_MID;
    static uint16_t last_pwm3 = SERVO3_MID;
    static uint16_t last_pwm4 = SERVO4_MID;
    static uint16_t last_pwm5 = SERVO5_MID;

    uint16_t pwm1;
    uint16_t pwm2;
    uint16_t pwm3;
    uint16_t pwm4;
    uint16_t pwm5;

    pwm1 = ADC_To_PWM(adc_values[0], SERVO_MIN, SERVO_MAX, 1);
    pwm2 = ADC_To_PWM(adc_values[1], SERVO_MIN, SERVO_MAX, 1);
    pwm3 = ADC_To_PWM(adc_values[2], SERVO_MIN, SERVO_MAX, 1);
    pwm4 = ADC_To_PWM(adc_values[3], SERVO_MIN, SERVO_MAX, 1);
    pwm5 = ADC_To_PWM(adc_values[4], SERVO_MIN, SERVO_MAX, 1);

    pwm1 = Servo_Apply_Deadband(pwm1, last_pwm1);
    pwm2 = Servo_Apply_Deadband(pwm2, last_pwm2);
    pwm3 = Servo_Apply_Deadband(pwm3, last_pwm3);
    pwm4 = Servo_Apply_Deadband(pwm4, last_pwm4);
    pwm5 = Servo_Apply_Deadband(pwm5, last_pwm5);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm4);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm5);

    last_pwm1 = pwm1;
    last_pwm2 = pwm2;
    last_pwm3 = pwm3;
    last_pwm4 = pwm4;
    last_pwm5 = pwm5;
}
