/**
  ******************************************************************************
  * @file    robot_gripper.c
  * @brief   六自由度机械臂夹爪控制模块源文件
  *
  * @details
  *          本文件实现夹爪舵机的打开与闭合控制。
  *          夹爪舵机使用 TIM4_CH2 输出 PWM。
  *          为避免夹爪机械结构顶死，程序对 PWM 输出进行了限幅处理。
  *
  ******************************************************************************
  */

#include "robot_gripper.h"
#include "tim.h"

/**
  * @brief  限制夹爪 PWM 值在安全范围内
  * @param  pwm 原始夹爪 PWM 值
  * @retval 限幅后的夹爪 PWM 值
  */
static uint16_t Gripper_Limit(uint16_t pwm)
{
    if (pwm < GRIPPER_PWM_MIN) pwm = GRIPPER_PWM_MIN;
    if (pwm > GRIPPER_PWM_MAX) pwm = GRIPPER_PWM_MAX;
    return pwm;
}

/**
  * @brief  输出指定 PWM 控制夹爪动作
  * @param  pwm 夹爪目标 PWM 值
  * @note   输出 1 秒后停止 TIM4_CH2 PWM，避免夹爪舵机持续受力发热
  */
static void Gripper_Test_PWM(uint16_t pwm)
{
    pwm = Gripper_Limit(pwm);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    HAL_Delay(1000);

    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
}

/**
  * @brief  打开夹爪
  */
void Gripper_Open(void)
{
    Gripper_Test_PWM(GRIPPER_OPEN_PWM);
}

/**
  * @brief  关闭夹爪
  */
void Gripper_Close(void)
{
    Gripper_Test_PWM(GRIPPER_CLOSE_PWM);
}
