/**
  ******************************************************************************
  * @file    robot_key.c
  * @brief   六自由度机械臂按键扫描模块源文件
  *
  * @details
  *          本文件实现 PB12~PB15 四个独立按键的扫描功能。
  *          按键采用低电平有效方式，按下后进行软件消抖，
  *          松开按键后返回一次有效键值。
  *
  ******************************************************************************
  */

#include "robot_key.h"
#include "gpio.h"

/**
  * @brief  扫描四个独立按键并返回按键编号
  * @note   按键低电平有效。
  *         KEY1：PB12，用于清空记录；
  *         KEY2：PB13，用于回放动作；
  *         KEY3：PB14，用于夹爪开合；
  *         KEY4：PB15，用于记录动作点。
  * @retval KEY_NONE / KEY1_NUM / KEY2_NUM / KEY3_NUM / KEY4_NUM
  */
uint8_t Key_GetNum(void)
{
    uint8_t key_num = KEY_NONE;

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
        {
            key_num = KEY1_NUM;
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET);
        }
    }

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
        {
            key_num = KEY2_NUM;
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET);
            HAL_Delay(300);   // KEY2 释放后消抖，防止一次按键触发两次
        }
    }

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
        {
            key_num = KEY3_NUM;
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET);
        }
    }

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET)
        {
            key_num = KEY4_NUM;
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET);
        }
    }

    return key_num;
}
