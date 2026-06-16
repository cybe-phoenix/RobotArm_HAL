#ifndef __ROBOT_KEY_H
#define __ROBOT_KEY_H

#include "main.h"

/* 无按键按下 */
#define KEY_NONE  0

/* 四个独立按键编号 */
#define KEY1_NUM  1   // PB12：清空记录
#define KEY2_NUM  2   // PB13：回放
#define KEY3_NUM  3   // PB14：夹爪开合
#define KEY4_NUM  4   // PB15：记录动作点

/**
  * @brief  扫描按键并返回按键编号
  * @note   按键采用低电平有效，松手后返回一次有效键值
  * @retval KEY_NONE / KEY1_NUM / KEY2_NUM / KEY3_NUM / KEY4_NUM
  */
uint8_t Key_GetNum(void);

#endif
