#ifndef __ROBOT_KEY_H
#define __ROBOT_KEY_H

#include "main.h"

/* 按键返回值 */
#define KEY_NONE  0
#define KEY1_NUM  1   // PB12：清空记录
#define KEY2_NUM  2   // PB13：回放
#define KEY3_NUM  3   // PB14：夹爪开合
#define KEY4_NUM  4   // PB15：记录动作点

uint8_t Key_GetNum(void);

#endif
