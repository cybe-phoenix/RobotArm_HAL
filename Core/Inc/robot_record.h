#ifndef __ROBOT_RECORD_H
#define __ROBOT_RECORD_H

#include "main.h"

/* 最大可记录动作点数量 */
#define MAX_RECORD_LENGTH 100

/* 当前已记录的动作点数量，供 main.c 和 Watch 调试观察 */
extern volatile uint16_t record_count;

/**
  * @brief  记录当前机械臂动作点
  * @param  adc_values 当前五路 ADC 采样值
  * @param  gripper_state 当前夹爪状态，0 表示打开，1 表示闭合
  */
void Record_Current_Point(uint16_t adc_values[5], uint8_t gripper_state);

/**
  * @brief  清空已记录的动作点
  */
void Record_Clear(void);

/**
  * @brief  回放已记录的动作序列
  * @note   通过线性插值实现相邻动作点之间的平滑过渡
  */
void Playback_Record(void);

#endif
