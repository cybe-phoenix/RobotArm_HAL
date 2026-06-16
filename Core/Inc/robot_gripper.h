#ifndef __ROBOT_GRIPPER_H
#define __ROBOT_GRIPPER_H

#include "main.h"

/* 夹爪 PWM 安全范围 */
#define GRIPPER_PWM_MIN    50
#define GRIPPER_PWM_MAX    140

/* 夹爪开合位置 */
#define GRIPPER_OPEN_PWM   140
#define GRIPPER_CLOSE_PWM  50

void Gripper_Open(void);
void Gripper_Close(void);

#endif
