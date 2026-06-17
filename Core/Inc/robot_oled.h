#ifndef __ROBOT_OLED_H
#define __ROBOT_OLED_H

#include "main.h"

void Robot_OLED_Init(void);

void Robot_OLED_Update(uint8_t key,
                       uint8_t gripper_state,
                       uint8_t playback_busy,
                       uint16_t record_count);

void Robot_OLED_Show_Joints_Status(uint16_t joint_pwm[5],
                                   uint8_t gripper_state,
                                   uint8_t playback_busy,
                                   uint16_t record_count,
                                   uint8_t robot_mode);
                                   
#endif
