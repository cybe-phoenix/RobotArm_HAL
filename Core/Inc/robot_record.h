#ifndef __ROBOT_RECORD_H
#define __ROBOT_RECORD_H

#include "main.h"

#define MAX_RECORD_LENGTH 100

extern volatile uint16_t record_count;

void Record_Current_Point(uint16_t adc_values[5], uint8_t gripper_state);
void Record_Clear(void);
void Playback_Record(void);

#endif
