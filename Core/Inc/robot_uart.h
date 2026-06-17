#ifndef __ROBOT_UART_H
#define __ROBOT_UART_H

#include "main.h"

typedef enum
{
    ROBOT_MODE_MANUAL = 0,
    ROBOT_MODE_PC     = 1
} Robot_Mode_t;

extern volatile Robot_Mode_t robot_mode;

void Robot_UART_Init(void);
void Robot_UART_Task(uint8_t *gripper_state);

#endif
