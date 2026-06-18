#include "robot_uart.h"
#include "usart.h"
#include "robot_servo.h"
#include "robot_gripper.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define UART_RX_BUF_LEN  64
#define UART_ACK_J_ENABLE  0

volatile Robot_Mode_t robot_mode = ROBOT_MODE_MANUAL;

static uint8_t uart_rx_byte = 0;

static char uart_rx_buf[UART_RX_BUF_LEN];
static char uart_cmd_buf[UART_RX_BUF_LEN];

static volatile uint8_t uart_rx_index = 0;
static volatile uint8_t uart_cmd_ready = 0;

static uint16_t pc_pwm[5] = {150, 150, 150, 150, 150};

volatile uint8_t uart_req_record = 0;
volatile uint8_t uart_req_play   = 0;
volatile uint8_t uart_req_clear  = 0;

static uint16_t UART_Limit_PWM(int value)
{
    if (value < SERVO_MIN)
    {
        return SERVO_MIN;
    }

    if (value > SERVO_MAX)
    {
        return SERVO_MAX;
    }

    return (uint16_t)value;
}

static void UART_Send_String(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

void Robot_UART_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

    UART_Send_String("Robot UART Ready\r\n");
}

static void Robot_UART_Parse_Command(char *cmd, volatile uint8_t *gripper_state)
{
    int p1, p2, p3, p4, p5;
    int grip;

    if (strcmp(cmd, "#MODE,PC!") == 0)
    {
        robot_mode = ROBOT_MODE_PC;
        UART_Send_String("OK:MODE PC\r\n");
    }
    else if (strcmp(cmd, "#MODE,MANUAL!") == 0)
    {
        robot_mode = ROBOT_MODE_MANUAL;
        UART_Send_String("OK:MODE MANUAL\r\n");
    }
    else if (sscanf(cmd, "#J,%d,%d,%d,%d,%d!", &p1, &p2, &p3, &p4, &p5) == 5)
    {
        if (robot_mode != ROBOT_MODE_PC)
        {
            UART_Send_String("ERR:NOT PC MODE\r\n");
            return;
        }

        pc_pwm[0] = UART_Limit_PWM(p1);
        pc_pwm[1] = UART_Limit_PWM(p2);
        pc_pwm[2] = UART_Limit_PWM(p3);
        pc_pwm[3] = UART_Limit_PWM(p4);
        pc_pwm[4] = UART_Limit_PWM(p5);

        Servo_Set_All_PWM(pc_pwm[0],
                          pc_pwm[1],
                          pc_pwm[2],
                          pc_pwm[3],
                          pc_pwm[4]);

        #if UART_ACK_J_ENABLE
        UART_Send_String("OK:J\r\n");
        #endif
    }
    else if (sscanf(cmd, "#G,%d!", &grip) == 1)
    {
        if (grip == 0)
        {
            *gripper_state = 0;
            Gripper_Open();
            UART_Send_String("OK:GRIP OPEN\r\n");
        }
        else if (grip == 1)
        {
            *gripper_state = 1;
            Gripper_Close();
            UART_Send_String("OK:GRIP CLOSE\r\n");
        }
        else
        {
            UART_Send_String("ERR:GRIP VALUE\r\n");
        }
    }
    else if (strcmp(cmd, "#MID!") == 0)
    {
        pc_pwm[0] = 150;
        pc_pwm[1] = 150;
        pc_pwm[2] = 150;
        pc_pwm[3] = 150;
        pc_pwm[4] = 150;

        Servo_Set_All_PWM(150, 150, 150, 150, 150);

        UART_Send_String("OK:MID\r\n");
    }
    else if (strcmp(cmd, "#REC!") == 0)
    {
        uart_req_record = 1;
        UART_Send_String("OK:REC\r\n");
    }
    else if (strcmp(cmd, "#PLAY!") == 0)
    {
        uart_req_play = 1;
        UART_Send_String("OK:PLAY\r\n");
    }
    else if (strcmp(cmd, "#CLR!") == 0)
    {
        uart_req_clear = 1;
        UART_Send_String("OK:CLR\r\n");
    }
    else
    {
        UART_Send_String("ERR:UNKNOWN CMD\r\n");
    }
}

void Robot_UART_Task(volatile uint8_t *gripper_state)
{
    char cmd[UART_RX_BUF_LEN];

    if (uart_cmd_ready == 0)
    {
        return;
    }

    __disable_irq();
    strcpy(cmd, uart_cmd_buf);
    uart_cmd_ready = 0;
    __enable_irq();

    Robot_UART_Parse_Command(cmd, gripper_state);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;

    if (huart->Instance == USART1)
    {
        if (uart_cmd_ready == 0)
        {
            if (uart_rx_byte == '#')
            {
                uart_rx_index = 0;
                uart_rx_buf[uart_rx_index++] = uart_rx_byte;
            }
            else if (uart_rx_index > 0)
            {
                if (uart_rx_byte == '!')
                {
                    uart_rx_buf[uart_rx_index++] = uart_rx_byte;
                    uart_rx_buf[uart_rx_index] = '\0';

                    for (i = 0; i < uart_rx_index + 1; i++)
                    {
                        uart_cmd_buf[i] = uart_rx_buf[i];
                    }

                    uart_cmd_ready = 1;
                    uart_rx_index = 0;
                }
                else
                {
                    if (uart_rx_index < UART_RX_BUF_LEN - 2)
                    {
                        uart_rx_buf[uart_rx_index++] = uart_rx_byte;
                    }
                    else
                    {
                        uart_rx_index = 0;
                    }
                }
            }
        }

        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}
