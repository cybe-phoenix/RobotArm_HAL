#ifndef __ROBOT_SERVO_H
#define __ROBOT_SERVO_H

#include "main.h"

/* 舵机中位 */
#define SERVO1_MID 150   // PA6  -> TIM3_CH1
#define SERVO2_MID 150   // PA7  -> TIM3_CH2
#define SERVO3_MID 150   // PB0  -> TIM3_CH3
#define SERVO4_MID 150   // PB1  -> TIM3_CH4
#define SERVO5_MID 150   // PB6  -> TIM4_CH1

/* ADC 最大值 */
#define ADC_MAX_VALUE 4095

/* 舵机安全 PWM 范围 */
#define SERVO_MIN 80
#define SERVO_MAX 220

uint16_t PWM_Limit(int32_t pwm, uint16_t min, uint16_t max);
uint16_t ADC_To_PWM(uint16_t adc, uint16_t min, uint16_t max, uint8_t reverse);

void Servo_PWM_Start_All(void);
void Servo_All_Mid(void);

void Servo_Update_From_ADC(uint16_t adc_values[5]);

void Servo_Set_All_PWM(uint16_t pwm1,
                       uint16_t pwm2,
                       uint16_t pwm3,
                       uint16_t pwm4,
                       uint16_t pwm5);

#endif
