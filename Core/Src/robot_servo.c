#include "robot_servo.h"
#include "tim.h"

uint16_t PWM_Limit(int32_t pwm, uint16_t min, uint16_t max)
{
    if (pwm < min) pwm = min;
    if (pwm > max) pwm = max;
    return (uint16_t)pwm;
}

uint16_t ADC_To_PWM(uint16_t adc, uint16_t min, uint16_t max, uint8_t reverse)
{
    int32_t pwm;

    if (adc > ADC_MAX_VALUE)
    {
        adc = ADC_MAX_VALUE;
    }

    pwm = min + (int32_t)((uint32_t)adc * (max - min) / ADC_MAX_VALUE);

    if (reverse)
    {
        pwm = min + max - pwm;
    }

    return PWM_Limit(pwm, min, max);
}

void Servo_PWM_Start_All(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void Servo_All_Mid(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO1_MID);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO2_MID);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, SERVO3_MID);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, SERVO4_MID);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO5_MID);
}

void Servo_Update_From_ADC(uint16_t adc_values[5])
{
    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_1,
        ADC_To_PWM(adc_values[0], SERVO_MIN, SERVO_MAX, 1)
    );

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_2,
        ADC_To_PWM(adc_values[1], SERVO_MIN, SERVO_MAX, 1)
    );

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_3,
        ADC_To_PWM(adc_values[2], SERVO_MIN, SERVO_MAX, 1)
    );

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_4,
        ADC_To_PWM(adc_values[3], SERVO_MIN, SERVO_MAX, 1)
    );

    __HAL_TIM_SET_COMPARE(
        &htim4,
        TIM_CHANNEL_1,
        ADC_To_PWM(adc_values[4], SERVO_MIN, SERVO_MAX, 1)
    );
}

void Servo_Set_All_PWM(uint16_t pwm1,
                       uint16_t pwm2,
                       uint16_t pwm3,
                       uint16_t pwm4,
                       uint16_t pwm5)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm4);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm5);
}
