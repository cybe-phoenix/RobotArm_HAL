#include "robot_record.h"
#include "robot_servo.h"
#include "robot_gripper.h"

uint16_t record_adc[MAX_RECORD_LENGTH][5] = {0};
uint8_t record_gripper[MAX_RECORD_LENGTH] = {0};

volatile uint16_t record_count = 0;

#define PLAYBACK_STEPS      100
#define PLAYBACK_DELAY_MS   15

void Record_Current_Point(uint16_t adc_values[5], uint8_t gripper_state)
{
    if (record_count >= MAX_RECORD_LENGTH)
    {
        return;
    }

    record_adc[record_count][0] = adc_values[0];
    record_adc[record_count][1] = adc_values[1];
    record_adc[record_count][2] = adc_values[2];
    record_adc[record_count][3] = adc_values[3];
    record_adc[record_count][4] = adc_values[4];

    record_gripper[record_count] = gripper_state;

    record_count++;
}

void Record_Clear(void)
{
    record_count = 0;
}

void Playback_Record(void)
{
    uint16_t i;
    uint16_t step;

    if (record_count < 2)
    {
        return;
    }

    for (i = 0; i < record_count - 1; i++)
    {
        uint16_t s1 = ADC_To_PWM(record_adc[i][0],     SERVO_MIN, SERVO_MAX, 1);
        uint16_t s2 = ADC_To_PWM(record_adc[i][1],     SERVO_MIN, SERVO_MAX, 1);
        uint16_t s3 = ADC_To_PWM(record_adc[i][2],     SERVO_MIN, SERVO_MAX, 1);
        uint16_t s4 = ADC_To_PWM(record_adc[i][3],     SERVO_MIN, SERVO_MAX, 1);
        uint16_t s5 = ADC_To_PWM(record_adc[i][4],     SERVO_MIN, SERVO_MAX, 1);

        uint16_t e1 = ADC_To_PWM(record_adc[i + 1][0], SERVO_MIN, SERVO_MAX, 1);
        uint16_t e2 = ADC_To_PWM(record_adc[i + 1][1], SERVO_MIN, SERVO_MAX, 1);
        uint16_t e3 = ADC_To_PWM(record_adc[i + 1][2], SERVO_MIN, SERVO_MAX, 1);
        uint16_t e4 = ADC_To_PWM(record_adc[i + 1][3], SERVO_MIN, SERVO_MAX, 1);
        uint16_t e5 = ADC_To_PWM(record_adc[i + 1][4], SERVO_MIN, SERVO_MAX, 1);

        for (step = 0; step <= PLAYBACK_STEPS; step++)
        {
            uint16_t p1 = s1 + (int32_t)(e1 - s1) * step / PLAYBACK_STEPS;
            uint16_t p2 = s2 + (int32_t)(e2 - s2) * step / PLAYBACK_STEPS;
            uint16_t p3 = s3 + (int32_t)(e3 - s3) * step / PLAYBACK_STEPS;
            uint16_t p4 = s4 + (int32_t)(e4 - s4) * step / PLAYBACK_STEPS;
            uint16_t p5 = s5 + (int32_t)(e5 - s5) * step / PLAYBACK_STEPS;

            Servo_Set_All_PWM(p1, p2, p3, p4, p5);

            HAL_Delay(PLAYBACK_DELAY_MS);
        }

        if (record_gripper[i + 1] == 0)
        {
            Gripper_Open();
        }
        else
        {
            Gripper_Close();
        }
    }
}
