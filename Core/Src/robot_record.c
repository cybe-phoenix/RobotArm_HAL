/**
  ******************************************************************************
  * @file    robot_record.c
  * @brief   六自由度机械臂动作记录与回放模块源文件
  *
  * @details
  *          本文件实现机械臂示教动作的记录与回放功能。
  *          每个动作点包含五路关节电位器 ADC 值和一个夹爪状态。
  *          回放时，相邻动作点之间采用线性插值方式生成平滑轨迹，
  *          并在每段动作结束后恢复对应记录点的夹爪状态。
  *
  ******************************************************************************
  */

#include "robot_record.h"
#include "robot_servo.h"
#include "robot_gripper.h"

/* 动作点 ADC 数据缓存，每个动作点保存五个关节的 ADC 采样值 */
uint16_t record_adc[MAX_RECORD_LENGTH][5] = {0};
/* 动作点夹爪状态缓存，0 表示打开，1 表示闭合 */
uint8_t record_gripper[MAX_RECORD_LENGTH] = {0};

/* 当前已记录的动作点数量 */
volatile uint16_t record_count = 0;

#define PLAYBACK_STEPS      100
#define PLAYBACK_DELAY_MS   15

/**
  * @brief  记录当前机械臂动作点
  * @param  adc_values 当前五路 ADC 采样值
  * @param  gripper_state 当前夹爪状态，0 表示打开，1 表示闭合
  * @note   当记录数量达到 MAX_RECORD_LENGTH 后，不再继续记录
  */
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

/**
  * @brief  清空已记录的动作序列
  */
void Record_Clear(void)
{
    record_count = 0;
}

/**
  * @brief  回放已记录的动作序列
  * @note   当记录点数量小于 2 时不执行回放。
  *         每两个相邻记录点之间采用线性插值，
  *         从而使机械臂动作更加平滑。
  */
void Playback_Record(void)
{
    uint16_t i;
    uint16_t step;

    /* 至少需要两个动作点才能形成一段运动轨迹 */
    if (record_count < 2)
    {
        return;
    }

    /* 依次回放相邻动作点：点0->点1，点1->点2，依此类推 */
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
