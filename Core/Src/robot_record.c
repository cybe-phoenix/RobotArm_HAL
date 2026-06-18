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

/* 动作点 PWM 数据缓存，每个动作点保存五个关节的 PWM 值 */
uint16_t record_pwm[MAX_RECORD_LENGTH][5] = {0};
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
void Record_Current_Point(uint16_t pwm_values[5], uint8_t gripper_state)
{
    if (record_count >= MAX_RECORD_LENGTH)
    {
        return;
    }

    record_pwm[record_count][0] = pwm_values[0];
    record_pwm[record_count][1] = pwm_values[1];
    record_pwm[record_count][2] = pwm_values[2];
    record_pwm[record_count][3] = pwm_values[3];
    record_pwm[record_count][4] = pwm_values[4];

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

    int32_t pwm1;
    int32_t pwm2;
    int32_t pwm3;
    int32_t pwm4;
    int32_t pwm5;

    if (record_count < 2)
    {
        return;
    }

    for (i = 0; i < record_count - 1; i++)
    {
        for (step = 0; step <= PLAYBACK_STEPS; step++)
        {
            pwm1 = record_pwm[i][0] + 
                   (int32_t)(record_pwm[i + 1][0] - record_pwm[i][0]) * step / PLAYBACK_STEPS;

            pwm2 = record_pwm[i][1] + 
                   (int32_t)(record_pwm[i + 1][1] - record_pwm[i][1]) * step / PLAYBACK_STEPS;

            pwm3 = record_pwm[i][2] + 
                   (int32_t)(record_pwm[i + 1][2] - record_pwm[i][2]) * step / PLAYBACK_STEPS;

            pwm4 = record_pwm[i][3] + 
                   (int32_t)(record_pwm[i + 1][3] - record_pwm[i][3]) * step / PLAYBACK_STEPS;

            pwm5 = record_pwm[i][4] + 
                   (int32_t)(record_pwm[i + 1][4] - record_pwm[i][4]) * step / PLAYBACK_STEPS;

            Servo_Set_All_PWM((uint16_t)pwm1,
                              (uint16_t)pwm2,
                              (uint16_t)pwm3,
                              (uint16_t)pwm4,
                              (uint16_t)pwm5);

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
