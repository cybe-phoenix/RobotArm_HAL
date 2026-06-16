#include "robot_gripper.h"
#include "tim.h"

static uint16_t Gripper_Limit(uint16_t pwm)
{
    if (pwm < GRIPPER_PWM_MIN) pwm = GRIPPER_PWM_MIN;
    if (pwm > GRIPPER_PWM_MAX) pwm = GRIPPER_PWM_MAX;
    return pwm;
}

static void Gripper_Test_PWM(uint16_t pwm)
{
    pwm = Gripper_Limit(pwm);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    HAL_Delay(1000);

    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
}

void Gripper_Open(void)
{
    Gripper_Test_PWM(GRIPPER_OPEN_PWM);
}

void Gripper_Close(void)
{
    Gripper_Test_PWM(GRIPPER_CLOSE_PWM);
}
