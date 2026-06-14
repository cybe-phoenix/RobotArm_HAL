/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t adc_values[5] = {0};

volatile uint16_t adc_debug0 = 0;
volatile uint16_t adc_debug1 = 0;
volatile uint16_t adc_debug2 = 0;
volatile uint16_t adc_debug3 = 0;
volatile uint16_t adc_debug4 = 0;

volatile uint8_t gripper_state = 0;  // 0 = 打开，1 = 闭合

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define SERVO1_MID 150   // PA6  -> TIM3_CH1
#define SERVO2_MID 150   // PA7  -> TIM3_CH2
#define SERVO3_MID 150   // PB0  -> TIM3_CH3
#define SERVO4_MID 190   // PB1  -> TIM3_CH4，需要调�?
#define SERVO5_MID 150   // PB6  -> TIM4_CH1

#define GRIPPER_PWM_MIN    50
#define GRIPPER_PWM_MAX    140

#define GRIPPER_OPEN_PWM   140
#define GRIPPER_CLOSE_PWM  50

#define ADC_MAX_VALUE 4095

#define SERVO1_MIN 130
#define SERVO1_MAX 170

#define SERVO2_MIN 130
#define SERVO2_MAX 170

#define SERVO3_MIN 130
#define SERVO3_MAX 170

#define SERVO4_MIN 170
#define SERVO4_MAX 210

#define SERVO5_MIN 130
#define SERVO5_MAX 170

static uint16_t PWM_Limit(int32_t pwm, uint16_t min, uint16_t max)
{
    if (pwm < min) pwm = min;
    if (pwm > max) pwm = max;
    return (uint16_t)pwm;
}

static uint16_t ADC_To_PWM(uint16_t adc, uint16_t min, uint16_t max, uint8_t reverse)
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

static void Servo_PWM_Start_All(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    // 夹爪先不启动
    // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

static void Servo_All_Mid(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO1_MID);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO2_MID);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, SERVO3_MID);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, SERVO4_MID);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO5_MID);
    // 夹爪先不设置
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, SERVO_PWM_MID);
}

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

static void Gripper_Open(void)
{
    Gripper_Test_PWM(GRIPPER_OPEN_PWM);
}

static void Gripper_Close(void)
{
    Gripper_Test_PWM(GRIPPER_CLOSE_PWM);
}

static void Servo_Update_From_ADC(void)
{
    /*
      0 = 正向
      1 = 反向
    */

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_1,
        ADC_To_PWM(adc_values[0], SERVO1_MIN, SERVO1_MAX, 1)
    );

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_2,
        ADC_To_PWM(adc_values[1], SERVO2_MIN, SERVO2_MAX, 1)
    );

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_3,
        ADC_To_PWM(adc_values[2], SERVO3_MIN, SERVO3_MAX, 1)
    );

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_4,
        ADC_To_PWM(adc_values[3], SERVO4_MIN, SERVO4_MAX, 1)
    );

    __HAL_TIM_SET_COMPARE(
        &htim4,
        TIM_CHANNEL_1,
        ADC_To_PWM(adc_values[4], SERVO5_MIN, SERVO5_MAX, 0)
    );
}

static uint8_t Key_GetNum(void)
{
    uint8_t key_num = 0;

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
        {
            key_num = 1;
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET);
        }
    }

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
        {
            key_num = 2;
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET);
        }
    }

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
        {
            key_num = 3;
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET);
        }
    }

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET)
        {
            key_num = 4;
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET);
        }
    }

    return key_num;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  Servo_All_Mid();
  Servo_PWM_Start_All();
  HAL_Delay(1000);

  // 夹爪测试已完成，ADC 测试阶段先不自动�?合夹�?
  // Gripper_Test_PWM(140);
  // HAL_Delay(1000);

  // Gripper_Test_PWM(50);
  // HAL_Delay(1000);

  // Gripper_Test_PWM(140);
  // HAL_Delay(1000);

  /* ADC 校准 */
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* 启动 ADC + DMA，连续采�? PA0~PA4 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, 5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint8_t key = 0;

    key = Key_GetNum();

    adc_debug0 = adc_values[0];
    adc_debug1 = adc_values[1];
    adc_debug2 = adc_values[2];
    adc_debug3 = adc_values[3];
    adc_debug4 = adc_values[4];

    Servo_Update_From_ADC();

    if (key == 3)
    {
      gripper_state = !gripper_state;

      if (gripper_state == 0)
      {
          Gripper_Open();
      }
      else
      {
          Gripper_Close();
      }
    }

    HAL_Delay(20);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

