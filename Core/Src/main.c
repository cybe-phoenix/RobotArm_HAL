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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "robot_gripper.h"
#include "robot_key.h"
#include "robot_oled_font.h"
#include "robot_oled.h"
#include "robot_record.h"
#include "robot_servo.h"
#include "robot_uart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 调试模式
// 0 = 关闭
// 1 = 打开
#define ROBOT_DEBUG_ENABLE  1
// #define ROBOT_DEBUG_ENABLE  0

#define OLED_JOINT_REFRESH_MS  300

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t adc_values[5] = {0};

volatile uint8_t gripper_state = 0;  // 0 = 打开�??1 = 闭合

volatile uint8_t key = 0;
volatile uint8_t playback_busy = 0;

#if ROBOT_DEBUG_ENABLE

volatile uint16_t adc_debug0 = 0;
volatile uint16_t adc_debug1 = 0;
volatile uint16_t adc_debug2 = 0;
volatile uint16_t adc_debug3 = 0;
volatile uint16_t adc_debug4 = 0;

volatile uint8_t key_last = 0;
volatile uint32_t key_count = 0;
volatile uint32_t playback_count = 0;

volatile uint8_t pb12_state = 1;
volatile uint8_t pb13_state = 1;
volatile uint8_t pb14_state = 1;
volatile uint8_t pb15_state = 1;

#endif

volatile uint32_t oled_last_tick = 0;

uint16_t oled_joint_pwm[5] = {150, 150, 150, 150, 150};
uint32_t oled_joint_last_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  Servo_All_Mid();
  Servo_PWM_Start_All();
  HAL_Delay(1000);

  // 夹爪测试已完成，ADC 测试阶段先不自动�???合夹�???
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

  /* OLED 初始化并显示初始界面 */
  Robot_OLED_Init();
  Servo_Get_Current_PWM(oled_joint_pwm);
  Robot_OLED_Show_Joints_Status(oled_joint_pwm,
                              gripper_state,
                              playback_busy,
                              record_count,
                              robot_mode);

  Robot_UART_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Robot_UART_Task(&gripper_state);

    key = Key_GetNum();

    #if ROBOT_DEBUG_ENABLE

    // 读取按键原始状态
    pb12_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    pb13_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
    pb14_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
    pb15_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);

    if (key != KEY_NONE)
    {
        key_last = key;
        key_count++;
    }

    // 更新 ADC 调试�??
    adc_debug0 = adc_values[0];
    adc_debug1 = adc_values[1];
    adc_debug2 = adc_values[2];
    adc_debug3 = adc_values[3];
    adc_debug4 = adc_values[4];

    #endif

    switch (key)
    {
      case KEY1_NUM:
          Record_Clear();
          break;

      case KEY2_NUM:
          #if ROBOT_DEBUG_ENABLE
          playback_count++;
          #endif

          playback_busy = 1;
          Robot_OLED_Show_Joints_Status(oled_joint_pwm,
                                  gripper_state,
                                  playback_busy,
                                  record_count,
                                  robot_mode);

          Playback_Record();

          HAL_Delay(1000);

          playback_busy = 0;
          Servo_Get_Current_PWM(oled_joint_pwm);
          Robot_OLED_Show_Joints_Status(oled_joint_pwm,
                                  gripper_state,
                                  playback_busy,
                                  record_count,
                                  robot_mode);
          break;

      case KEY3_NUM:
          gripper_state = !gripper_state;

          if (gripper_state == 0)
          {
              Gripper_Open();
          }
          else
          {
              Gripper_Close();
          }
          break;

      case KEY4_NUM:
          Record_Current_Point(adc_values, gripper_state);
          break;

      default:
          break;
    }

    if (playback_busy == 0 && robot_mode == ROBOT_MODE_MANUAL)
    {
        Servo_Update_From_ADC(adc_values);
    }

    if (HAL_GetTick() - oled_joint_last_tick >= OLED_JOINT_REFRESH_MS)
    {
      oled_joint_last_tick = HAL_GetTick();

      Servo_Get_Current_PWM(oled_joint_pwm);
      Robot_OLED_Show_Joints_Status(oled_joint_pwm,
                              gripper_state,
                              playback_busy,
                              record_count,
                              robot_mode);
    }

    HAL_Delay(10);

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

