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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * ADC 阈值：低于此值 = 感光管检测到黑线
 * 实际电路：白线反射红外 → 光电管导通 → 高 ADC 值；黑线吸收红外 → 截止 → 低 ADC 值
 */
#define BLACK_THRESHOLD  2000

/* 电机 PWM 速度（0 ~ 1000，对应 TIM1 Period = 1000） */
#define SPEED_BASE   600    /* 直行速度                          */
#define SPEED_TRIM    80    /* 轻微偏差修正幅度（base ± TRIM）    */
#define SPEED_FAST   750    /* 中度转弯外侧轮速度                 */
#define SPEED_SLOW   200    /* 中度转弯内侧轮速度                 */
#define SPEED_SPIN   450    /* 急转弯反转速度                     */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
 * DMA 持续将 ADC1 四通道结果写入此缓冲区（12 位，0~4095）：
 *   adc_buf[0]  PA0  → ADC1_IN1   最左侧传感器
 *   adc_buf[1]  PA1  → ADC1_IN2   左中传感器
 *   adc_buf[2]  PA3  → ADC1_IN4   右中传感器
 *   adc_buf[3]  PB0  → ADC1_IN11  最右侧传感器
 */
volatile uint16_t adc_buf[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void Motor_Drive(int16_t leftPWM, int16_t rightPWM);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * 驱动两个电机。
 * leftPWM / rightPWM：-1000（全速后退）~ +1000（全速前进），0 = 停止。
 *
 * 引脚对应关系（见 main.h / gpio.c）：
 *   左轮  Motor A：AIN1 = PF0，AIN2 = PF1，PWM = PA8（TIM1_CH1）
 *   右轮  Motor B：BIN1 = PB1，BIN2 = PA10，PWM = PA9（TIM1_CH2）
 *
 * 若小车运动方向与预期相反，交换 leftPWM / rightPWM 参数，
 * 或将对应电机的 xIN1 / xIN2 逻辑对调即可。
 */
static void Motor_Drive(int16_t leftPWM, int16_t rightPWM)
{
    /* 限幅 */
    if (leftPWM  >  1000) leftPWM  =  1000;
    if (leftPWM  < -1000) leftPWM  = -1000;
    if (rightPWM >  1000) rightPWM =  1000;
    if (rightPWM < -1000) rightPWM = -1000;

    /* 左轮（Motor A） */
    if (leftPWM > 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else if (leftPWM < 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        leftPWM = -leftPWM;
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }

    /* 右轮（Motor B） */
    if (rightPWM > 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    } else if (rightPWM < 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        rightPWM = -rightPWM;
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)leftPWM);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)rightPWM);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* 校准必须在 DMA 启动之前完成 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 4);

  /* 启动 PWM 输出通道 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*
     * 读取四路传感器并二值化：1 = 检测到黑线，0 = 白色地面
     *
     * 传感器排列（从左到右）：
     *   sL   sML   sMR   sR
     *   PA0  PA1   PA3   PB0
     */
    uint8_t sL  = (adc_buf[0] < BLACK_THRESHOLD) ? 1 : 0;
    uint8_t sML = (adc_buf[1] < BLACK_THRESHOLD) ? 1 : 0;
    uint8_t sMR = (adc_buf[2] < BLACK_THRESHOLD) ? 1 : 0;
    uint8_t sR  = (adc_buf[3] < BLACK_THRESHOLD) ? 1 : 0;

    /* 将四路状态编码为 4 位值 [sL sML sMR sR] */
    uint8_t st = (sL << 3) | (sML << 2) | (sMR << 1) | sR;

    switch (st)
    {
      /* ── 直行：中间两路都压线 ── */
      case 0x06: /* 0110 sML+sMR         */
      case 0x07: /* 0111 sML+sMR+sR      */
      case 0x0E: /* 1110 sL+sML+sMR      */
      case 0x0F: /* 1111 全部             */
        Motor_Drive(SPEED_BASE, SPEED_BASE);
        break;

      /* ── 轻微右偏（仅 sMR 压线）：小幅左转修正 ── */
      case 0x02: /* 0010 */
        Motor_Drive(SPEED_BASE + SPEED_TRIM, SPEED_BASE - SPEED_TRIM);
        break;

      /* ── 明显右偏（sMR + sR 压线）：中度左转 ── */
      case 0x03: /* 0011 */
        Motor_Drive(SPEED_FAST, SPEED_SLOW);
        break;

      /* ── 严重右偏（仅 sR 压线）：急左转 ── */
      case 0x01: /* 0001 */
        Motor_Drive(SPEED_FAST, -SPEED_SPIN);
        break;

      /* ── 轻微左偏（仅 sML 压线）：小幅右转修正 ── */
      case 0x04: /* 0100 */
        Motor_Drive(SPEED_BASE - SPEED_TRIM, SPEED_BASE + SPEED_TRIM);
        break;

      /* ── 明显左偏（sL + sML 压线）：中度右转 ── */
      case 0x0C: /* 1100 */
        Motor_Drive(SPEED_SLOW, SPEED_FAST);
        break;

      /* ── 严重左偏（仅 sL 压线）：急右转 ── */
      case 0x08: /* 1000 */
        Motor_Drive(-SPEED_SPIN, SPEED_FAST);
        break;

      /* ── T 形路口 / 终点（两侧触线）：直行通过 ── */
      case 0x09: /* 1001 */
        Motor_Drive(SPEED_BASE, SPEED_BASE);
        break;

      /* ── 丢线：低速直行等待重新找线 ── */
      case 0x00:
      default:
        Motor_Drive(SPEED_BASE / 2, SPEED_BASE / 2);
        break;
    }

    HAL_Delay(5);

    /* USER CODE END 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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
#ifdef USE_FULL_ASSERT
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
