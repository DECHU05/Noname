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
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ps2.h"
#include "pca9685.h"
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
PS2Buttons ps2_data; // ??????

// ?????(A=??, B=??, C=??)
float arm_A = 307;
float arm_B = 307;
float arm_C = 307;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void GPIOB_LED_Init(void);
void Motor_SelfTest(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 板载 LED = PC13。
 * CubeMX 的 .ioc 里根本没有 PC13，所以 MX_GPIO_Init() 既没有使能 GPIOC 时钟，
 * 也没有把 PC13 配成输出 —— 于是 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13) 对
 * 一个未初始化的引脚操作，LED 不会亮。这里补上。 */
void GPIOB_LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  /* 板载 LED 低电平点亮 */

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* ==================================================================
 *  电机控制部分（与机械臂无关，改这里不会影响 arm_A/B/C 和 PCA9685）
 *
 *  PWM 满量程 = TIM2/TIM3 的 Period = 999。
 *  原来 pwm_val = speed * 9，speed 最大 100，所以最大只有 900/1000 = 90%，
 *  这就是"轮子力量很弱"的一个原因。这里改成满量程 100%。
 * ================================================================== */
#define MOTOR_PWM_MAX        999    /* 与 tim.c 里 TIM2/TIM3 的 Period 一致 */
#define MOTOR_SPEED_MAX      100    /* Kinematics 输出的速度量纲 ±100 */

/* 单个电机自检开关：改成 1 后上电先逐个通道正反转，用来查硬件。
 * 注意这期间不会响应手柄，测完记得改回 0。 */
#define MOTOR_SELF_TEST      0

int Limit(int speed) {
    if (speed > MOTOR_SPEED_MAX) return MOTOR_SPEED_MAX;
    if (speed < -MOTOR_SPEED_MAX) return -MOTOR_SPEED_MAX;
    return speed;
}

/* 把一个电机接到某个 PWM 通道上：
 *   pwm_ch    : 该电机"正转"对应的通道
 *   pwm_ch_inv: 该电机"反转"对应的通道
 * 正转时 pwm_ch 输出占空比、pwm_ch_inv 输出 0，反之亦然。 */
static void Motor_PWM(TIM_HandleTypeDef *htim, uint32_t pwm_ch, uint32_t pwm_ch_inv,
                      int16_t speed) {
    uint16_t duty;

    speed = (int16_t)Limit(speed);

    /* 满量程换算：|speed| 100 -> 999 */
    duty = (uint16_t)(((speed >= 0) ? speed : -speed) * MOTOR_PWM_MAX / MOTOR_SPEED_MAX);

    if (speed >= 0) {
        __HAL_TIM_SET_COMPARE(htim, pwm_ch, duty);
        __HAL_TIM_SET_COMPARE(htim, pwm_ch_inv, 0);
    } else {
        __HAL_TIM_SET_COMPARE(htim, pwm_ch, 0);
        __HAL_TIM_SET_COMPARE(htim, pwm_ch_inv, duty);
    }
}

// ??????(??:???????????)
void Set_Motor(uint8_t motor_id, int16_t speed) {
    switch (motor_id) {
        case 1: /* 左前 M1  PA0/PA1  TIM2_CH1/CH2 */
            Motor_PWM(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, speed);
            break;
        case 2: /* 右前 M2  PA2/PA3  TIM2_CH3/CH4 */
            Motor_PWM(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, speed);
            break;
        case 3: /* 左后 M3  PA6/PA7  TIM3_CH1/CH2 */
            Motor_PWM(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, speed);
            break;
        case 4: /* 右后 M4  PB0/PB1  TIM3_CH3/CH4 */
            Motor_PWM(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, speed);
            break;
        default:
            break;
    }
}

/* 电机响应测试：依次驱动 4 个通道，正转/反转/停。
 * 用来判断"某个轮子不转"是代码问题还是硬件问题。 */
void Motor_SelfTest(void) {
    static const uint8_t seq[4] = { 1, 2, 3, 4 };

    for (;;) {
        for (uint8_t i = 0; i < 4; i++) {
            /* 正转 */
            Set_Motor(seq[i], MOTOR_SPEED_MAX);
            osDelay(2000);
            /* 停 */
            Set_Motor(seq[i], 0);
            osDelay(800);
            /* 反转 */
            Set_Motor(seq[i], -MOTOR_SPEED_MAX);
            osDelay(2000);
            /* 停 */
            Set_Motor(seq[i], 0);
            osDelay(800);
        }
    }
}

// ?????????????,???Vy??!
void Kinematics(int Vx, int Omega) {
    int V_LF, V_RF, V_LB, V_RB;

    /* Vx 按剩余余量缩放，保证 Vx±Omega 不会互相把对方顶到饱和，
     * 否则满舵转弯时一侧走 100、另一侧只剩 20，车会画龙。 */
    if (Vx != 0) {
        int room = 100 - ((Omega < 0) ? -Omega : Omega);
        if (room < 0) {
            room = 0;
        }
        Vx = Vx * room / 100;
    }

    V_LF = Vx + Omega; // ??
    V_RF = Vx - Omega; // ??
    V_LB = Vx + Omega; // ??
    V_RB = Vx - Omega; // ??

    Set_Motor(1, V_LF);
    Set_Motor(2, V_RF);
    Set_Motor(3, V_LB);
    Set_Motor(4, V_RB);
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* CubeMX 生成的 hi2c1 是 {0} 初始化，OwnAddress1 = 0，也就是“从机模式 1 地址 0”还开着。
   * STM32F1 的 I2C 在外设使能(PE=1)时 OAR1 是只读的，所以必须：
   * 关 PE -> 清 OAR1 的地址位 -> 重新开 PE。
   * 否则偶发把 SDA 拉死，表现为 I2C 卡住不返回。 */
  hi2c1.Instance->CR1 &= ~I2C_CR1_PE;
  hi2c1.Instance->OAR1 &= ~(I2C_OAR1_ADDMODE | I2C_OAR1_ADD1_7 | I2C_OAR1_ADD0);
  hi2c1.Instance->CR1 |= I2C_CR1_PE;

  /* 板载 LED：PC13（CubeMX 的 .ioc 里没有这个引脚，见下方实现） */
  GPIOB_LED_Init();

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
