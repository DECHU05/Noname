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
int  deadband(int v, int dead);
void Mecanum_Drive(int Vx, int Vy, int W);
void Mecanum_SelfTest(void);
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
 *  电机控制部分（与机械臂无关）
 *
 *  PWM 满量程 = TIM2/TIM3 的 Period = 999。
 *  原来 pwm_val = speed * 9，speed 最大 100，所以最大只有 900/1000 = 90%，
 *  这是"轮子力量弱"的一个原因。这里改成满量程 100%。
 * ================================================================== */
#define MOTOR_PWM_MAX        999    /* 与 tim.c 里 TIM2/TIM3 的 Period 一致 */
#define MOTOR_SPEED_MAX      100    /* 运动学输出的速度量纲 ±100 */

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

/* ================== X 型麦克纳姆轮运动学 ==================
 *
 * 车体坐标系（俯视）： +X = 车头前进方向， +Y = 车体左侧， +W = 逆时针自转
 * 4 个轮子的安装位置（X 型布局）：
 *
 *        车头 (+X)
 *     M1(左前)   M2(右前)
 *     M3(左后)   M4(右后)
 *
 * X 型布局的轮速组合（辊子方向为 X 形，最常用的一种）：
 *
 *     M1 = +Vx + Vy + W
 *     M2 = +Vx - Vy - W
 *     M3 = +Vx - Vy + W
 *     M4 = +Vx + Vy - W
 *
 * W 的符号是"逆时针为正"，所以右前/右后用 -W 表示它们要反向转。
 *
 * 逻辑轮 => 物理电机通道（沿用原来已验证过的接法）：
 *     M1 左前 -> case 1 -> PA0/PA1  (TIM2_CH1/CH2)
 *     M2 右前 -> case 2 -> PA2/PA3  (TIM2_CH3/CH4)
 *     M3 左后 -> case 3 -> PA6/PA7  (TIM3_CH1/CH2)
 *     M4 右后 -> case 4 -> PB0/PB1  (TIM3_CH3/CH4)
 *
 * ---- 方向标定（唯一可能需要按实车改的地方）----
 * 先给 4 个轮子都喂正速度，看哪个轮子倒转，就把对应的 DIR_x 改成 -1。
 * 判断标准："正速度"应该让该轮推动车向前。
 *   - 若右前/右后轮倒转：是因为电机镜像安装，改 DIR_RF / DIR_RR
 *   - 若是整个底盘左右平移方向相反：改 MECANUM_VY_SIGN
 */
#define DIR_LF   (+1)   /* M1 左前 */
#define DIR_RF   (+1)   /* M2 右前 */
#define DIR_LB   (+1)   /* M3 左后 */
#define DIR_RR   (+1)   /* M4 右后 */

#define MECANUM_VY_SIGN   (+1)   /* 平移方向整体反了就改成 -1（X 型/O 型互换） */

/* 把 |值| < dead 的输入当作 0，避免摇杆回中漂移 */
int deadband(int v, int dead) {
    if (v > -dead && v < dead) return 0;
    return v;
}

/* X 型麦轮正运动学：Vx 前后 / Vy 左右（+为向左）/ W 自转（+为逆时针），量纲都是 -100~+100 */
void Mecanum_Drive(int Vx, int Vy, int W) {
    int m1, m2, m3, m4;
    int max_abs, scale;

    Vy = Vy * MECANUM_VY_SIGN;

    /* 轮速组合 */
    m1 = Vx + Vy + W;   /* 左前 */
    m2 = Vx - Vy - W;   /* 右前 */
    m3 = Vx - Vy + W;   /* 左后 */
    m4 = Vx + Vy - W;   /* 右后 */

    /* 找出最大幅值，避免直接限幅把某个轮子削平导致转向畸变 */
    max_abs = (m1 < 0) ? -m1 : m1;
    if (((m2 < 0) ? -m2 : m2) > max_abs) max_abs = (m2 < 0) ? -m2 : m2;
    if (((m3 < 0) ? -m3 : m3) > max_abs) max_abs = (m3 < 0) ? -m3 : m3;
    if (((m4 < 0) ? -m4 : m4) > max_abs) max_abs = (m4 < 0) ? -m4 : m4;

    if (max_abs == 0) {
        Set_Motor(1, 0);
        Set_Motor(2, 0);
        Set_Motor(3, 0);
        Set_Motor(4, 0);
        return;
    }

    if (max_abs > 100) {
        scale = 100 * 100 / max_abs;      /* 等比缩放，保持各轮比例 */
        m1 = m1 * scale / 100;
        m2 = m2 * scale / 100;
        m3 = m3 * scale / 100;
        m4 = m4 * scale / 100;
    }

    Set_Motor(1, (int16_t)(m1 * DIR_LF));
    Set_Motor(2, (int16_t)(m2 * DIR_RF));
    Set_Motor(3, (int16_t)(m3 * DIR_LB));
    Set_Motor(4, (int16_t)(m4 * DIR_RR));
}

/* ------------------------------------------------------------------
 * 方向标定用：4 个轮子一起给正速度。
 * 在 StartTask03 里把 MECANUM_SELF_TEST 改成 1，上电后观察：
 *   - 4 个轮子都应该把车往前推
 *   - 哪个反了，就改上面那个轮子的 DIR_x
 *   - 如果 4 个"看起来都在往前推"但车实际倒退，说明是左右两侧镜像接法，
 *     把 DIR_RF 和 DIR_RR 同时取反
 * 标定完记得改回 0。
 * ------------------------------------------------------------------ */
#define MECANUM_SELF_TEST 0

void Mecanum_SelfTest(void) {
    for (;;) {
        Mecanum_Drive(60, 0, 0);   /* 纯前进，25 单位/秒 */
        osDelay(1000);
        Mecanum_Drive(0, 0, 0);    /* 停 */
        osDelay(1000);
    }
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
