/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "ps2.h"
#include "pca9685.h"

// ?????????
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1; // ?? TIM1
extern PS2Buttons ps2_data;
extern float arm_A, arm_B, arm_C;

int Limit(int speed);
void Set_Motor(uint8_t motor_id, int16_t speed);
void Kinematics(int Vx, int Omega);
void Motor_SelfTest(void);
float limit_angle(float val, float min, float max);

/* 电机逐个通道自检开关（要排查硬件时改成 1，测完改回 0） */
#ifndef MOTOR_SELF_TEST
#define MOTOR_SELF_TEST 0
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float limit_angle(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/* ==================================================================
 *  ★★★ 上电初始位 + 关节限位（全工程唯一来源，只改这里）★★★
 *  位置都是 PCA9685 计数，约 1 个计数 ≈ 4.88 us
 *  307 ≈ 1500us（舵机中位）；数值越大 -> 脉宽越大 -> 转角一个方向
 * ================================================================== */
/* 上电时三个关节停在哪 */
#define HOME_A             307    /* 夹爪 A 初始位 */
#define HOME_B             307    /* 大臂 B 初始位 */
#define HOME_C             400    /* 小臂 C 初始位 */

/* 关节软限位：超出会被 limit_angle 截断，用来防止撞机械结构。
 * 手动控制和一键夹取都走这几个值，改一处两边同时生效。 */
#define LIM_A_MIN          200
#define LIM_A_MAX          450
#define LIM_B_MIN          100
#define LIM_B_MAX          500
#define LIM_C_MIN          150
#define LIM_C_MAX          500

/* ==================================================================
 *  一键夹取（按住 L1）可调参数区 —— 按你的真实机械臂改这些数字
 *  位置量纲与 arm_A/B/C 相同（PCA9685 计数）
 *  时间单位 ms；任务周期 20 ms，所以时间会被量化到 20 ms
 *  注意：这些只是"自动流程"的目标值，不会改动你手动控制用的参数
 * ================================================================== */
#define GRAB_CLAW_OPEN     400    /* 夹爪张开位置（对应你 R1 的"开"） */
#define GRAB_CLAW_CLOSE    255    /* 夹爪收紧位置（对应你 R1 的"合"） */

#define GRAB_DOWN_B        475    /* 降下来时 B 的目标 */
#define GRAB_DOWN_C        150    /* 降下来时 C 的目标 */

#define GRAB_UP_B          250    /* 收回后 B 的目标（初始位） */
#define GRAB_UP_C          500    /* 收回后 C 的目标（初始位） */

#define GRAB_OPEN_MS       600    /* T1 张开夹爪（准备抓取）用时 */
#define GRAB_DOWN_MS       3000   /* T2 兜底超时：正常到位就提前进入下一步 */
#define GRAB_CLOSE_MS      800    /* T3 夹取用时 */
#define GRAB_HOLD_MS       300    /* T4 夹紧后停留 */
#define GRAB_UP_MS         3000   /* T5 兜底超时：正常到位就提前进入下一步 */

/* --- 放料 + 复位（按"到位后停顿1秒 → 张开 → 停顿1秒 → 回初始位"）--- */
#define GRAB_PAUSE1_MS     1000   /* T6 到位后停顿 1 秒（此时夹爪仍收紧） */
#define GRAB_RELEASE_MS    800    /* T7 张开夹爪用时 */
#define GRAB_PAUSE2_MS     1000   /* T8 张开后停顿 1 秒 */
#define GRAB_RETURN_MS     3000   /* T9 回初始位的兜底超时 */

/* 注意：GRAB_DOWN_MS / GRAB_UP_MS / GRAB_RETURN_MS 现在只是"卡住保护"，
 * 正常情况下关节走到目标就立刻进入下一阶段，不再靠这些时间硬切。 */

/* 每 20 ms 让关节朝目标走多少个计数：越小越稳越慢，建议 2~6 */
#define GRAB_MOVE_STEP     4

static uint8_t  grab_state = 0;      /* 0 = 空闲，1~9 = 九个阶段 */
static uint32_t grab_tick  = 0;      /* 当前阶段已过 ms */

/* 判断 B/C 是否已经到达指定目标（grab_move_toward 会精确写到目标值） */
static uint8_t grab_at_target(float b, float c) {
    return (uint8_t)((arm_B == b) && (arm_C == c));
}

/* 把关节值朝目标值靠拢，每 20 ms 走 GRAB_MOVE_STEP */
static void grab_move_toward(float *cur, float target) {
    if (*cur < target) {
        *cur += GRAB_MOVE_STEP;
        if (*cur > target) *cur = target;
    } else if (*cur > target) {
        *cur -= GRAB_MOVE_STEP;
        if (*cur < target) *cur = target;
    }
}

/* 一键夹取状态机。返回 1 = 自动流程进行中（调用方应屏蔽手动控制） */
static uint8_t grab_task_auto(uint8_t l1) {
    static uint8_t l1_prev = 0;

    /* L1 下降沿启动一次，按住不会反复触发 */
    if (l1 == 1 && l1_prev == 0 && grab_state == 0) {
        grab_state = 1;
        grab_tick = 0;
    }
    l1_prev = l1;

    if (grab_state == 0) {
        return 0;
    }

    grab_tick += 20;   /* 与主循环 osDelay(20) 对齐 */

    switch (grab_state) {
        case 1:   /* T1 张开夹爪 */
            arm_A = GRAB_CLAW_OPEN;
            if (grab_tick >= GRAB_OPEN_MS) { grab_state = 2; grab_tick = 0; }
            break;

        case 2:   /* T2 降下来：走到目标才继续（不是等固定时间） */
            grab_move_toward(&arm_B, GRAB_DOWN_B);
            grab_move_toward(&arm_C, GRAB_DOWN_C);
            if (grab_at_target((float)GRAB_DOWN_B, (float)GRAB_DOWN_C)
                || grab_tick >= GRAB_DOWN_MS) {
                grab_state = 3; grab_tick = 0;
            }
            break;

        case 3:   /* T3 夹取 */
            arm_A = GRAB_CLAW_CLOSE;
            if (grab_tick >= GRAB_CLOSE_MS) { grab_state = 4; grab_tick = 0; }
            break;

        case 4:   /* T4 夹紧停留 */
            arm_A = GRAB_CLAW_CLOSE;
            if (grab_tick >= GRAB_HOLD_MS) { grab_state = 5; grab_tick = 0; }
            break;

        case 5:   /* T5 走到设定的收回位置：夹爪全程保持收紧，到位才继续 */
            arm_A = GRAB_CLAW_CLOSE;
            grab_move_toward(&arm_B, GRAB_UP_B);
            grab_move_toward(&arm_C, GRAB_UP_C);
            if (grab_at_target((float)GRAB_UP_B, (float)GRAB_UP_C)
                || grab_tick >= GRAB_UP_MS) {
                grab_state = 6; grab_tick = 0;
            }
            break;

        case 6:   /* T6 到位后停顿 1 秒，夹爪仍然收紧 */
            arm_A = GRAB_CLAW_CLOSE;
            if (grab_tick >= GRAB_PAUSE1_MS) { grab_state = 7; grab_tick = 0; }
            break;

        case 7:   /* T7 张开夹爪（放料） */
            arm_A = GRAB_CLAW_OPEN;
            if (grab_tick >= GRAB_RELEASE_MS) { grab_state = 8; grab_tick = 0; }
            break;

        case 8:   /* T8 张开后停顿 1 秒 */
            arm_A = GRAB_CLAW_OPEN;
            if (grab_tick >= GRAB_PAUSE2_MS) { grab_state = 9; grab_tick = 0; }
            break;

        case 9:   /* T9 三个关节回到上电初始位（给电位置） */
            grab_move_toward(&arm_A, HOME_A);
            grab_move_toward(&arm_B, HOME_B);
            grab_move_toward(&arm_C, HOME_C);
            if ((arm_A == (float)HOME_A && arm_B == (float)HOME_B && arm_C == (float)HOME_C)
                || grab_tick >= GRAB_RETURN_MS) {
                grab_state = 0;
                grab_tick = 0;
            }
            break;

        default:
            grab_state = 0;
            break;
    }
    return 1;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ps2Handle;
osThreadId classesHandle;
osThreadId armHandle;
osThreadId debugHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ps2 */
  osThreadDef(ps2, StartTask02, osPriorityHigh, 0, 256);
  ps2Handle = osThreadCreate(osThread(ps2), NULL);

  /* definition and creation of classes */
  osThreadDef(classes, StartTask03, osPriorityAboveNormal, 0, 256);
  classesHandle = osThreadCreate(osThread(classes), NULL);

  /* definition and creation of arm */
  osThreadDef(arm, StartTask04, osPriorityNormal, 0, 256);
  armHandle = osThreadCreate(osThread(arm), NULL);

  /* definition and creation of debug */
  osThreadDef(debug, StartTask05, osPriorityLow, 0, 128);
  debugHandle = osThreadCreate(osThread(debug), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the ps2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	PS2_Init(&htim1, &ps2_data); // ? &htim4 ?? &htim1
  /* Infinite loop */
  for(;;)
  {
    PS2_Update();
    osDelay(20);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the classes thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  /* 电机自检（编译期开关，见 main.c 里的 MOTOR_SELF_TEST）。
   * 置 1 时会逐个通道正转 2s / 反转 2s，用于确认"某个轮子不转"是接线还是代码。 */
#if MOTOR_SELF_TEST
  Motor_SelfTest();
#endif
  /* Infinite loop */
  for(;;)
  {
     
    int Vx, Omega;
    
    taskENTER_CRITICAL();   
    /* 原来这里给的是 60/100，只有六成动力，配合 pwm_val=speed*9 的 90% 上限，
     * 实际只发挥了 54% 的电机能力 —— 这就是"力量很弱"的主因。
     * 改成 100（满速）。觉得太快就把它调小，例如 80。 */
    if (ps2_data.UP)        Vx =  100;
    else if (ps2_data.DOWN) Vx = -100;
    else                    Vx = 0;
    
    Omega = ps2_data.RX - 128;   // 右摇杆左右
    taskEXIT_CRITICAL();

    /* ================= 自转（原地转向）调参 =================
     * 原来：死区 ±25 之后 Omega * 80 / 128  -> 满舵只有 79% 占空比
     * 现在：同样死区，满量程映射到 100      -> 满舵 99%，全程出力都比原来高
     *       ROT_GAIN 调小可恢复柔和手感
     *       ROT_DEAD 调小可让中段更灵敏（但松手漂移风险变大）
     * ======================================================= */
#define ROT_DEAD   25     /* 自转死区（防松手漂移） */
#define ROT_GAIN   100    /* 自转满舵输出：100 = 满占空比（原来是 80） */
    if (Omega > -ROT_DEAD && Omega < ROT_DEAD) {
        Omega = 0;
    } else {
        Omega = Omega * ROT_GAIN / 128;
        if (Omega >  100) Omega =  100;
        if (Omega < -100) Omega = -100;
    }
    
    Kinematics(Vx, Omega);
    osDelay(20);
  
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the arm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  osDelay(100);
  PCA9685_Init();
  osDelay(100);

  /* ===== 机械臂手感调参区（改这几个数字即可，不用动逻辑） ===== */
#define ARM_STEP_DIV     20    /* 摇杆幅度 -> 速度：分母越小越快。24 => 满舵 5 步/周期 */
#define ARM_STEP_MIN     20    /* 死区：|摇杆-128| 小于它就认为没推 */
#define ARM_STEP_MAX     6     /* 速度上限(步/周期)：6 步/20ms = 300 单位/秒，防止甩到底 */
  /* 50 Hz 下 1 个计数约 4.88 us，所以 100 个计数约 490 us 的行程，安全范围 */

  // 上电先把 3 个关节写到 HOME_A/B/C（宏定义见文件上方 USER CODE PM），
  // 并同步 arm_A/B/C，否则 claw_state=0 与舵机真实状态不一致，第一次按 R1 会突跳
  taskENTER_CRITICAL();
  arm_A = HOME_A;
  arm_B = HOME_B;
  arm_C = HOME_C;
  taskEXIT_CRITICAL();
  PCA9685_SetPWM(0, HOME_A);
  PCA9685_SetPWM(1, HOME_B);
  PCA9685_SetPWM(2, HOME_C);

  osDelay(400); // 等舵机走到中位（够用即可，缩短上电等待）
	uint8_t last_r1 = 0;
  uint8_t claw_state = 0;
  /* Infinite loop */
  for(;;)
  {
    // 1. 整帧读取手柄数据：ps2 任务优先级更高，会在这里插进来改 ps2_data，
    //    不在临界区里读可能拿到“半新半旧”的一帧
    uint8_t js_LY, js_RY, js_R1, js_L1;
    taskENTER_CRITICAL();
    js_LY = ps2_data.LY;
    js_RY = ps2_data.RY;
    js_R1 = ps2_data.R1;
    js_L1 = ps2_data.L1;
    taskEXIT_CRITICAL();

    // 0. 一键夹取：按住 L1 触发，自动流程执行期间屏蔽摇杆和 R1 手动控制
    uint8_t auto_running = grab_task_auto(js_L1);
    if (auto_running) {
      arm_A = limit_angle(arm_A, LIM_A_MIN, LIM_A_MAX);
      arm_B = limit_angle(arm_B, LIM_B_MIN, LIM_B_MAX);
      arm_C = limit_angle(arm_C, LIM_C_MIN, LIM_C_MAX);
      PCA9685_SetPWM(0, (uint16_t)arm_A);
      PCA9685_SetPWM(1, (uint16_t)arm_B);
      PCA9685_SetPWM(2, (uint16_t)arm_C);
      osDelay(20);
      continue;
    }

    int LY = js_LY - 128;
    int RY = js_RY - 128;
    int step_LY = 0;
    int step_RY = 0;

    // 2. 摇杆幅度映射成速度：轻推=微调，推到底=快速（原来只有固定的 1 步，太慢）
    if (LY > ARM_STEP_MIN || LY < -ARM_STEP_MIN) {
      step_LY = LY / ARM_STEP_DIV;
      if (step_LY == 0) {
        step_LY = (LY > 0) ? 1 : -1;   // 保证刚出死区能动
      }
      if (step_LY >  ARM_STEP_MAX) step_LY =  ARM_STEP_MAX;
      if (step_LY < -ARM_STEP_MAX) step_LY = -ARM_STEP_MAX;
    }
    if (RY > ARM_STEP_MIN || RY < -ARM_STEP_MIN) {
      step_RY = RY / ARM_STEP_DIV;
      if (step_RY == 0) {
        step_RY = (RY > 0) ? 1 : -1;
      }
      if (step_RY >  ARM_STEP_MAX) step_RY =  ARM_STEP_MAX;
      if (step_RY < -ARM_STEP_MAX) step_RY = -ARM_STEP_MAX;
    }

    // 3. 方向已按实测翻转：推上 -> B 减小
    arm_B -= step_LY;
    // 4. 推右 -> C 增大
    arm_C += step_RY;

    // 5. R1 下降沿 -> 切换夹爪 A（两个位置对调即可交换开合方向）
    if (js_R1 == 1 && last_r1 == 0) {
        claw_state = !claw_state;
        if (claw_state == 1) { arm_A = 255; } // 合
        else                 { arm_A = 400; } // 开
    }
    last_r1 = js_R1;

    // 6. 关节限位，防止撞机械结构（数值见文件上方 LIM_* 宏）
    arm_A = limit_angle(arm_A, LIM_A_MIN, LIM_A_MAX);
    arm_B = limit_angle(arm_B, LIM_B_MIN, LIM_B_MAX);
    arm_C = limit_angle(arm_C, LIM_C_MIN, LIM_C_MAX);

    // 7. 输出到 PCA9685
    PCA9685_SetPWM(0, (uint16_t)arm_A);
    PCA9685_SetPWM(1, (uint16_t)arm_B);
    PCA9685_SetPWM(2, (uint16_t)arm_C);

    osDelay(20);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* 心跳灯 + PCA9685 自检：
   *  400ms 均匀慢闪  = 一切正常（PCA9685 有应答）
   *  100ms 急促快闪  = PCA9685 没有应答（I2C 接线/供电/地址问题）
   * 完全不闪       = 调度器没跑起来或卡死在高优先级任务里
   * 等 1.2 秒是为了让 StartTask04 先完成 PCA9685_Init() 再判断结果。 */
  osDelay(1200);
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay((pca9685_ok != 0u) ? 400 : 100);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

