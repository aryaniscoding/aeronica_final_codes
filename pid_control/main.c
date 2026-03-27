/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Drone PID Controller — Roll/Pitch/Yaw Stabilization
  *                   STM32F446RE  |  4 PWM outputs (1000–2000 µs ESC protocol)
  *
  *  CubeMX SETUP REQUIRED:
  *    1. TIM1 → Channel 1 + Channel 2 → PWM Generation
  *    2. TIM3 → Channel 1 + Channel 2 → PWM Generation
  *    3. USART2 → Asynchronous, 115200 baud
  *    4. Timer prescaler = 83, Period = 19999  (1 µs tick, 50 Hz)
  *    5. Generate code, then paste this file as main.c
  *
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief PID controller instance with anti-windup and derivative filtering
 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float prev_derivative;
    float integral_limit;
    float d_filter_alpha;
} PID_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- PWM / ESC limits ---- */
#define PWM_MIN        1000
#define PWM_MAX        2000

/* ---- Base (hover) throttle — tune for your quad ---- */
#define BASE_THROTTLE  1500

/* ---- PID gains (TUNE THESE FOR YOUR DRONE) ---- */
#define PID_ROLL_KP    4.0f
#define PID_ROLL_KI    0.02f
#define PID_ROLL_KD    1.5f

#define PID_PITCH_KP   4.0f
#define PID_PITCH_KI   0.02f
#define PID_PITCH_KD   1.5f

#define PID_YAW_KP     6.0f
#define PID_YAW_KI     0.05f
#define PID_YAW_KD     0.5f

/* ---- Anti-windup & derivative filter ---- */
#define INTEGRAL_LIMIT     200.0f
#define D_FILTER_ALPHA     0.1f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CLAMP(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static PID_t pid_roll;
static PID_t pid_pitch;
static PID_t pid_yaw;

/* Setpoints (radians) — 0 = level flight */
static float setpoint_roll  = 0.0f;
static float setpoint_pitch = 0.0f;
static float setpoint_yaw   = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

static void PID_Init(PID_t *pid, float kp, float ki, float kd,
                     float i_limit, float d_alpha);
static float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt);
static void Motor_SetPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
static void Sensor_Read(float *roll, float *pitch, float *yaw);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void PID_Init(PID_t *pid, float kp, float ki, float kd,
                     float i_limit, float d_alpha)
{
    pid->Kp              = kp;
    pid->Ki              = ki;
    pid->Kd              = kd;
    pid->integral        = 0.0f;
    pid->prev_error      = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->integral_limit  = i_limit;
    pid->d_filter_alpha  = d_alpha;
}

static float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    /* P */
    float p_out = pid->Kp * error;

    /* I with anti-windup */
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float i_out = pid->Ki * pid->integral;

    /* D with low-pass filter */
    float raw_d = (error - pid->prev_error) / dt;
    float filt_d = pid->d_filter_alpha * raw_d
                 + (1.0f - pid->d_filter_alpha) * pid->prev_derivative;
    float d_out = pid->Kd * filt_d;

    pid->prev_error      = error;
    pid->prev_derivative = filt_d;

    return p_out + i_out + d_out;
}

/**
 * @brief  Read calibrated & filtered sensor values
 * TODO:   Replace with your actual IMU driver (MPU6050/BNO055 etc.)
 */
static void Sensor_Read(float *roll, float *pitch, float *yaw)
{
    /* PLACEHOLDER — replace with real sensor read */
    *roll  = 0.0f;
    *pitch = 0.0f;
    *yaw   = 0.0f;
}

static void Motor_SetPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, CLAMP(m1, PWM_MIN, PWM_MAX));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, CLAMP(m2, PWM_MIN, PWM_MAX));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, CLAMP(m3, PWM_MIN, PWM_MAX));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, CLAMP(m4, PWM_MIN, PWM_MAX));
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* Start PWM channels */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* Arm ESCs — 2 s at minimum throttle */
  Motor_SetPWM(PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN);
  HAL_Delay(2000);

  /* Initialise PID controllers */
  PID_Init(&pid_roll,  PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,
           INTEGRAL_LIMIT, D_FILTER_ALPHA);
  PID_Init(&pid_pitch, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD,
           INTEGRAL_LIMIT, D_FILTER_ALPHA);
  PID_Init(&pid_yaw,   PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,
           INTEGRAL_LIMIT, D_FILTER_ALPHA);

  uint32_t prev_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    float    dt  = (float)(now - prev_tick) * 0.001f;
    if (dt < 0.0005f) continue;
    prev_tick = now;

    /* Read sensors */
    float roll_meas, pitch_meas, yaw_meas;
    Sensor_Read(&roll_meas, &pitch_meas, &yaw_meas);

    /* Compute PID outputs */
    float roll_out  = PID_Compute(&pid_roll,  setpoint_roll,  roll_meas,  dt);
    float pitch_out = PID_Compute(&pid_pitch, setpoint_pitch, pitch_meas, dt);
    float yaw_out   = PID_Compute(&pid_yaw,   setpoint_yaw,   yaw_meas,  dt);

    /* X-frame motor mixing
     *  M1 = throttle - roll + pitch - yaw   (front-left,  CW)
     *  M2 = throttle + roll + pitch + yaw   (front-right, CCW)
     *  M3 = throttle - roll - pitch + yaw   (rear-left,   CCW)
     *  M4 = throttle + roll - pitch - yaw   (rear-right,  CW)
     */
    float m1 = BASE_THROTTLE - roll_out - pitch_out - yaw_out; //cw
    float m2 = BASE_THROTTLE + roll_out - pitch_out + yaw_out; //acw
    float m3 = BASE_THROTTLE + roll_out + pitch_out - yaw_out; //acw
    float m4 = BASE_THROTTLE - roll_out + pitch_out + yaw_out; //cw
 


    Motor_SetPWM((uint16_t)m1, (uint16_t)m2, (uint16_t)m3, (uint16_t)m4);

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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 84 - 1;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = 20000 - 1;
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = PWM_MIN;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 0;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 84 - 1;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 20000 - 1;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = PWM_MIN;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
