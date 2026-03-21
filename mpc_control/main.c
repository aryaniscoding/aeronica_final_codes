/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Drone Explicit MPC Controller — STM32F446RE
  *  CubeMX: TIM1 CH1+CH2 PWM, TIM3 CH1+CH2 PWM, USART2 Async 115200
  *  Timer: Prescaler=83, Period=19999. Add matrix.h/c to project.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include "matrix.h"
/* USER CODE END Includes */
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* USER CODE BEGIN PD */
#define PWM_MIN        1000
#define PWM_MAX        2000
#define BASE_THROTTLE  1500
#define STATE_DIM   6
#define CTRL_DIM    3
#define U_MAX       500.0f
#define SLEW_RATE   50.0f
/* USER CODE END PD */
/* USER CODE BEGIN PM */
#define CLAMP(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
/* USER CODE END PM */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Explicit MPC gain (3x6) — computed offline via DARE, horizon N=5 */
static const float K_mpc_data[CTRL_DIM * STATE_DIM] = {
     6.20f, 0.0f,  0.0f,  1.10f, 0.0f,  0.0f,
     0.0f,  6.20f, 0.0f,  0.0f,  1.10f, 0.0f,
     0.0f,  0.0f,  4.80f, 0.0f,  0.0f,  0.85f
};
static Matrix K_mpc;
static float prev_roll=0, prev_pitch=0, prev_yaw=0;
static float prev_u[CTRL_DIM] = {0};
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void Motor_SetPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
static void Sensor_Read(float *roll, float *pitch, float *yaw);
static float Slew_Limit(float u_new, float u_old, float max_delta);
/* USER CODE END PFP */
/* USER CODE BEGIN 0 */
static void Sensor_Read(float *roll, float *pitch, float *yaw)
{ *roll=0; *pitch=0; *yaw=0; }

static void Motor_SetPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, CLAMP(m1, PWM_MIN, PWM_MAX));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, CLAMP(m2, PWM_MIN, PWM_MAX));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, CLAMP(m3, PWM_MIN, PWM_MAX));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, CLAMP(m4, PWM_MIN, PWM_MAX));
}

static float Slew_Limit(float u_new, float u_old, float max_delta)
{
    float delta = u_new - u_old;
    if (delta >  max_delta) return u_old + max_delta;
    if (delta < -max_delta) return u_old - max_delta;
    return u_new;
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  Motor_SetPWM(PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN);
  HAL_Delay(2000);
  mat_set(&K_mpc, CTRL_DIM, STATE_DIM, K_mpc_data);
  uint32_t prev_tick = HAL_GetTick();
  /* USER CODE END 2 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - prev_tick) * 0.001f;
    if (dt < 0.0005f) continue;
    prev_tick = now;

    float roll, pitch, yaw;
    Sensor_Read(&roll, &pitch, &yaw);

    float roll_rate  = (roll  - prev_roll)  / dt;
    float pitch_rate = (pitch - prev_pitch) / dt;
    float yaw_rate   = (yaw   - prev_yaw)   / dt;
    prev_roll = roll; prev_pitch = pitch; prev_yaw = yaw;

    float x[STATE_DIM] = { roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate };
    float u[CTRL_DIM];
    mat_mul_vec(&K_mpc, x, u);
    for (int i = 0; i < CTRL_DIM; i++)
    {
        u[i] = CLAMP(-u[i], -U_MAX, U_MAX);
        u[i] = Slew_Limit(u[i], prev_u[i], SLEW_RATE);
        prev_u[i] = u[i];
    }

    float m1f = BASE_THROTTLE - u[0] + u[1] - u[2];
    float m2f = BASE_THROTTLE + u[0] + u[1] + u[2];
    float m3f = BASE_THROTTLE - u[0] - u[1] + u[2];
    float m4f = BASE_THROTTLE + u[0] - u[1] - u[2];
    Motor_SetPWM((uint16_t)m1f, (uint16_t)m2f, (uint16_t)m3f, (uint16_t)m4f);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM1_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_MIN;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM3_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_MIN;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim3);
}

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
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
