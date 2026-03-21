/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Drone DYNAMIC MPC Controller — STM32F446RE
  *  Solves a QP at every control step using projected gradient descent.
  *  Horizon N=5, 15 decision variables, ~1-2ms per solve at 84MHz.
  *  CubeMX: TIM1 CH1+CH2 PWM, TIM3 CH1+CH2 PWM, USART2 115200
  *  Timer:  Prescaler=83, Period=19999. Add matrix.h/c to project.
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
#define HORIZON     5                             /* N = 5 steps */
#define QP_VARS     (HORIZON * CTRL_DIM)          /* 15 decision variables */
#define PRED_ROWS   (HORIZON * STATE_DIM)         /* 30 rows in Phi/Gamma */

#define U_LIM       150.0f    /* Tight actuator constraint */
#define QP_MAX_ITER 60        /* Max projected gradient iterations */
#define QP_TOL      1e-4f     /* Convergence tolerance */

/* USER CODE END PD */
/* USER CODE BEGIN PM */
#define CLAMP(x, lo, hi) ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
/* USER CODE END PM */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* ---- Pre-computed offline (from MATLAB) ---- */

/* Hessian H (15x15) = Gamma' * Q_bar * Gamma + R_bar
   Computed offline, stored as flat row-major array.
   Run sim_mpc_dynamic.m to recompute if you change Q, R, or N. */
static float H_flat[QP_VARS * QP_VARS];

/* Gamma' * Q_bar (15 x 30) — for computing f = GtQ * (Phi*x - X_ref) */
static float GtQ_flat[QP_VARS * PRED_ROWS];

/* Phi (30 x 6) — prediction matrix */
static float Phi_flat[PRED_ROWS * STATE_DIM];

/* Step size for projected gradient (1 / max_eigenvalue(H)) */
static float qp_alpha = 0.0f;

/* Previous angles for rate estimation */
static float prev_roll = 0.0f, prev_pitch = 0.0f, prev_yaw = 0.0f;

/* ---- Discretised system matrices (computed at startup) ---- */
/* Ad = I + A_c * Ts,  Bd = B_c * Ts */
static float Ad_flat[STATE_DIM * STATE_DIM];
static float Bd_flat[STATE_DIM * CTRL_DIM];

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void Motor_SetPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
static void Sensor_Read(float *roll, float *pitch, float *yaw);
static void Build_MPC_Matrices(float Ts);
static void Solve_QP(const float *f_vec, float *U_opt);
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

/* ----------------------------------------------------------------
 *  Build_MPC_Matrices — compute Ad, Bd, Phi, Gamma, H, GtQ, alpha
 *  Called ONCE at startup (offline precomputation on-chip)
 * ---------------------------------------------------------------- */
static void Build_MPC_Matrices(float Ts)
{
    const float Ixx = 0.0142f, Iyy = 0.0142f, Izz = 0.0284f;
    const float damp = 0.5f;

    /* Q and R cost matrices (diagonal) */
    float Q[STATE_DIM] = {200.0f, 200.0f, 160.0f, 20.0f, 20.0f, 16.0f};
    float Ri[CTRL_DIM] = {0.5f, 0.5f, 0.5f};

    int i, j, s, r, c;

    /* ---- Build Ad (6x6) = I + A_c * Ts ---- */
    memset(Ad_flat, 0, sizeof(Ad_flat));
    for (i = 0; i < STATE_DIM; i++) Ad_flat[i*STATE_DIM+i] = 1.0f;
    /* Add A_c * Ts: rows 0-2 get +Ts in rate columns */
    Ad_flat[0*STATE_DIM+3] = Ts;    /* roll_dot = p */
    Ad_flat[1*STATE_DIM+4] = Ts;    /* pitch_dot = q */
    Ad_flat[2*STATE_DIM+5] = Ts;    /* yaw_dot = r */
    /* Damping on rates */
    Ad_flat[3*STATE_DIM+3] += (-damp/Ixx)*Ts;
    Ad_flat[4*STATE_DIM+4] += (-damp/Iyy)*Ts;
    Ad_flat[5*STATE_DIM+5] += (-damp/Izz)*Ts;

    /* ---- Build Bd (6x3) = B_c * Ts ---- */
    memset(Bd_flat, 0, sizeof(Bd_flat));
    Bd_flat[3*CTRL_DIM+0] = Ts / Ixx;
    Bd_flat[4*CTRL_DIM+1] = Ts / Iyy;
    Bd_flat[5*CTRL_DIM+2] = Ts / Izz;

    /* ---- Build Phi (30x6) and Gamma (30x15) ---- */
    memset(Phi_flat, 0, sizeof(Phi_flat));
    float Gamma_flat[PRED_ROWS * QP_VARS];
    memset(Gamma_flat, 0, sizeof(Gamma_flat));

    /* Ad_powers[i] = Ad^(i+1), stored flat 6x6 */
    float Ad_pow[HORIZON][STATE_DIM * STATE_DIM];

    /* Ad^1 = Ad */
    memcpy(Ad_pow[0], Ad_flat, sizeof(Ad_flat));

    /* Ad^i for i=2..N */
    for (i = 1; i < HORIZON; i++) {
        /* Ad_pow[i] = Ad_pow[i-1] * Ad */
        for (r = 0; r < STATE_DIM; r++) {
            for (c = 0; c < STATE_DIM; c++) {
                float sum = 0.0f;
                for (s = 0; s < STATE_DIM; s++)
                    sum += Ad_pow[i-1][r*STATE_DIM+s] * Ad_flat[s*STATE_DIM+c];
                Ad_pow[i][r*STATE_DIM+c] = sum;
            }
        }
    }

    /* Fill Phi: row block i = Ad^(i+1) */
    for (i = 0; i < HORIZON; i++) {
        for (r = 0; r < STATE_DIM; r++)
            for (c = 0; c < STATE_DIM; c++)
                Phi_flat[(i*STATE_DIM+r)*STATE_DIM+c] = Ad_pow[i][r*STATE_DIM+c];
    }

    /* Fill Gamma: block (i,j) = Ad^(i-j) * Bd for j<=i */
    for (i = 0; i < HORIZON; i++) {
        for (j = 0; j <= i; j++) {
            /* power = i - j: if 0, use I*Bd=Bd; else Ad_pow[i-j-1]*Bd */
            int pw = i - j;
            for (r = 0; r < STATE_DIM; r++) {
                for (c = 0; c < CTRL_DIM; c++) {
                    float sum = 0.0f;
                    if (pw == 0) {
                        sum = Bd_flat[r*CTRL_DIM+c];
                    } else {
                        for (s = 0; s < STATE_DIM; s++)
                            sum += Ad_pow[pw-1][r*STATE_DIM+s] * Bd_flat[s*CTRL_DIM+c];
                    }
                    Gamma_flat[(i*STATE_DIM+r)*QP_VARS + j*CTRL_DIM+c] = sum;
                }
            }
        }
    }

    /* ---- Build Q_bar (30x30 diagonal) ---- */
    /* Q for stages 0..N-2, then P_terminal for stage N-1.
       For simplicity, we use Q everywhere (P_terminal ≈ Q for short horizon). */
    float Q_bar_diag[PRED_ROWS];
    for (i = 0; i < HORIZON; i++)
        for (r = 0; r < STATE_DIM; r++)
            Q_bar_diag[i*STATE_DIM+r] = Q[r];

    /* ---- Compute GtQ = Gamma' * Q_bar (15x30) ---- */
    /* Since Q_bar is diagonal: GtQ[r][c] = Gamma[c][r] * Q_bar[c][c] */
    for (r = 0; r < QP_VARS; r++) {
        for (c = 0; c < PRED_ROWS; c++) {
            GtQ_flat[r*PRED_ROWS+c] = Gamma_flat[c*QP_VARS+r] * Q_bar_diag[c];
        }
    }

    /* ---- Compute H = GtQ * Gamma + R_bar (15x15) ---- */
    for (r = 0; r < QP_VARS; r++) {
        for (c = 0; c < QP_VARS; c++) {
            float sum = 0.0f;
            for (s = 0; s < PRED_ROWS; s++)
                sum += GtQ_flat[r*PRED_ROWS+s] * Gamma_flat[s*QP_VARS+c];
            H_flat[r*QP_VARS+c] = sum;
        }
        /* Add R on diagonal */
        H_flat[r*QP_VARS+r] += Ri[r % CTRL_DIM];
    }

    /* Symmetrise H */
    for (r = 0; r < QP_VARS; r++)
        for (c = r+1; c < QP_VARS; c++) {
            float avg = 0.5f * (H_flat[r*QP_VARS+c] + H_flat[c*QP_VARS+r]);
            H_flat[r*QP_VARS+c] = avg;
            H_flat[c*QP_VARS+r] = avg;
        }

    /* ---- Compute step size: 1 / max row abs sum (Gershgorin bound) ---- */
    float max_row_sum = 0.0f;
    for (r = 0; r < QP_VARS; r++) {
        float row_sum = 0.0f;
        for (c = 0; c < QP_VARS; c++)
            row_sum += fabsf(H_flat[r*QP_VARS+c]);
        if (row_sum > max_row_sum) max_row_sum = row_sum;
    }
    qp_alpha = 1.0f / max_row_sum;
}

/* ----------------------------------------------------------------
 *  Solve_QP — projected Nesterov gradient descent
 *  Solves:  min (1/2)*U'*H*U + f'*U  s.t.  -U_LIM <= U <= U_LIM
 * ---------------------------------------------------------------- */
static void Solve_QP(const float *f_vec, float *U_opt)
{
    int i, j, it;
    float U[QP_VARS], U_prev[QP_VARS], V[QP_VARS], grad[QP_VARS];

    /* Warm start: unconstrained U = -H\f approximated by gradient step */
    for (i = 0; i < QP_VARS; i++) {
        U[i] = CLAMP(-f_vec[i] * qp_alpha, -U_LIM, U_LIM);
        U_prev[i] = U[i];
    }

    for (it = 0; it < QP_MAX_ITER; it++) {
        /* Nesterov momentum */
        float beta = (float)(it) / (float)(it + 3);
        for (i = 0; i < QP_VARS; i++)
            V[i] = U[i] + beta * (U[i] - U_prev[i]);

        /* Gradient: grad = H * V + f */
        for (i = 0; i < QP_VARS; i++) {
            float s = f_vec[i];
            for (j = 0; j < QP_VARS; j++)
                s += H_flat[i*QP_VARS+j] * V[j];
            grad[i] = s;
        }

        /* Gradient step + projection */
        float max_change = 0.0f;
        for (i = 0; i < QP_VARS; i++) {
            U_prev[i] = U[i];
            float u_new = V[i] - qp_alpha * grad[i];
            U[i] = CLAMP(u_new, -U_LIM, U_LIM);
            float ch = fabsf(U[i] - U_prev[i]);
            if (ch > max_change) max_change = ch;
        }

        if (max_change < QP_TOL) break;
    }

    for (i = 0; i < QP_VARS; i++) U_opt[i] = U[i];
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

  /* Build all MPC matrices (takes ~10ms, done once) */
  Build_MPC_Matrices(0.01f);   /* Ts = 10 ms = 100 Hz */

  uint32_t prev_tick = HAL_GetTick();
  /* USER CODE END 2 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - prev_tick) * 0.001f;
    if (dt < 0.005f) continue;   /* ~100 Hz max */
    prev_tick = now;

    /* Read sensors */
    float roll, pitch, yaw;
    Sensor_Read(&roll, &pitch, &yaw);

    /* Estimate rates */
    float p = (roll  - prev_roll)  / dt;
    float q = (pitch - prev_pitch) / dt;
    float r_rate = (yaw - prev_yaw) / dt;
    prev_roll = roll; prev_pitch = pitch; prev_yaw = yaw;

    /* State vector */
    float x[STATE_DIM] = { roll, pitch, yaw, p, q, r_rate };

    /* ---- Compute f = GtQ * (Phi * x - X_ref) ---- */
    /* For simplicity, X_ref = 0 (level flight) */
    /* Phi_x = Phi * x (30x1) */
    float Phi_x[PRED_ROWS];
    int i, j;
    for (i = 0; i < PRED_ROWS; i++) {
        float s = 0.0f;
        for (j = 0; j < STATE_DIM; j++)
            s += Phi_flat[i*STATE_DIM+j] * x[j];
        Phi_x[i] = s;
    }

    /* f = GtQ * Phi_x (15x1) */
    float f_vec[QP_VARS];
    for (i = 0; i < QP_VARS; i++) {
        float s = 0.0f;
        for (j = 0; j < PRED_ROWS; j++)
            s += GtQ_flat[i*PRED_ROWS+j] * Phi_x[j];
        f_vec[i] = s;
    }

    /* ---- Solve QP ---- */
    float U_opt[QP_VARS];
    Solve_QP(f_vec, U_opt);

    /* Apply FIRST control only (receding horizon) */
    float u0 = U_opt[0];
    float u1 = U_opt[1];
    float u2 = U_opt[2];

    /* X-frame motor mixing */
    float m1f = BASE_THROTTLE - u0 + u1 - u2;
    float m2f = BASE_THROTTLE + u0 + u1 + u2;
    float m3f = BASE_THROTTLE - u0 - u1 + u2;
    float m4f = BASE_THROTTLE + u0 - u1 - u2;
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
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
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
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
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
