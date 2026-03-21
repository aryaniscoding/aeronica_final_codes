# Drone Axis Control — PID / LQR / MPC for STM32F446RE

## Files Created

| # | File | Description |
|---|------|-------------|
| 1 | [main.c](file:///z:/aeronica_vs_code_codes/pid_control/main.c) | **PID Controller** — 3 independent PID loops (roll, pitch, yaw) with anti-windup + derivative filtering |
| 2 | [main.c](file:///z:/aeronica_vs_code_codes/lqr_control/main.c) | **LQR Controller** — Pre-computed 3×6 gain matrix K, state = [angles, rates], u = −K·x |
| 3 | [main.c](file:///z:/aeronica_vs_code_codes/mpc_control/main.c) | **Explicit MPC** — DARE-based gain for N=5 horizon + slew-rate limiter, no online QP |
| 4 | [matrix.h](file:///z:/aeronica_vs_code_codes/shared/matrix.h) | Matrix library header (used by LQR + MPC) |
| 5 | [matrix.c](file:///z:/aeronica_vs_code_codes/shared/matrix.c) | Matrix library implementation — multiply, add, sub, scale, mat-vec |

---

## How to use in STM32CubeIDE

### PID Project
Drop [pid_control/main.c](file:///z:/aeronica_vs_code_codes/pid_control/main.c) into `Core/Src/`, replacing the template [main.c](file:///z:/aeronica_vs_code_codes/mpc_control/main.c). No extra files needed.

### LQR Project
1. Copy [lqr_control/main.c](file:///z:/aeronica_vs_code_codes/lqr_control/main.c) → `Core/Src/main.c`
2. Copy [shared/matrix.h](file:///z:/aeronica_vs_code_codes/shared/matrix.h) → `Core/Inc/matrix.h`
3. Copy [shared/matrix.c](file:///z:/aeronica_vs_code_codes/shared/matrix.c) → `Core/Src/matrix.c`

### MPC Project
1. Copy [mpc_control/main.c](file:///z:/aeronica_vs_code_codes/mpc_control/main.c) → `Core/Src/main.c`
2. Copy [shared/matrix.h](file:///z:/aeronica_vs_code_codes/shared/matrix.h) → `Core/Inc/matrix.h`
3. Copy [shared/matrix.c](file:///z:/aeronica_vs_code_codes/shared/matrix.c) → `Core/Src/matrix.c`

---

## Architecture (common to all three)

```
Sensor_Read() → [roll, pitch, yaw] (radians)
       ↓
  Control Law  (PID / LQR / MPC)
       ↓
  [u_roll, u_pitch, u_yaw]
       ↓
  X-Frame Motor Mixing
       ↓
  Motor_SetPWM(M1, M2, M3, M4)  →  TIM1_CH1, TIM1_CH2, TIM3_CH1, TIM3_CH2
```

### Motor Mixing (X-frame, all files identical)
```
M1 = throttle − roll + pitch − yaw   (front-left,  CW)
M2 = throttle + roll + pitch + yaw   (front-right, CCW)
M3 = throttle − roll − pitch + yaw   (rear-left,   CCW)
M4 = throttle + roll − pitch − yaw   (rear-right,  CW)
```

### Timer Configuration
- **TIM1** → Motors 1 & 2 (CH1, CH2) — advanced timer with break/dead-time
- **TIM3** → Motors 3 & 4 (CH1, CH2) — general-purpose timer
- Prescaler = 83 → 1 µs resolution at 84 MHz SYSCLK
- Period = 19999 → 50 Hz (standard ESC protocol)
- PWM range: **1000–2000 µs**

---

## What you MUST customise before flight

> [!CAUTION]
> These files compile and run, but the [Sensor_Read()](file:///z:/aeronica_vs_code_codes/lqr_control/main.c#118-129) function returns zeros. You must plug in your actual IMU driver.

| Item | Where | Notes |
|------|-------|-------|
| **Sensor_Read()** | All 3 [main.c](file:///z:/aeronica_vs_code_codes/mpc_control/main.c) files | Replace placeholder with your IMU read (MPU6050/BNO055 etc.) |
| **PID gains** | [pid_control/main.c](file:///z:/aeronica_vs_code_codes/pid_control/main.c) `#define` section | Tune `Kp/Ki/Kd` for your airframe |
| **LQR gain K** | [lqr_control/main.c](file:///z:/aeronica_vs_code_codes/lqr_control/main.c) `K_data[]` array | Recompute with MATLAB `dlqr(Ad,Bd,Q,R)` for your model |
| **MPC gain K_mpc** | [mpc_control/main.c](file:///z:/aeronica_vs_code_codes/mpc_control/main.c) `K_mpc_data[]` array | Recompute via DARE for your discretised model + horizon |
| **BASE_THROTTLE** | All files | Adjust 1500 to your quad's hover throttle |
| **Timer pin mapping** | STM32CubeMX `.ioc` file | Ensure TIM1_CH1/CH2 and TIM3_CH1/CH2 are mapped to your ESC GPIO pins |

---

## Key Design Choices

- **No heap allocation** — all matrices are stack-allocated with fixed `MAT_MAX_ROWS/COLS = 12`
- **Finite-difference rates** — angular rates (p, q, r) are estimated from successive angle readings; if your IMU provides gyro data directly, use that instead for better accuracy
- **ESC arming** — 2-second minimum-throttle hold at startup
- **Heartbeat LED** — LD2 toggles every loop iteration as a visual alive indicator
