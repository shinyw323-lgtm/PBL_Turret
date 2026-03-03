/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Gimbal Controller - 2-Axis PID Control with CAN Communication
  *                   - Manual Mode: Direction control via CAN (0x101)
  *                   - Auto Mode: PID tracking via CAN (0x110)
  *                   - Mode Switch: CAN (0x111)
  *                   - Laser Control: CAN (0x120)
  *                   - Heartbeat: CAN (0x702) every 500ms
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// PID 제어기 구조체
typedef struct {
    float Kp;           // 비례 게인
    float Ki;           // 적분 게인
    float Kd;           // 미분 게인

    float integral;     // 적분 누적값
    float prev_error;   // 이전 오차 (미분용)

    float output_min;   // 출력 하한
    float output_max;   // 출력 상한

    float integral_max; // Anti-windup: 적분 상한
} PID_Controller;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// PID 제어 주기 (ms)
#define PID_PERIOD_MS       50

// PWM 주파수 범위 (Hz) - TB6560 펄스 주파수 (1kHz 이내)
#define PWM_FREQ_MIN        50      // 최소 펄스 주파수
#define PWM_FREQ_MAX        1000    // 최대 펄스 주파수 (TB6560 제한)

// Dead zone: 이 오차 이내면 모터 정지
#define DEAD_ZONE           5

// 타이머 클럭 (APB1 = 42MHz, prescaler != 1이므로 TIM 클럭 = APB1 * 2 = 84MHz)
#define TIMER_CLOCK         84000000UL

// 수동 모드 속도 (고정 PWM 주파수)
#define MANUAL_SPEED_HZ     200

// 수동 모드 펄스 지속 시간 (200ms = 5 * 10ms)
#define MANUAL_TIMEOUT_COUNT  2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// PAN (X축) PID 컨트롤러
// 입력 범위: X = -960 ~ +960 (해상도 1920x1080 기준)
// 출력 범위: PWM 주파수 제어용 (50Hz ~ 1000Hz)
PID_Controller pid_pan = {
    .Kp = 0.011f,           // 초기값 (튜닝 필요)
    .Ki = 0.002f,           // 초기값 (P 튜닝 후 조정)
    .Kd = 0.002f,           // 초기값 (PI 튜닝 후 조정)
    .integral = 0.0f,
    .prev_error = 0.0f,
    .output_min = -1000.0f,
    .output_max = 1000.0f,
    .integral_max = 500.0f
};

// TILT (Y축) PID 컨트롤러
// 입력 범위: Y = -540 ~ +540 (해상도 1920x1080 기준)
// 출력 범위: PWM 주파수 제어용 (50Hz ~ 1000Hz)
PID_Controller pid_tilt = {
    .Kp = 0.011f,           // 초기값 (튜닝 필요)
    .Ki = 0.002f,           // 초기값 (P 튜닝 후 조정)
    .Kd = 0.002f,           // 초기값 (PI 튜닝 후 조정)
    .integral = 0.0f,
    .prev_error = 0.0f,
    .output_min = -1000.0f,
    .output_max = 1000.0f,
    .integral_max = 500.0f
};

// PID 제어 타이밍
uint32_t last_pid_time = 0;

// 현재 오차값 (CAN으로 수신)
volatile int16_t error_x = 0;
volatile int16_t error_y = 0;

// 제어 모드: 0 = 자동 (PID), 1 = 수동
volatile uint8_t control_mode = 1;  // 초기값: 수동 모드

// 레이저 상태: 0 = OFF, 1 = ON
volatile uint8_t laser_state = 0;

// 수동 모드 타이머 카운트 (10ms 단위)
volatile uint8_t manual_pan_timer = 0;
volatile uint8_t manual_tilt_timer = 0;

// 수동 모드 방향 플래그
volatile uint8_t manual_pan_dir = 0;   // 0: 정지, 1: CCW(좌), 2: CW(우)
volatile uint8_t manual_tilt_dir = 0;  // 0: 정지, 1: CCW(상), 2: CW(하)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// PID 관련 함수 선언
void PID_Init(PID_Controller *pid);
float PID_Compute(PID_Controller *pid, float setpoint, float measurement, float dt);
void Motor_SetPWMFrequency(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz);
void Motor_Stop(TIM_HandleTypeDef *htim, uint32_t channel);
void Control_PAN(float pid_output);
void Control_TILT(float pid_output);

// 레이저 제어
void Laser_Set(uint8_t state);
void Laser_Toggle(void);

// 수동 모드 제어
void Manual_ControlPAN(uint8_t left, uint8_t right);
void Manual_ControlTILT(uint8_t up, uint8_t down);

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // PID 초기화
  PID_Init(&pid_pan);
  PID_Init(&pid_tilt);

  // PWM 시작 (초기에는 정지 상태)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // PAN (PB5)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // TILT (PB6)
  Motor_Stop(&htim3, TIM_CHANNEL_2);
  Motor_Stop(&htim4, TIM_CHANNEL_1);

  // 타이머 인터럽트 시작 (10ms 주기 - 수동 모드 타이머 & 하트비트)
  HAL_TIM_Base_Start_IT(&htim2);

  // CAN 필터 설정 및 시작
  MX_CAN1_Filter_Gimbal();
  HAL_CAN_Start(&hcan1);

  // 레이저 초기화 (OFF)
  Laser_Set(0);

  // 타이밍 초기화
  last_pid_time = HAL_GetTick();

  printf("\r\n========================================\r\n");
  printf("    Gimbal Controller Initialized\r\n");
  printf("    Mode: %s\r\n", control_mode ? "MANUAL" : "AUTO (PID)");
  printf("    Laser: %s\r\n", laser_state ? "ON" : "OFF");
  printf("========================================\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t current_time = HAL_GetTick();

      // ========== CAN 메시지 처리 ==========
      CAN_HandleRxMessage();

      // ========== 모드 토글 처리 ==========
      if (CAN_ConsumeModeToggle())
      {
          control_mode = !control_mode;

          // 모드 전환 시 모터 정지 및 PID 리셋
          Motor_Stop(&htim3, TIM_CHANNEL_2);
          Motor_Stop(&htim4, TIM_CHANNEL_1);
          PID_Init(&pid_pan);
          PID_Init(&pid_tilt);
          manual_pan_timer = 0;
          manual_tilt_timer = 0;

          printf("[MODE] Switched to %s mode\r\n", control_mode ? "MANUAL" : "AUTO (PID)");
      }

      // ========== 레이저 ON/OFF 처리 ==========
      {
          uint8_t laser_on;
          if (CAN_GetLaserCmd(&laser_on))
          {
              Laser_Set(laser_on);
              printf("[LASER] %s\r\n", laser_state ? "ON" : "OFF");
          }
      }

      // ========== 수동 모드 ==========
      if (control_mode == 1)
      {
          uint8_t up, down, left, right;

          // CAN에서 수동 제어 명령 수신
          if (CAN_GetManualCmd(&up, &down, &left, &right))
          {
              Manual_ControlPAN(left, right);
              Manual_ControlTILT(up, down);
          }

          // PAN 타이머 만료 → 정지
          if (manual_pan_timer == 0 && manual_pan_dir != 0)
          {
              Motor_Stop(&htim3, TIM_CHANNEL_2);
              manual_pan_dir = 0;
          }

          // TILT 타이머 만료 → 정지
          if (manual_tilt_timer == 0 && manual_tilt_dir != 0)
          {
              Motor_Stop(&htim4, TIM_CHANNEL_1);
              manual_tilt_dir = 0;
          }
      }
      // ========== 자동 (PID) 모드 ==========
      // 좌표 범위: X = -960 ~ +960, Y = -540 ~ +540 (해상도 1920x1080 기준)
      else
      {
          int16_t err_x, err_y;

          // CAN에서 자동 제어 명령 수신 (오차값)
          if (CAN_GetAutoCmd(&err_x, &err_y))
          {
              __disable_irq();
              error_x = err_x;
              error_y = err_y;
              __enable_irq();

              // 드론 오차 좌표 로그 출력 (10진수)
              printf("[AUTO] Drone Error: X=%d, Y=%d\r\n", err_x, err_y);
          }

          // PID 제어 주기 체크 (50ms 마다)
          if ((current_time - last_pid_time) >= PID_PERIOD_MS)
          {
              float dt = (current_time - last_pid_time) / 1000.0f;  // 초 단위
              last_pid_time = current_time;

              // 현재 오차값 복사 (인터럽트 보호)
              __disable_irq();
              int16_t local_err_x = error_x;
              int16_t local_err_y = error_y;
              __enable_irq();

              // PID 계산 (목표: 오차 0)
              // X양수 → PAN CW, X음수 → PAN CCW
              // Y양수 → TILT CCW, Y음수 → TILT CW
              float output_pan = PID_Compute(&pid_pan, 0.0f, (float)local_err_x, dt);
              float output_tilt = PID_Compute(&pid_tilt, 0.0f, (float)local_err_y, dt);

              // PID 출력 로그 (튜닝용) - float를 정수로 변환 출력
              printf("[PID] X_err=%d, Y_err=%d, PAN_out=%d, TILT_out=%d\r\n",
                     local_err_x, local_err_y, (int)output_pan, (int)output_tilt);

              // 모터 제어 적용
              Control_PAN(output_pan);
              Control_TILT(output_tilt);
          }
      }

      // ========== CAN 에러 체크 및 복구 ==========
      CAN_CheckAndRecover();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief PID 컨트롤러 초기화
 */
void PID_Init(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/**
 * @brief PID 제어 계산
 */
float PID_Compute(PID_Controller *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // Dead zone: 오차가 작으면 적분값 리셋하고 0 반환
    if (fabsf(error) < DEAD_ZONE)
    {
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        return 0.0f;
    }

    // 비례항 (P)
    float p_term = pid->Kp * error;

    // 적분항 (I) - Anti-windup 적용
    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i_term = pid->Ki * pid->integral;

    // 미분항 (D)
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->Kd * derivative;
    pid->prev_error = error;

    // PID 출력 합산
    float output = p_term + i_term + d_term;

    // 출력 제한
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

/**
 * @brief PWM 주파수 설정 (TB6560 펄스 속도 제어)
 */
void Motor_SetPWMFrequency(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz)
{
    if (freq_hz < PWM_FREQ_MIN) freq_hz = PWM_FREQ_MIN;
    if (freq_hz > PWM_FREQ_MAX) freq_hz = PWM_FREQ_MAX;

    // ARR 계산
    uint32_t psc = htim->Init.Prescaler;
    uint32_t timer_freq = TIMER_CLOCK / (psc + 1);
    uint32_t arr = (timer_freq / freq_hz) - 1;

    if (arr < 1) arr = 1;
    if (arr > 65535) arr = 65535;

    // ARR 업데이트
    __HAL_TIM_SET_AUTORELOAD(htim, arr);

    // Duty 50%로 설정 (펄스 생성)
    __HAL_TIM_SET_COMPARE(htim, channel, arr / 2);
}

/**
 * @brief 모터 정지 (PWM 출력 0)
 */
void Motor_Stop(TIM_HandleTypeDef *htim, uint32_t channel)
{
    __HAL_TIM_SET_COMPARE(htim, channel, 0);
}

/**
 * @brief PAN 모터 제어 (자동 모드)
 * @note X좌표 양수(드론이 오른쪽) → CCW, X좌표 음수(드론이 왼쪽) → CW
 *       좌표 범위: X = -960 ~ +960 (해상도 1920x1080 기준 중심으로부터의 오차)
 */
void Control_PAN(float pid_output)
{
    if (fabsf(pid_output) < DEAD_ZONE)
    {
        Motor_Stop(&htim3, TIM_CHANNEL_2);
        return;
    }

    // 방향 설정 (PA8: PAN_CW) - 방향 반전 적용
    // X좌표 양수 → pid_output 음수 → CCW (GPIO_PIN_RESET)
    // X좌표 음수 → pid_output 양수 → CW (GPIO_PIN_SET)
    if (pid_output > 0)
    {
        HAL_GPIO_WritePin(PAN_CW_GPIO_Port, PAN_CW_Pin, GPIO_PIN_SET);    // CW
    }
    else
    {
        HAL_GPIO_WritePin(PAN_CW_GPIO_Port, PAN_CW_Pin, GPIO_PIN_RESET);  // CCW
    }

    // 속도 설정
    float speed = fabsf(pid_output);
    uint32_t freq = (uint32_t)(PWM_FREQ_MIN + (speed / 1000.0f) * (PWM_FREQ_MAX - PWM_FREQ_MIN));

    Motor_SetPWMFrequency(&htim3, TIM_CHANNEL_2, freq);
}

/**
 * @brief TILT 모터 제어 (자동 모드)
 * @note Y좌표 양수(드론이 위) → CCW, Y좌표 음수(드론이 아래) → CW
 *       좌표 범위: Y = -540 ~ +540 (해상도 1920x1080 기준 중심으로부터의 오차)
 */
void Control_TILT(float pid_output)
{
    if (fabsf(pid_output) < DEAD_ZONE)
    {
        Motor_Stop(&htim4, TIM_CHANNEL_1);
        return;
    }

    // 방향 설정 (PA10: TILT_CW) - 방향 반전 적용
    // Y좌표 양수 → pid_output 음수 → CCW (GPIO_PIN_SET)
    // Y좌표 음수 → pid_output 양수 → CW (GPIO_PIN_RESET)
    if (pid_output > 0)
    {
        HAL_GPIO_WritePin(TILT_CW_GPIO_Port, TILT_CW_Pin, GPIO_PIN_RESET);  // CW
    }
    else
    {
        HAL_GPIO_WritePin(TILT_CW_GPIO_Port, TILT_CW_Pin, GPIO_PIN_SET);    // CCW
    }

    // 속도 설정
    float speed = fabsf(pid_output);
    uint32_t freq = (uint32_t)(PWM_FREQ_MIN + (speed / 1000.0f) * (PWM_FREQ_MAX - PWM_FREQ_MIN));

    Motor_SetPWMFrequency(&htim4, TIM_CHANNEL_1, freq);
}

/**
 * @brief 레이저 ON/OFF 설정
 */
void Laser_Set(uint8_t state)
{
    laser_state = state ? 1 : 0;

    // PB4: Laser, PA9: Laser_LED, PC6/PC7/PC8: 추가 레이저 3개
    if (laser_state)
    {
        HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Laser_LED_GPIO_Port, Laser_LED_Pin, GPIO_PIN_SET);
        // 추가 레이저 3개 ON (PC6, PC7, PC8)
        HAL_GPIO_WritePin(Laser1_LED_GPIO_Port, Laser1_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Laser3_LED_GPIO_Port, Laser3_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Laser2_LED_GPIO_Port, Laser2_LED_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Laser_LED_GPIO_Port, Laser_LED_Pin, GPIO_PIN_RESET);
        // 추가 레이저 3개 OFF (PC6, PC7, PC8)
        HAL_GPIO_WritePin(Laser1_LED_GPIO_Port, Laser1_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Laser3_LED_GPIO_Port, Laser3_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Laser2_LED_GPIO_Port, Laser2_LED_Pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief 레이저 상태 토글
 */
void Laser_Toggle(void)
{
    Laser_Set(!laser_state);
}

/**
 * @brief 수동 모드 - PAN 제어
 */
void Manual_ControlPAN(uint8_t left, uint8_t right)
{
    if (left && !right)
    {
        // CW (좌) - 방향 반전
        HAL_GPIO_WritePin(PAN_CW_GPIO_Port, PAN_CW_Pin, GPIO_PIN_SET);
        Motor_SetPWMFrequency(&htim3, TIM_CHANNEL_2, MANUAL_SPEED_HZ);
        manual_pan_dir = 1;
        manual_pan_timer = MANUAL_TIMEOUT_COUNT;
    }
    else if (right && !left)
    {
        // CCW (우) - 방향 반전
        HAL_GPIO_WritePin(PAN_CW_GPIO_Port, PAN_CW_Pin, GPIO_PIN_RESET);
        Motor_SetPWMFrequency(&htim3, TIM_CHANNEL_2, MANUAL_SPEED_HZ);
        manual_pan_dir = 2;
        manual_pan_timer = MANUAL_TIMEOUT_COUNT;
    }
    else
    {
        // 정지 (아무 키도 안눌림 또는 둘 다 눌림)
        Motor_Stop(&htim3, TIM_CHANNEL_2);
        manual_pan_dir = 0;
        manual_pan_timer = 0;
    }
}

/**
 * @brief 수동 모드 - TILT 제어
 * @note '상' 명령 → CCW, '하' 명령 → CW (방향 반전 적용)
 */
void Manual_ControlTILT(uint8_t up, uint8_t down)
{
    if (up && !down)
    {
        // CCW (상) - 방향 반전: GPIO_PIN_SET = CCW
        HAL_GPIO_WritePin(TILT_CW_GPIO_Port, TILT_CW_Pin, GPIO_PIN_SET);
        Motor_SetPWMFrequency(&htim4, TIM_CHANNEL_1, MANUAL_SPEED_HZ);
        manual_tilt_dir = 1;
        manual_tilt_timer = MANUAL_TIMEOUT_COUNT;
    }
    else if (down && !up)
    {
        // CW (하) - 방향 반전: GPIO_PIN_RESET = CW
        HAL_GPIO_WritePin(TILT_CW_GPIO_Port, TILT_CW_Pin, GPIO_PIN_RESET);
        Motor_SetPWMFrequency(&htim4, TIM_CHANNEL_1, MANUAL_SPEED_HZ);
        manual_tilt_dir = 2;
        manual_tilt_timer = MANUAL_TIMEOUT_COUNT;
    }
    else
    {
        // 정지
        Motor_Stop(&htim4, TIM_CHANNEL_1);
        manual_tilt_dir = 0;
        manual_tilt_timer = 0;
    }
}

/**
 * @brief CAN 수신 콜백 (ISR)
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &canRxHeader, &can1Rx0Data[0]);
        can1_rx0_flag = 1;
    }
}

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
