/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern void CAN_CheckAndRecover(void);
extern void CAN1_Send_Gimbal_Heartbeat(uint8_t status);
extern void MX_CAN1_Filter_Config(void);
extern void Driving_Stop(void);
extern void Driving_Init(void);
extern void Driving_SetSpeed(uint8_t ls, uint8_t rs, uint8_t ld, uint8_t rd);
extern void CAN_GetDriveCommand(uint8_t *ls, uint8_t *rs, uint8_t *ld, uint8_t *rd, uint8_t *brk);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern volatile int16_t    g_target_x;
extern volatile int16_t    g_target_y;
extern volatile uint8_t    g_control_mode;
extern volatile uint8_t    g_laser_cmd;
extern volatile uint8_t    dronemode;

extern volatile uint32_t   g_last_rx_tick;
extern volatile uint32_t   g_last_target_tick;
extern volatile uint32_t   g_last_laser_tick;

extern volatile int16_t    g_manual_up;
extern volatile int16_t    g_manual_down;
extern volatile int16_t    g_manual_left;
extern volatile int16_t    g_manual_right;

osThreadId GimbalTASKHandle;
osThreadId MonitorTaskHandle;
osThreadId ActionTaskHandle;
/* USER CODE END Variables */
osThreadId GimbalTASKHandle;
osThreadId MonitorTaskHandle;
osThreadId ActionTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint16_t Calculate_Mavlink_CRC(uint8_t *data, uint16_t len);
uint16_t crc_accumulate(uint8_t b, uint16_t crc);
/* USER CODE END FunctionPrototypes */

void StartGimbalTask(void const * argument);
void StartMonitorTask(void const * argument);
void StartActionTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
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
  /* definition and creation of GimbalTASK */
  osThreadDef(GimbalTASK, StartGimbalTask, osPriorityHigh, 0, 512);
  GimbalTASKHandle = osThreadCreate(osThread(GimbalTASK), NULL);

  /* definition and creation of MonitorTask */
  osThreadDef(MonitorTask, StartMonitorTask, osPriorityRealtime, 0, 512);
  MonitorTaskHandle = osThreadCreate(osThread(MonitorTask), NULL);

  /* definition and creation of ActionTask */
  osThreadDef(ActionTask, StartActionTask, osPriorityNormal, 0, 256);
  ActionTaskHandle = osThreadCreate(osThread(ActionTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
  * @brief  Function implementing the GimbalTASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGimbalTask */

  /* USER CODE BEGIN StartGimbalTask */
void StartGimbalTask(void const * argument) {
    static float current_pitch = 0.0f;
    static float current_yaw   = 0.0f;
    uint8_t storm_cmd[19];
    uint8_t rx_ack[6];
    char debug_buf[128];

    for (;;) {
        // 1. 제어 로직
        if (g_control_mode == 1) { // AUTO
            current_yaw   = (float)g_target_x * 0.1f;
            current_pitch = (float)g_target_y * 0.1f;
        } else { // MANUAL
            if (g_manual_up)    current_pitch -= 2.0f; //업이지만 다운
            if (g_manual_down)  current_pitch += 2.0f; //다운이지만 위로
            if (g_manual_left)  current_yaw   += 2.0f;
            if (g_manual_right) current_yaw   -= 2.0f;
        }

        // 2. STorM32 패킷 조립
        storm_cmd[0] = 0xFA;
        storm_cmd[1] = 0x0E;
        storm_cmd[2] = 0x11;
        float roll_zero = 0.0f;
        memcpy(&storm_cmd[3],  &current_pitch, 4);
        memcpy(&storm_cmd[7],  &roll_zero, 4);
        memcpy(&storm_cmd[11], &current_yaw, 4);
        storm_cmd[15] = 0x00;
        storm_cmd[16] = 0x00;
        uint16_t crc = Calculate_Mavlink_CRC(&storm_cmd[1], 16);
        storm_cmd[17] = (uint8_t)(crc & 0xFF);
        storm_cmd[18] = (uint8_t)(crc >> 8);

        // 3. 전송 및 응답 확인
        HAL_UART_Transmit(&huart3, storm_cmd, 19, 20);
        const char* ack_res = (HAL_UART_Receive(&huart3, rx_ack, 6, 10) == HAL_OK) ? "OK" : "NR";

        // 4. 안전한 디버그 출력 (snprintf 사용으로 메모리 오염 방지)
        int msg_len = snprintf(debug_buf, sizeof(debug_buf), "P:%.1f Y:%.1f ACK:%s\r\n",
                               current_pitch, current_yaw, ack_res);
        HAL_UART_Transmit(&huart2, (uint8_t*)debug_buf, msg_len, 10);

        osDelay(33);
    }
}
  /* USER CODE END StartGimbalTask */


/* USER CODE BEGIN Header_StartMonitorTask */
/**
* @brief Function implementing the MonitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorTask */

  /* USER CODE BEGIN StartMonitorTask */
void StartMonitorTask(void const * argument) {
    MX_CAN1_Filter_Config();
    Driving_Stop();

    uint8_t last_peer_alive = 1;
    uint32_t last_hb_tick = osKernelSysTick();

    for (;;) {
    	CAN_CheckAndRecover();
        uint32_t current_tick = osKernelSysTick();

        // 1. Peer 생존 확인 (1초 타임아웃)
        uint8_t current_peer_alive = (current_tick - g_last_rx_tick < 1000) ? 1 : 0;

        if (!current_peer_alive) {
            if (last_peer_alive == 1) {
                Driving_Init();
                HAL_UART_Transmit(&huart2, (uint8_t*)"[FAIL-OVER] Take Control!\r\n", 27, 10);
            }
            uint8_t l_spd, r_spd, l_dir, r_dir, brake;
            CAN_GetDriveCommand(&l_spd, &r_spd, &l_dir, &r_dir, &brake);

            if (brake == 1) Driving_Stop();
            else Driving_SetSpeed(l_spd, r_spd, l_dir, r_dir);
        }

        // 2. 정확한 500ms 하트비트 전송
        if (current_tick - last_hb_tick >= 500) {
            CAN1_Send_Gimbal_Heartbeat(0);
            last_hb_tick = current_tick;
        }

        last_peer_alive = current_peer_alive;
        osDelay(10);
    }
}
  /* USER CODE END StartMonitorTask */


/* USER CODE BEGIN Header_StartActionTask */
/**
* @brief Function implementing the ActionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartActionTask */

  /* USER CODE BEGIN StartActionTask */
void StartActionTask(void const * argument) {
    for (;;) {
        // 자동 모드 유효성 체크 (150ms)
        if (osKernelSysTick() - g_last_target_tick > 150) {
            dronemode = 0;
        }

        // 레이저 제어
        if (dronemode == 1 || g_laser_cmd == 1) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
        }

        osDelay(50);
    }
}
  /* USER CODE END StartActionTask */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
uint16_t crc_accumulate(uint8_t b, uint16_t crc) {
    uint8_t ch = (uint8_t)(b ^ (uint8_t)(crc & 0x00ff));
    ch = (uint8_t)(ch ^ (uint8_t)(ch << 4));
    return (uint16_t)((crc >> 8) ^ (uint16_t)(ch << 8) ^ (uint16_t)(ch << 3) ^ (uint16_t)(ch >> 4));
}

uint16_t Calculate_Mavlink_CRC(uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF; // X25 초기값
    for (uint16_t i = 0; i < len; i++) {
        crc = crc_accumulate(data[i], crc);
    }
    return crc;
}
/* USER CODE END Application */
