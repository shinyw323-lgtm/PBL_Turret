/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"

// --- 에러 복구 및 모니터링 변수 ---
volatile uint32_t can_error_count = 0;
volatile uint8_t  can_bus_off_flag = 0;
volatile uint32_t can_recovery_tick = 0;
#define CAN_RECOVERY_DELAY_MS 1000

// 외부 변수 선언
extern volatile uint8_t  g_drive_cmd[5];
extern volatile uint8_t  g_laser_cmd;
extern volatile uint8_t  g_control_mode;
extern volatile uint8_t  dronemode;
extern volatile int16_t  g_target_x;
extern volatile int16_t  g_target_y;

extern volatile uint32_t g_last_rx_tick;
extern volatile uint32_t g_last_target_tick;
extern volatile uint32_t g_last_heartbeat_rx_tick;
extern volatile uint32_t g_last_laser_tick;

extern volatile int16_t  g_manual_up;
extern volatile int16_t  g_manual_down;
extern volatile int16_t  g_manual_left;
extern volatile int16_t  g_manual_right;
extern UART_HandleTypeDef huart2;

/* USER CODE END 0 */

// 링커 에러 해결의 핵심: hcan1의 실제 메모리 공간 정의
CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

// MSP(Low Level) 설정 - CubeMX 기본 코드
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief CAN 필터 설정 및 통신 시작
 */
void MX_CAN1_Filter_Config(void) {
    CAN_FilterTypeDef canFilter;

    canFilter.FilterBank = 0;
    canFilter.FilterMode = CAN_FILTERMODE_IDLIST;
    canFilter.FilterScale = CAN_FILTERSCALE_16BIT;
    canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canFilter.FilterActivation = ENABLE;

    canFilter.FilterIdHigh = 0x101 << 5;
    canFilter.FilterIdLow = 0x110 << 5;
    canFilter.FilterMaskIdHigh = 0x111 << 5;
    canFilter.FilterMaskIdLow = 0x120 << 5;
    HAL_CAN_ConfigFilter(&hcan1, &canFilter);

    canFilter.FilterBank = 1;
    canFilter.FilterIdHigh = 0x701 << 5;
    canFilter.FilterIdLow = 0x100 << 5;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow = 0x0000;
    HAL_CAN_ConfigFilter(&hcan1, &canFilter);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF);
}

/**
 * @brief 짐벌 하트비트 전송
 */
void CAN1_Send_Gimbal_Heartbeat(uint8_t status) {
    CAN_TxHeaderTypeDef canTxHeader;
    uint8_t can1Tx0Data[8] = {0,};
    uint32_t TxMailBox;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        SET_BIT(hcan1.Instance->TSR, CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2);
        return;
    }

    canTxHeader.StdId = 0x702;
    canTxHeader.RTR = CAN_RTR_DATA;
    canTxHeader.IDE = CAN_ID_STD;
    canTxHeader.DLC = 8;
    can1Tx0Data[0] = status;

    HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, can1Tx0Data, &TxMailBox);
}

/**
 * @brief CAN 수신 콜백
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef canRxHeader;
    uint8_t can1Rx0Data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxHeader, can1Rx0Data) == HAL_OK) {
        // 1. 구동 명령 (모터)
        if (canRxHeader.StdId == 0x100) {
            memcpy((void*)g_drive_cmd, can1Rx0Data, 5);
        }
        // 2. 레이저 제어
        else if (canRxHeader.StdId == 0x120) {
            g_laser_cmd = can1Rx0Data[0];
            g_last_laser_tick = HAL_GetTick(); // HAL_GetTick이 ISR에서 더 안전함
        }
        // 3. 컨트롤 모드 전환 (Manual/Auto)
        else if (canRxHeader.StdId == 0x111) {
            g_control_mode = can1Rx0Data[0];
            // 모드 변경 시 즉시 반영 (옵션)
            dronemode = g_control_mode;
        }
        // 4. 각도 제어 데이터 (수동/자동 분기)
        else if (canRxHeader.StdId == 0x101 && g_control_mode == 0) { // MANUAL
            g_manual_up    = can1Rx0Data[0];
            g_manual_down  = can1Rx0Data[1];
            g_manual_left  = can1Rx0Data[2];
            g_manual_right = can1Rx0Data[3];
            dronemode = 0;
        }
        else if (canRxHeader.StdId == 0x110 && g_control_mode == 1) { // AUTO
            g_target_x = (int16_t)((can1Rx0Data[0] << 8) | can1Rx0Data[1]);
            g_target_y = (int16_t)((can1Rx0Data[2] << 8) | can1Rx0Data[3]);
            g_last_target_tick = HAL_GetTick();
            dronemode = 1;
        }
        // 5. 하트비트 체크
        else if (canRxHeader.StdId == 0x701) {
            g_last_rx_tick = HAL_GetTick();
        }
    }
}

/**
 * @brief 주행 명령 데이터 반환 (freertos.c 에러 해결)
 */
void CAN_GetDriveCommand(uint8_t *ls, uint8_t *rs, uint8_t *ld, uint8_t *rd, uint8_t *brk) {
    *ls = g_drive_cmd[0];
    *rs = g_drive_cmd[1];
    *ld = g_drive_cmd[2];
    *rd = g_drive_cmd[3];
    *brk = g_drive_cmd[4];
}

/**
 * @brief CAN 에러 콜백
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance != CAN1) return;

    uint32_t error = HAL_CAN_GetError(hcan);
    can_error_count++;

    if (error & HAL_CAN_ERROR_BOF) {
        can_bus_off_flag = 1;
        can_recovery_tick = osKernelSysTick();

        char msg[64];
        uint32_t esr = hcan->Instance->ESR;
        snprintf(msg, sizeof(msg), "[CAN] BUS-OFF! TEC:%lu REC:%lu\r\n", (esr>>16)&0xFF, (esr>>24)&0xFF);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
    }
}

/**
 * @brief 자동 복구 함수
 */
void CAN_CheckAndRecover(void) {
    if (can_bus_off_flag && (osKernelSysTick() - can_recovery_tick >= CAN_RECOVERY_DELAY_MS)) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[CAN] Recovery Attempt...\r\n", 27, 10);

        HAL_CAN_Stop(&hcan1);
        HAL_CAN_DeInit(&hcan1);
        osDelay(10);

        MX_CAN1_Init();
        MX_CAN1_Filter_Config();

        can_bus_off_flag = 0;
        HAL_UART_Transmit(&huart2, (uint8_t*)"[CAN] Recovery OK\r\n", 19, 10);
    }
}
/* USER CODE END 1 */
