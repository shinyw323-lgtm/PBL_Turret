/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   Gimbal CAN Communication Module
  *          - Receives: 0x101 (Manual), 0x110 (Auto), 0x111 (Mode), 0x120 (Laser)
  *          - Transmits: 0x702 (Heartbeat)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>

/*============================================================================
 * CAN IDs (ICD 기반)
 *============================================================================*/
#define CAN_ID_TURRET_MANUAL    0x101   // Gateway → Gimbal: 수동 제어 (30ms)
#define CAN_ID_TURRET_AUTO      0x110   // Jetson → Gimbal: 자동 제어 (30ms)
#define CAN_ID_MODE_SWITCH      0x111   // Gateway → Gimbal: 모드 전환 (이벤트)
#define CAN_ID_LASER_TOGGLE     0x120   // Gateway → Gimbal: 레이저 전환 (이벤트)
#define CAN_ID_GIMBAL_HEARTBEAT 0x702   // Gimbal → Gateway: 하트비트 (500ms)

/*============================================================================
 * CAN 통신 변수
 *============================================================================*/
uint8_t               can1_rx0_flag = 0;
CAN_FilterTypeDef     canFilter;
CAN_RxHeaderTypeDef   canRxHeader;
CAN_TxHeaderTypeDef   canTxHeader;
uint32_t              TxMailBox;
uint8_t               can1Rx0Data[8];
uint8_t               can1Tx0Data[8];

/*============================================================================
 * 수신 데이터 구조체 (Race Condition 방지를 위한 더블 버퍼링)
 *============================================================================*/
// 수동 제어 명령 (0x101)
typedef struct {
    uint8_t up;      // 상 (1: on, 0: off)
    uint8_t down;    // 하 (1: on, 0: off)
    uint8_t left;    // 좌 (1: on, 0: off)
    uint8_t right;   // 우 (1: on, 0: off)
} ManualCmd_t;

// 자동 제어 명령 (0x110)
typedef struct {
    int16_t error_x;   // X축 오차 (pixel, signed)
    int16_t error_y;   // Y축 오차 (pixel, signed)
} AutoCmd_t;

// ISR에서 쓰는 버퍼
static volatile ManualCmd_t manual_cmd_isr = {0};
static volatile AutoCmd_t   auto_cmd_isr = {0};
static volatile uint8_t     mode_toggle_flag = 0;      // 모드 토글 요청 플래그
static volatile uint8_t     laser_cmd_value = 0;       // 레이저 명령값 (0: OFF, 1: ON)
static volatile uint8_t     new_laser_cmd = 0;         // 레이저 명령 수신 플래그

// 메인 루프에서 읽는 버퍼 (복사본)
static ManualCmd_t manual_cmd_main = {0};
static AutoCmd_t   auto_cmd_main = {0};

// 새 데이터 도착 플래그
static volatile uint8_t new_manual_data = 0;
static volatile uint8_t new_auto_data = 0;

// Gimbal 상태 (하트비트용)
static volatile uint8_t gimbal_status = 0;  // 0: 정상, 1: 비정상

// CAN 에러 카운터 (버스 오류 복구용)
static volatile uint32_t can_error_count = 0;
static volatile uint32_t can_last_rx_tick = 0;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;      // ★ Bus-Off 자동 복구 활성화
  hcan1.Init.AutoWakeUp = ENABLE;      // ★ 자동 Wake-up 활성화
  hcan1.Init.AutoRetransmission = ENABLE;  // ★ 전송 실패 시 자동 재전송
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */
    // CAN SCE (Status Change Error) 인터럽트 활성화 - Bus-Off, Error 복구용
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/*============================================================================
 * CAN 필터 설정 - Gimbal용 (0x101, 0x110, 0x111, 0x120 수신)
 *============================================================================*/
void MX_CAN1_Filter_Gimbal(void)
{
    canFilter.FilterMode = CAN_FILTERMODE_IDLIST;
    canFilter.FilterScale = CAN_FILTERSCALE_16BIT;
    canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canFilter.FilterActivation = ENABLE;

    // Bank0: 0x101 (수동), 0x110 (자동), 0x111 (모드), 0x120 (레이저)
    canFilter.FilterBank = 0;
    canFilter.FilterIdHigh      = (CAN_ID_TURRET_MANUAL << 5);  // 0x101
    canFilter.FilterIdLow       = (CAN_ID_TURRET_AUTO << 5);    // 0x110
    canFilter.FilterMaskIdHigh  = (CAN_ID_MODE_SWITCH << 5);    // 0x111
    canFilter.FilterMaskIdLow   = (CAN_ID_LASER_TOGGLE << 5);   // 0x120
    HAL_CAN_ConfigFilter(&hcan1, &canFilter);

    // CAN RX 인터럽트 활성화 (수신 + FIFO 오버런)
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | 
                                          CAN_IT_RX_FIFO0_FULL | 
                                          CAN_IT_RX_FIFO0_OVERRUN);
    
    // CAN 에러 인터럽트 활성화 (Bus-Off, Error Warning, Error Passive)
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | 
                                          CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE |
                                          CAN_IT_ERROR);

    printf("[CAN] Gimbal filter configured (0x101, 0x110, 0x111, 0x120)\r\n");
}

/*============================================================================
 * CAN 프레임 송신 (재시도 로직 포함)
 *============================================================================*/
static HAL_StatusTypeDef CAN_SendStdFrame(uint16_t std_id, uint8_t dlc, uint8_t *payload)
{
    canTxHeader.StdId = std_id;
    canTxHeader.RTR   = CAN_RTR_DATA;
    canTxHeader.IDE   = CAN_ID_STD;
    canTxHeader.DLC   = dlc;

    // 3회 재시도
    for (uint8_t retry = 0; retry < 3; retry++)
    {
        // 메일박스가 가득 찼으면 잠시 대기
        uint32_t free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
        if (free_level == 0)
        {
            HAL_Delay(1);
            continue;
        }

        if (HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, payload, &TxMailBox) == HAL_OK)
        {
            printf("[CAN TX] 0x%03X [%d] %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                   std_id, dlc,
                   payload[0], payload[1], payload[2], payload[3],
                   payload[4], payload[5], payload[6], payload[7]);
            return HAL_OK;
        }
    }

    can_error_count++;
    printf("[CAN TX ERR] 0x%03X failed (err_cnt=%lu)\r\n", std_id, can_error_count);
    return HAL_ERROR;
}

/*============================================================================
 * Gimbal 하트비트 송신 (500ms 주기)
 *============================================================================*/
void CAN_SendGimbalHeartbeat(void)
{
    uint8_t frame[8] = {0};
    frame[0] = gimbal_status;  // 0: 정상, 1: 비정상
    CAN_SendStdFrame(CAN_ID_GIMBAL_HEARTBEAT, 8, frame);
}

/*============================================================================
 * Gimbal 상태 설정 (하트비트에 반영됨)
 *============================================================================*/
void CAN_SetGimbalStatus(uint8_t status)
{
    gimbal_status = status;
}

/*============================================================================
 * CAN 수신 메시지 처리 (메인 루프에서 호출)
 * ★ Race Condition 방지: 인터럽트 비활성화 후 데이터 복사
 *============================================================================*/
void CAN_HandleRxMessage(void)
{
    if (can1_rx0_flag == 0) return;

    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    // ★ 임계 영역: 인터럽트 비활성화 후 데이터 복사
    __disable_irq();
    can1_rx0_flag = 0;
    header = canRxHeader;
    memcpy(data, can1Rx0Data, sizeof(data));
    __enable_irq();

    // 표준 ID만 처리
    if (header.IDE != CAN_ID_STD)
    {
        printf("[CAN RX] EXT ID 0x%08lX ignored\r\n", header.ExtId);
        return;
    }

    // 수신 시간 기록 (타임아웃 감지용)
    can_last_rx_tick = HAL_GetTick();

    printf("[CAN RX] 0x%03lX [%lu] %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
           header.StdId, (unsigned long)header.DLC,
           data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);

    switch (header.StdId)
    {
    case CAN_ID_TURRET_MANUAL:  // 0x101: 수동 제어
        __disable_irq();
        manual_cmd_isr.up    = data[0];
        manual_cmd_isr.down  = data[1];
        manual_cmd_isr.left  = data[2];
        manual_cmd_isr.right = data[3];
        new_manual_data = 1;
        __enable_irq();
        printf("  -> Manual: Up=%d Down=%d Left=%d Right=%d\r\n",
               data[0], data[1], data[2], data[3]);
        break;

    case CAN_ID_TURRET_AUTO:  // 0x110: 자동 제어
    {
        int16_t err_x = (int16_t)((data[0] << 8) | data[1]);
        int16_t err_y = (int16_t)((data[2] << 8) | data[3]);
        __disable_irq();
        auto_cmd_isr.error_x = err_x;
        auto_cmd_isr.error_y = err_y;
        new_auto_data = 1;
        __enable_irq();
        printf("  -> Auto: X_err=%d, Y_err=%d\r\n", err_x, err_y);
        break;
    }

    case CAN_ID_MODE_SWITCH:  // 0x111: 모드 전환
        if (data[0] == 1)
        {
            __disable_irq();
            mode_toggle_flag = 1;
            __enable_irq();
            printf("  -> Mode Toggle Request\r\n");
        }
        break;

    case CAN_ID_LASER_TOGGLE:  // 0x120: 레이저 ON/OFF 명령
        __disable_irq();
        laser_cmd_value = data[0] ? 1 : 0;  // 0: OFF, 1: ON
        new_laser_cmd = 1;
        __enable_irq();
        printf("  -> Laser Command: %s\r\n", data[0] ? "ON" : "OFF");
        break;

    default:
        printf("  -> Unknown ID\r\n");
        break;
    }
}

/*============================================================================
 * 수동 제어 명령 가져오기 (메인 루프에서 호출)
 *============================================================================*/
uint8_t CAN_GetManualCmd(uint8_t *up, uint8_t *down, uint8_t *left, uint8_t *right)
{
    uint8_t has_data = 0;

    __disable_irq();
    if (new_manual_data)
    {
        new_manual_data = 0;
        manual_cmd_main = manual_cmd_isr;
        has_data = 1;
    }
    __enable_irq();

    if (has_data)
    {
        *up    = manual_cmd_main.up;
        *down  = manual_cmd_main.down;
        *left  = manual_cmd_main.left;
        *right = manual_cmd_main.right;
    }

    return has_data;
}

/*============================================================================
 * 자동 제어 명령 가져오기 (메인 루프에서 호출)
 *============================================================================*/
uint8_t CAN_GetAutoCmd(int16_t *error_x, int16_t *error_y)
{
    uint8_t has_data = 0;

    __disable_irq();
    if (new_auto_data)
    {
        new_auto_data = 0;
        auto_cmd_main = auto_cmd_isr;
        has_data = 1;
    }
    __enable_irq();

    if (has_data)
    {
        *error_x = auto_cmd_main.error_x;
        *error_y = auto_cmd_main.error_y;
    }

    return has_data;
}

/*============================================================================
 * 모드 토글 플래그 확인 및 소비
 *============================================================================*/
uint8_t CAN_ConsumeModeToggle(void)
{
    uint8_t flag = 0;
    __disable_irq();
    if (mode_toggle_flag)
    {
        mode_toggle_flag = 0;
        flag = 1;
    }
    __enable_irq();
    return flag;
}

/*============================================================================
 * 레이저 명령 가져오기 (0: OFF, 1: ON)
 *============================================================================*/
uint8_t CAN_GetLaserCmd(uint8_t *laser_on)
{
    uint8_t has_cmd = 0;
    __disable_irq();
    if (new_laser_cmd)
    {
        new_laser_cmd = 0;
        *laser_on = laser_cmd_value;
        has_cmd = 1;
    }
    __enable_irq();
    return has_cmd;
}

/*============================================================================
 * CAN 복구 요청 플래그 (ISR에서 설정, 메인 루프에서 처리)
 *============================================================================*/
static volatile uint8_t can_recovery_needed = 0;

/*============================================================================
 * CAN 하드웨어 재초기화 (실제 복구 수행)
 *============================================================================*/
static void CAN_DoRecovery(void)
{
    printf("[CAN] Performing recovery...\r\n");
    
    // CAN 정지
    HAL_CAN_Stop(&hcan1);
    
    // 잠시 대기 (버스 안정화)
    HAL_Delay(10);
    
    // CAN 재초기화
    HAL_CAN_DeInit(&hcan1);
    HAL_Delay(5);
    
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        printf("[CAN ERR] Re-init failed!\r\n");
        return;
    }
    
    // 필터 재설정 (인터럽트도 재활성화됨)
    MX_CAN1_Filter_Gimbal();
    
    // CAN 재시작
    if (HAL_CAN_Start(&hcan1) == HAL_OK)
    {
        gimbal_status = 0;  // 정상 상태로 복귀
        printf("[CAN] Recovery successful!\r\n");
    }
    else
    {
        printf("[CAN ERR] Restart failed!\r\n");
    }
}

/*============================================================================
 * CAN 버스 에러 확인 및 복구 (메인 루프에서 호출)
 *============================================================================*/
void CAN_CheckAndRecover(void)
{
    // ISR에서 복구 요청이 있으면 수행
    if (can_recovery_needed)
    {
        can_recovery_needed = 0;
        CAN_DoRecovery();
        return;
    }

    uint32_t error = HAL_CAN_GetError(&hcan1);

    if (error != HAL_CAN_ERROR_NONE)
    {
        can_error_count++;
        printf("[CAN ERR] Error=0x%08lX, Count=%lu\r\n", error, can_error_count);

        // Bus-Off 또는 심각한 에러 시 복구
        if (error & (HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_EWG))
        {
            CAN_DoRecovery();
        }
        else
        {
            // 에러 플래그만 클리어
            __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_ERRI);
        }
    }
}

/*============================================================================
 * CAN 에러 콜백 (HAL 콜백 - ISR 컨텍스트)
 *============================================================================*/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        can_error_count++;
        
        uint32_t error = HAL_CAN_GetError(hcan);
        
        // Bus-Off, Error Passive, Error Warning 발생 시 복구 요청
        if (error & (HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_EWG))
        {
            gimbal_status = 1;  // 비정상 상태로 설정
            can_recovery_needed = 1;  // 메인 루프에서 복구 수행 요청
        }
        
        // 에러 플래그 클리어
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_ERRI);
    }
}

/*============================================================================
 * CAN FIFO0 Full 콜백 (HAL 콜백 - ISR 컨텍스트)
 *============================================================================*/
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        // FIFO가 가득 찼을 때 - 메시지를 읽어서 비워줌
        CAN_RxHeaderTypeDef tempHeader;
        uint8_t tempData[8];
        
        // FIFO에 있는 모든 메시지 읽기
        while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &tempHeader, tempData);
            
            // 마지막 메시지만 플래그로 처리 (가장 최신 데이터)
            canRxHeader = tempHeader;
            for (int i = 0; i < 8; i++) can1Rx0Data[i] = tempData[i];
            can1_rx0_flag = 1;
        }
    }
}

/* USER CODE END 1 */
