/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for the can.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/**
  * @brief 하드웨어 및 통신 초기화 함수
  */
void MX_CAN1_Filter_Config(void);

/**
  * @brief Gimbal 하트비트 송신 함수 (ID: 0x702)
  */
void CAN1_Send_Gimbal_Heartbeat(uint8_t status);

/**
 * @brief [추가] 주행 명령 데이터를 가져오는 Getter 함수 (freertos.c에서 호출용)
 */
void CAN_GetDriveCommand(uint8_t *ls, uint8_t *rs, uint8_t *ld, uint8_t *rd, uint8_t *brk);

/**
  * @brief 다른 파일(freertos.c, main.c 등)과 변수를 공유하기 위한 extern 선언
  */
extern CAN_RxHeaderTypeDef canRxHeader;
extern CAN_TxHeaderTypeDef canTxHeader;
extern uint32_t            TxMailBox;
extern uint8_t             can1Rx0Data[8];
extern uint8_t             can1Tx0Data[8];

/* 실시간 제어 변수 (인터럽트 공유이므로 volatile 필수) */
extern volatile int16_t g_target_x;
extern volatile int16_t g_target_y;
extern volatile uint8_t g_control_mode;
extern volatile uint8_t g_laser_cmd;
extern volatile uint8_t last_gate_cmd;
extern volatile uint8_t dronemode;
extern volatile int16_t g_manual_up;
extern volatile int16_t g_manual_down;
extern volatile int16_t g_manual_left;
extern volatile int16_t g_manual_right;

/* 시스템 상태 감시용 변수 */
extern volatile uint32_t   g_last_heartbeat_rx_tick;
extern volatile uint32_t   g_last_rx_tick;
extern volatile uint32_t   g_last_laser_tick;
extern volatile uint32_t g_last_target_tick;

/* [추가] 주행 명령 원본 데이터 (0x100) */
extern volatile uint8_t    g_drive_cmd[5];

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

