/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   Gimbal CAN Communication Header
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

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
extern uint8_t              can1_rx0_flag;
extern CAN_FilterTypeDef    canFilter;
extern CAN_RxHeaderTypeDef  canRxHeader;
extern CAN_TxHeaderTypeDef  canTxHeader;
extern uint32_t             TxMailBox;
extern uint8_t              can1Rx0Data[8];
extern uint8_t              can1Tx0Data[8];
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/*============================================================================
 * CAN 초기화/설정
 *============================================================================*/
/**
 * @brief Gimbal용 CAN 필터 설정 (0x101, 0x110, 0x111, 0x120)
 */
void MX_CAN1_Filter_Gimbal(void);

/*============================================================================
 * CAN 수신 처리
 *============================================================================*/
/**
 * @brief CAN 수신 메시지 처리 (메인 루프에서 호출)
 */
void CAN_HandleRxMessage(void);

/**
 * @brief 수동 제어 명령 가져오기
 * @param up    상 방향 (1: on, 0: off)
 * @param down  하 방향
 * @param left  좌 방향
 * @param right 우 방향
 * @return 새 데이터가 있으면 1, 없으면 0
 */
uint8_t CAN_GetManualCmd(uint8_t *up, uint8_t *down, uint8_t *left, uint8_t *right);

/**
 * @brief 자동 제어 명령 가져오기 (PID용 오차값)
 * @param error_x X축 오차 (pixel, signed)
 * @param error_y Y축 오차 (pixel, signed)
 * @return 새 데이터가 있으면 1, 없으면 0
 */
uint8_t CAN_GetAutoCmd(int16_t *error_x, int16_t *error_y);

/**
 * @brief 모드 토글 플래그 확인 및 소비
 * @return 토글 요청이 있으면 1, 없으면 0
 */
uint8_t CAN_ConsumeModeToggle(void);

/**
 * @brief 레이저 명령 가져오기
 * @param laser_on 레이저 상태 (0: OFF, 1: ON)
 * @return 새 명령이 있으면 1, 없으면 0
 */
uint8_t CAN_GetLaserCmd(uint8_t *laser_on);

/*============================================================================
 * CAN 송신
 *============================================================================*/
/**
 * @brief Gimbal 하트비트 송신 (500ms 주기로 호출)
 */
void CAN_SendGimbalHeartbeat(void);

/**
 * @brief Gimbal 상태 설정 (하트비트에 반영)
 * @param status 0: 정상, 1: 비정상
 */
void CAN_SetGimbalStatus(uint8_t status);

/*============================================================================
 * CAN 에러 처리
 *============================================================================*/
/**
 * @brief CAN 버스 에러 확인 및 복구 (메인 루프에서 주기적 호출)
 */
void CAN_CheckAndRecover(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

