/*
 * driving.h
 *
 *  Created on: Jan 22, 2026
 *      Author: shiny
 */

#ifndef DRIVING_H
#define DRIVING_H

#include "main.h"

void Driving_Init(void);
// 주행 속도 설정 함수 (0~255 범위를 받아서 처리)
void Driving_SetSpeed(uint8_t l_spd, uint8_t r_spd, uint8_t l_dir, uint8_t r_dir);

// 즉시 정지 함수
void Driving_Stop(void);

#endif /* INC_DRIVING_H_ */
