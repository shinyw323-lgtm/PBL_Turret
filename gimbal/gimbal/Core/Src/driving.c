/*
 * driving.c
 *
 *  Created on: Jan 22, 2026
 *      Author: shiny
 */


#include "driving.h"
#include "tim.h"

#define PWM_MAX 4199u
#define CAN_PWM_MAX 255u

void Driving_Init(void) {
    // 1. PWM 시작 (이 시점까지는 핀이 Idle 상태)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // 듀티비를 확실히 0으로 초기화
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

    // 2. 드라이버 EN 활성화 (이제야 모터에 전기가 흐를 준비가 됨)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
}
// 0~255 속도를 0~4199 PWM 수치로 변환
static uint32_t scale_speed(uint8_t v) {
    return ((uint32_t)v * PWM_MAX) / CAN_PWM_MAX;
}

void Driving_SetSpeed(uint8_t l_spd, uint8_t r_spd, uint8_t l_dir, uint8_t r_dir) {
    uint32_t lp = scale_speed(l_spd);
    uint32_t rp = scale_speed(r_spd);

    // 왼쪽 모터 제어 (CH1, CH2)
    if (l_dir == 0) { // 전진
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, lp);
    } else { // 후진
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, lp);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }

    // 오른쪽 모터 제어 (CH3, CH4)
    if (r_dir == 0) { // 전진
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, rp);
    } else { // 후진
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, rp);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}

void Driving_Stop(void) {
    Driving_SetSpeed(0, 0, 0, 0);
}
