#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_
#include "main.h"
#include "stdbool.h"

struct keys
{
	uint8_t key_judage;
	uint8_t key_time;
	bool single_flag;
	bool long_flag;
	bool key_sta;
};

extern uint8_t hour_high;
extern uint8_t hour_low;
extern uint8_t minute_high;
extern uint8_t minute_low;
extern uint8_t second_high;
extern uint8_t second_low;

extern uint8_t hour_high1;
extern uint8_t hour_low1;
extern uint8_t minute_high1;
extern uint8_t minute_low1;
extern uint8_t second_high1;
extern uint8_t second_low1;
extern bool up_flag;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif

