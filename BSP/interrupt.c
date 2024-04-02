#include "interrupt.h"
#include "usart.h"
#include "string.h"
struct keys key[4]={0,0,0,0};
bool TIM2_flag=0;
bool TIM1_flag=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		key[0].key_sta = HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_0);
		key[1].key_sta = HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_1);
		key[2].key_sta = HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_2);
		key[3].key_sta = HAL_GPIO_ReadPin(GPIOA , GPIO_PIN_0);
		
		for(int i=0;i<4;i++){
			switch(key[i].key_judage){
				case 0:{
						if(key[i].key_sta == 0){
							key[i].key_judage = 1;
							key[i].key_time = 0;
						}
				}break;
				case 1:{
					if(key[i].key_sta == 1){
						key[i].key_judage = 0;
					}else{
						key[i].key_judage = 2;
					}
				}break;
				case 2:{
					if(key[i].key_sta == 1){
						key[i].key_judage = 0;
						if(key[i].key_time<=80){
							key[i].single_flag = 1;
						}
					}else{
						key[i].key_time++;
						if(key[i].key_time>80){
							key[i].long_flag = 1;
							key[i].key_judage = 0;
						}
					}
				}break;
			}
		}
	}
	
	if(htim->Instance == TIM2)
	{
		
		TIM2_flag = 1;
	}
	
	if(htim->Instance == TIM1)
	{
		TIM1_flag=!TIM1_flag;
	}
}

uint8_t rx_buffer[30];
bool uart_flag=0;
uint8_t rx_data;
uint8_t rx_pointer=0;
uint8_t uart_mode=1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance == USART1)
		{
			HAL_UART_Receive_IT(&huart1 , &rx_data , 1);
			rx_buffer[rx_pointer++] = rx_data;
			if(rx_buffer[0]=='k')
			{
				uart_mode=0;
			}else{
				uart_mode=1;
			}
			
			if(uart_mode ==0)
			{
				if(rx_pointer == 5)	uart_flag = 1;
			}else if(uart_mode == 1)
			{
				if(rx_pointer == 16)	uart_flag = 1;
			}
			
		}


}




