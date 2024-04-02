/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "i2c_hal.h"
#include "interrupt.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "badc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void view_pro(void);
void key_control(void);
void clock(void);
void uart_show(void);
void upData(void);
void led_control(void);
uint8_t eeprom_read(uint8_t addr);
void eeprom_write(uint8_t addr,uint8_t dat);	
float readFloatFromEEPROM(uint8_t startAddress);
void E2P_Read(u8 *string, u16 add, u16 num);
void E2P_Write(u8 *string, u16 add, u16 num);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool view=0;
uint8_t hour_high=2;
uint8_t hour_low=3;
uint8_t minute_high=5;
uint8_t minute_low=9;
uint8_t second_high=5;
uint8_t second_low=5;

uint8_t hour_high1=0;
uint8_t hour_low1=0;
uint8_t minute_high1=0;
uint8_t minute_low1=0;
uint8_t second_high1=0;
uint8_t second_low1=0;
bool LED_mode=1;
extern struct keys key[4];
extern bool TIM2_flag;
extern uint8_t rx_buffer[30];
extern bool uart_flag;
extern uint8_t rx_data;
extern uint8_t rx_pointer;
extern uint8_t uart_mode;
extern bool TIM1_flag;
float k=0.1;
double V1;
uint8_t clock_setting=0;
bool up_flag=0;
bool up_fflag=0;
bool LED_show;


typedef union {
				float f;
				uint8_t b[4];
		} FloatUnion;

		// ??????EEPROM
		void WriteFloatToEEPROM(uint16_t addr, float value) 
		{
				FloatUnion fu;
				fu.f = value;
				for (int i = 0; i < sizeof(fu.b); i++) {
						eeprom_write(addr + i, fu.b[i]);
				}
		}

		// ?EEPROM?????
		float ReadFloatFromEEPROM(uint16_t addr)
		{
				FloatUnion fu;
				for (int i = 0; i < sizeof(fu.b); i++) {
						fu.b[i] = eeprom_read(addr + i);
				}
				return fu.f;
		}

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	I2CInit();
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	HAL_TIM_Base_Init(&htim3);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_IT(&huart1 , &rx_data , 1);
	HAL_TIM_Base_Init(&htim1);
	HAL_TIM_Base_Start_IT(&htim1);
	
//	float myFloat = 1.23f;
//	u8 *p = (u8*)&myFloat;
//	u16 length = sizeof(myFloat);

//	E2P_Write(p, 0, length); // ?????EEPROM???0????
//	float k_read; // ??????????
//	E2P_Read((uint8_t*)&k_read, 0, sizeof(k_read));

	//WriteFloatToEEPROM(0,5);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	
    /* USER CODE BEGIN 3 */
		
		//char text[10];
		//sprintf(text,"%f\n",readFloatFromEEPROM(0));
		//HAL_UART_Transmit(&huart1 , (uint8_t*)text , strlen(text) , 0xff);
		
		upData();
		key_control();
		view_pro();
		clock();
		uart_show();
		led_control();
		//HAL_UART_Transmit(&huart1 , (uint8_t*)"OK\r\n" , strlen("OK\r\n") , 0xff);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void view_pro()
{
	if(view==0)
	{
		char text[20];
		sprintf(text , "   V1:%.2fV     ",get_ADC(&hadc2));
		LCD_DisplayStringLine(Line2 , (uint8_t*)text);
		//float k_read;
		//E2P_Read((u8*)&k_read, 0, sizeof(k_read));
		//sprintf(text , "%d\n",eeprom_read(0));
		//HAL_UART_Transmit(&huart1 , (uint8_t*)text , strlen(text) , 0xff);
		sprintf(text , "   k:%.1f     ",readFloatFromEEPROM(0));
		LCD_DisplayStringLine(Line3 , (uint8_t*)text);
		if(LED_mode==0)	LCD_DisplayStringLine(Line4 , (uint8_t*)"   LED:OFF     ");
		if(LED_mode==1)	LCD_DisplayStringLine(Line4 , (uint8_t*)"   LED:ON     ");
		sprintf(text , "   T:%d%d-%d%d-%d%d",hour_high,hour_low,minute_high,minute_low,second_high,second_low);
		LCD_DisplayStringLine(Line5 , (uint8_t*)text);
	}
	if(view==1)
	{
		char text[20];
		LCD_DisplayStringLine(Line2 , (uint8_t*)"      Setting     ");
		LCD_DisplayStringLine(Line3 , (uint8_t*)"               ");
		LCD_DisplayStringLine(Line4 , (uint8_t*)"               ");
		sprintf(text , "     %d%d-%d%d-%d%d     ",hour_high1,hour_low1,minute_high1,minute_low1,second_high1,second_low1);
		LCD_DisplayStringLine(Line5 , (uint8_t*)text);
		LCD_DisplayStringLine(Line6 , (uint8_t*)"                 ");
	}

}

void key_control()
{
	if(key[0].single_flag == 1)
	{
		LED_mode=!LED_mode;
		key[0].single_flag = 0;
		//char text[20];
		//sprintf(text,"%d\r\n",hour_high);
		//HAL_UART_Transmit(&huart1,text,strlen(text),0xff);
	}
	if(key[1].single_flag == 1)
	{
		if(view==1)	up_fflag=0;
		view=!view;
		
		key[1].single_flag = 0;
	}
	if(key[2].single_flag == 1)
	{
		clock_setting++;
		if(clock_setting==5)	clock_setting=0;
		key[2].single_flag = 0;
	}
	if(key[3].single_flag == 1)
	{
		//char text[5];
		//sprintf(text,"%d\r\n",clock_setting);
		//HAL_UART_Transmit(&huart1,(uint8_t *)text,strlen(text),0xff);
		if(clock_setting==0)	hour_high1++;
		if(clock_setting==1)	hour_low1++;
		if(clock_setting==2)	minute_high1++;
		if(clock_setting==3)	minute_low1++;
		if(clock_setting==4)	second_high1++;
		if(clock_setting==5)	second_low1++;
		
		if(hour_high1==10)	hour_high1=0;
		if(hour_low1==10)	hour_low1=0;
		if(minute_high1==10)	minute_high1=0;
		if(minute_low1==10)	minute_low1=0;
		if(second_high1==10)	second_high1=0;
		if(second_low1==10)	second_low1=0;
		key[3].single_flag = 0;
	}

}

void led_control()
{
	if(get_ADC(&hadc2)>3.3*k){
		LED_show=1;
	}else{
		LED_show=0;
	}
	if(LED_mode==1 && LED_show==1){
		if(TIM1_flag==0){
			HAL_GPIO_WritePin(GPIOC , GPIO_PIN_8,GPIO_PIN_SET);
		}else if(TIM1_flag==1){
			HAL_GPIO_WritePin(GPIOC , GPIO_PIN_8,GPIO_PIN_RESET);
		}
	}else{
		HAL_GPIO_WritePin(GPIOC , GPIO_PIN_8,GPIO_PIN_SET);
	}
	
	HAL_GPIO_WritePin(GPIOC , GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2 , GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2 , GPIO_PIN_RESET);//熄灭�?有led
}

void uart_show()
{
	if(uart_mode==0 && uart_flag==1)
	{
		//char text[10];
		sscanf((char*)rx_buffer , "k%f\n",&k);		//读取k
		
		//sprintf(text , "%.1f\n",k);
		//HAL_UART_Transmit(&huart1 , (uint8_t*)text , strlen(text) , 0xff);
		//uint8_t a = 5;
		//eeprom_write(0,k);
		E2P_Write((u8*)&k, 0, sizeof(k));			//写入�?
		//sprintf(text , "   k:%.1f     ",k);
		//uint8_t a_write = 5;
		//E2P_Write(&a_write, 0, sizeof(a_write));

		// 读取测试�?
		//uint8_t a_read;
		//E2P_Read(&a_read, 0, sizeof(a_read));

		// 发�?�测试�??
		//sprintf(text, "%u\n", a_read);
		//HAL_UART_Transmit(&huart1, (uint8_t*)text, strlen(text), 0xFF);
		HAL_UART_Transmit(&huart1 , (uint8_t*)"ok\n" , strlen("ok\n") , 0xff);
		
		uart_flag=0;	
		rx_pointer=0;
		memset(rx_buffer , 0 , 30);
	}
}

float readFloatFromEEPROM(uint8_t startAddress) {
    float result;
    uint8_t *p = (uint8_t *)&result;
    
    for (int i = 0; i < sizeof(float); i++) {
        p[i] = eeprom_read(startAddress + i);
    }
    return result;
}
void clock()
{
	if(TIM2_flag == 1)
	{
		second_low++;
		TIM2_flag = 0;
		if(second_low >9)	
		{
			second_low = 0;
			second_high++;
		}
		if(second_high >5)
		{
			second_high = 0;
			minute_low++;
		}
		if(minute_low >9)
		{
			minute_low = 0;
			minute_high ++;
		}
		if(minute_high >5)
		{
			minute_high = 0;
			hour_low ++;
		}
		if(hour_low >9)
		{
			hour_low = 0;
			hour_high ++;				
		}
		if(hour_high == 2 && hour_low == 4)
		{
				hour_high = 0;
				hour_low = 0;
				minute_high = 0;
				minute_low = 0;
				second_high = 0;
				second_low = 0;
		}
		
	
	}
}

void upData()
{
	if((minute_high1==minute_high)&&(minute_low==minute_low1)&&(second_high1==second_high)&&(second_low1==second_low)&&(hour_high1==hour_high)&&(hour_low1==hour_low)&&(up_fflag==0))
	{
		up_flag=1;
	}
	if(up_flag==1)
	{
		char text[20];
		sprintf(text,"%.2f+%.1f+%d%d%d%d%d%d\n",get_ADC(&hadc2),readFloatFromEEPROM(0),hour_high,hour_low,minute_high,minute_low,second_high,second_low);
		HAL_UART_Transmit(&huart1,(uint8_t*)text,strlen(text),0xff);
		up_flag=0;
		up_fflag=1;
	}

}

uint8_t eeprom_read(uint8_t addr)
{
	uint8_t dat;
	I2CStart();
	I2CSendByte(0xa0);	//Write
	I2CWaitAck();	//Wait
	I2CSendByte(addr);
	I2CWaitAck();
	//I2CStop();
	
	I2CStart();
	I2CSendByte(0xa1);
	I2CWaitAck();
	dat = I2CReceiveByte();
	I2CSendNotAck();
	I2CStop();
	return dat;
}

void eeprom_write(uint8_t addr,uint8_t dat)
{
	I2CStart();
	I2CSendByte(0xa0);	//Write
	I2CWaitAck();	//Wait
	I2CSendByte(addr);
	I2CWaitAck();	//Wait
	I2CSendByte(dat);
	I2CWaitAck();	//Wait
	I2CStop();
}
void E2P_Read(u8 *string, u16 add, u16 num){
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	
	I2CSendByte(add);
	I2CWaitAck();
	
	I2CStart();
	I2CSendByte(0xa1);
	I2CWaitAck();	
	
	while(num--){
		*string = I2CReceiveByte();
		if(num) I2CSendAck();
		else I2CSendNotAck();
	}
	I2CStop();
	HAL_Delay(5);
}

void E2P_Write(u8 *string, u16 add, u16 num){
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	
	I2CSendByte(add);
	I2CWaitAck();
	
	while(num--){
		I2CSendByte(*string++);
		I2CWaitAck();
	}
	I2CStop();
	HAL_Delay(5);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
