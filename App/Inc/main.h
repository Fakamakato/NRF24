/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__
#include "string.h"
#include "stdbool.h"
#include "nrf24.h"
#include "stm32f10x_conf.h"
#include "stdio.h"
#include "fir_stm.h"
#define RCC_PLLSource_HSE_Div1           ((uint32_t)0x00010000)
//#define SYSCLK_FREQ_40MHz   40000000
#define SYSCLK_FREQ_24MHz   24000000
#define SYS_CORE_CLOCK      40000000
#define BUFSIZE 1000
#define SPI_MASTER_PIN_IRQ           GPIO_Pin_1
#define SPI_MASTER_PIN_CE            GPIO_Pin_0
#define SPI_MASTER_PIN_NSS           GPIO_Pin_12
#define SPI_MASTER_PIN_SCK           GPIO_Pin_13
#define SPI_MASTER_PIN_MISO          GPIO_Pin_14
#define SPI_MASTER_PIN_MOSI          GPIO_Pin_15

/* Includes ------------------------------------
 * ------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
typedef enum
{
    APP_STATE_INIT = 0,
    APP_STATE_UART_CONF,
    APP_STATE_WHILE,
    APP_STATE_WAIT_UART,
    APP_STATE_UART_ERROR,
    APP_STATE_SMA

} APP_STATES;

/* USER CODE END Private defines */
volatile bool flagS,flagE;
int uart_c;
char cmdt[20],cmdr[100];
#ifdef __cplusplus
 extern "C" {
#endif
void SystemClock_Config(void);
void MY_GPIO_Init(void);
void MY_USART3_Init(void);
void MY_TIM2_Init(void);
void MY_TIM3_Init(void);
void MY_ADC0_Init(void);
void NVIC_Configuration(void);
void UART_SEND_STR(char *str,uint8_t size);
void SetSysClock(void);
void Delay(__IO uint32_t nCount);
void _Error_Handler(char *, int);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
