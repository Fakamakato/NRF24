/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
/* Includes ------------------------------------------------------------------*/
//#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#ifdef JOY_NUM1
    #define SB6 "SB6_1"
    #define SB5 "SB5_1"
    #define SB4 "SB4_1"
    #define SB3 "SB3_1"
    #define SB2 "SB2_1"
    #define SB1 "SB1_1"
#endif
#ifdef JOY_NUM2
    #define SB6 "SB6_2"
    #define SB5 "SB5_2"
    #define SB4 "SB4_2"
    #define SB3 "SB3_2"
    #define SB2 "SB2_2"
    #define SB1 "SB1_2"
#endif
#ifdef JOY_NUM3
    #define SB5 "SB5_3"
    #define SB4 "SB4_3"
    #define SB3 "SB3_3"
    #define SB2 "SB2_3"
    #define SB1 "SB1_3"
#endif
#ifdef JOY_NUM4
    #define SB5 "SB5_4"
    #define SB4 "SB4_4"
    #define SB3 "SB3_4"
    #define SB2 "SB2_4"
    #define SB1 "SB1_4"
#endif
#ifdef JOY_NUM5
    #define SB5 "SB5_5"
    #define SB4 "SB4_5"
    #define SB3 "SB3_5"
    #define SB2 "SB2_5"
    #define SB1 "SB1_5"
#endif
//#define IS_GPIO_PIN(PIN)           ((((PIN) & GPIO_PIN_MASK ) != 0x00U) && (((PIN) & ~GPIO_PIN_MASK) == 0x00U))
/* USER CODE BEGIN 0 */
extern uint8_t buf[6],RX_BUF[5];
extern volatile  double ADC1ConvertedValue;
extern volatile uint16_t adc[BUFSIZE];
extern volatile uint16_t adc_count, ADC2ConvertedValue;
extern volatile  APP_STATES appState;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
    /* USER CODE BEGIN HardFault_IRQn 1 */

    /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
    /* USER CODE BEGIN MemoryManagement_IRQn 1 */

    /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
    /* USER CODE BEGIN BusFault_IRQn 1 */

    /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
    /* USER CODE BEGIN UsageFault_IRQn 1 */

    /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{


}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM2 global interrupt.
*/
void ADC1_IRQHandler(void)
{


    //USART_SendData(USART1,ADC2ConvertedValue >> 8);
}
volatile uint16_t counter=0;
void TIM2_IRQHandler(void)
{

    if(TIM_GetITStatus(TIM2,TIM_IT_Update))
    {
        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
        GPIOC->ODR ^= GPIO_Pin_13;

    }

}


/**
* @brief This function handles USART1 global interrupt.
*/
volatile char  buff[50];
volatile uint16_t sr,RxCounter=0;
void USART3_IRQHandler(void){

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART3,USART_IT_RXNE);
        // Read one byte from the receive data register
        buff[RxCounter++] = USART_ReceiveData(USART3);
        if( RxCounter == 0xF )
        {
            RxCounter = 0;
        }
    }
    if(USART_GetITStatus(USART3, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(USART3,USART_IT_TC);
    }

}

volatile uint32_t tmp;
#pragma GCC push_options
#pragma GCC optimize("O0")

void __attribute__((optimize("O0"))) EXTI9_5_IRQHandler(void)
//EXTI15_10_IRQnIRQHandler(void)
{
    volatile uint8_t reg;

    if(EXTI_GetITStatus(EXTI_Line9) != RESET)
    {

        EXTI->PR = EXTI_Line9;
        NRF24_Init();

        R_SEND(SB5,5);

        __DSB();
        __ISB();
        __NOP();
        __NOP();
        __NOP();
    }
    if(EXTI_GetITStatus(EXTI_Line8) != RESET)
    {
        EXTI->PR = EXTI_Line8;
        NRF24_Init();
        R_SEND(SB6,5);
        __DSB();
        __ISB();
        __NOP();
        __NOP();
        __NOP();
    }
    if(EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
        EXTI->PR = EXTI_Line7;
        NRF24_Init();
        R_SEND(SB1,5);
        __DSB();
        __ISB();
        __NOP();
        __NOP();
        __NOP();
    }
    if(EXTI_GetITStatus(EXTI_Line6) != RESET)
    {
        EXTI->PR = EXTI_Line6;
        NRF24_Init();
        R_SEND(SB2,5);
        __DSB();
        __ISB();
        __NOP();
        __NOP();
        __NOP();
    }

    if(EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        EXTI->PR = EXTI_Line5;
        NRF24_Init();
        R_SEND(SB3,5);
        __DSB();
        __ISB();
        __NOP();
        __NOP();
        __NOP();
    }


}



void __attribute__((optimize("O0"))) EXTI0_IRQHandler(void)
{
    //volatile uint8_t reg;
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI->PR = EXTI_Line0;

        #if NRF_isTX
        NRF24_Init();
        R_SEND(SB4,5);
        __DSB();
        __ISB();
        #else
        GPIOC->ODR ^= GPIO_Pin_13;

        R_READ(RX_BUF);
        //NRF24_Init();
        __DSB();
        __ISB();
        __NOP();
        UART_SEND_STR(RX_BUF,5);
        __NOP();
        __NOP();
        #endif
    }
}

#pragma GCC pop_options
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
