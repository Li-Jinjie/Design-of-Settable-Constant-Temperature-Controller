/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define INC_pwm_Pin GPIO_PIN_2
#define INC_pwm_GPIO_Port GPIOE
#define UD_pwm_Pin GPIO_PIN_3
#define UD_pwm_GPIO_Port GPIOE
#define DAC_now_Pin GPIO_PIN_4
#define DAC_now_GPIO_Port GPIOA
#define DAC_set_Pin GPIO_PIN_5
#define DAC_set_GPIO_Port GPIOA
#define ADC_PID_Pin GPIO_PIN_7
#define ADC_PID_GPIO_Port GPIOA
#define A_IO4_Pin GPIO_PIN_11
#define A_IO4_GPIO_Port GPIOF
#define C_IO3_Pin GPIO_PIN_13
#define C_IO3_GPIO_Port GPIOF
#define B_IO2_Pin GPIO_PIN_15
#define B_IO2_GPIO_Port GPIOF
#define DS18B20_Pin GPIO_PIN_0
#define DS18B20_GPIO_Port GPIOG
#define D_IO1_Pin GPIO_PIN_1
#define D_IO1_GPIO_Port GPIOG
#define IO5_Pin GPIO_PIN_8
#define IO5_GPIO_Port GPIOE
#define IO6_Pin GPIO_PIN_10
#define IO6_GPIO_Port GPIOE
#define IO7_Pin GPIO_PIN_12
#define IO7_GPIO_Port GPIOE
#define up_Pin GPIO_PIN_12
#define up_GPIO_Port GPIOB
#define up_EXTI_IRQn EXTI15_10_IRQn
#define down_Pin GPIO_PIN_13
#define down_GPIO_Port GPIOB
#define down_EXTI_IRQn EXTI15_10_IRQn
#define weishu_Pin GPIO_PIN_14
#define weishu_GPIO_Port GPIOB
#define weishu_EXTI_IRQn EXTI15_10_IRQn
#define Cold_control_Pin GPIO_PIN_0
#define Cold_control_GPIO_Port GPIOE
#define CS_pwm_Pin GPIO_PIN_1
#define CS_pwm_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
