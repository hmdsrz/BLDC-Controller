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

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define M2_FF1_Pin GPIO_PIN_2
#define M2_FF1_GPIO_Port GPIOE
#define M2_FF2_Pin GPIO_PIN_3
#define M2_FF2_GPIO_Port GPIOE
#define M2_PWM_Pin GPIO_PIN_5
#define M2_PWM_GPIO_Port GPIOE
#define M1_PWM_Pin GPIO_PIN_6
#define M1_PWM_GPIO_Port GPIOE
#define M1_DIR_Pin GPIO_PIN_13
#define M1_DIR_GPIO_Port GPIOC
#define M1_FF1_Pin GPIO_PIN_14
#define M1_FF1_GPIO_Port GPIOC
#define M1_FF2_Pin GPIO_PIN_15
#define M1_FF2_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_0
#define OLED_SDA_GPIO_Port GPIOF
#define OLED_SCA_Pin GPIO_PIN_1
#define OLED_SCA_GPIO_Port GPIOF
#define Vbat_FB_Pin GPIO_PIN_3
#define Vbat_FB_GPIO_Port GPIOF
#define Error_Flag_Pin GPIO_PIN_4
#define Error_Flag_GPIO_Port GPIOF
#define Charger_En_Pin GPIO_PIN_5
#define Charger_En_GPIO_Port GPIOF
#define Direct_Kick_Pin GPIO_PIN_6
#define Direct_Kick_GPIO_Port GPIOF
#define Chip_Kick_Pin GPIO_PIN_7
#define Chip_Kick_GPIO_Port GPIOF
#define Freq_1_Pin GPIO_PIN_9
#define Freq_1_GPIO_Port GPIOF
#define M1_CLimit_Pin GPIO_PIN_10
#define M1_CLimit_GPIO_Port GPIOF
#define M1_ENA_Pin GPIO_PIN_0
#define M1_ENA_GPIO_Port GPIOA
#define M1_ENB_Pin GPIO_PIN_1
#define M1_ENB_GPIO_Port GPIOA
#define M1_TACHO_Pin GPIO_PIN_2
#define M1_TACHO_GPIO_Port GPIOA
#define M2_CLimit_Pin GPIO_PIN_3
#define M2_CLimit_GPIO_Port GPIOA
#define RxNRF_NSS_Pin GPIO_PIN_4
#define RxNRF_NSS_GPIO_Port GPIOA
#define RxNRF_SCK_Pin GPIO_PIN_5
#define RxNRF_SCK_GPIO_Port GPIOA
#define RxNRF_MISO_Pin GPIO_PIN_6
#define RxNRF_MISO_GPIO_Port GPIOA
#define RxNRF_MOSI_Pin GPIO_PIN_7
#define RxNRF_MOSI_GPIO_Port GPIOA
#define RxNRF_IRQ_Pin GPIO_PIN_4
#define RxNRF_IRQ_GPIO_Port GPIOC
#define RxNRF_CE_Pin GPIO_PIN_5
#define RxNRF_CE_GPIO_Port GPIOC
#define Freq_38_Pin GPIO_PIN_1
#define Freq_38_GPIO_Port GPIOB
#define M1_FF1F11_Pin GPIO_PIN_11
#define M1_FF1F11_GPIO_Port GPIOF
#define M1_FF2F12_Pin GPIO_PIN_12
#define M1_FF2F12_GPIO_Port GPIOF
#define Ball_Flag_Pin GPIO_PIN_13
#define Ball_Flag_GPIO_Port GPIOF
#define IMU_RX_Pin GPIO_PIN_7
#define IMU_RX_GPIO_Port GPIOE
#define IMU_TX_Pin GPIO_PIN_8
#define IMU_TX_GPIO_Port GPIOE
#define TxNRF_NSS_Pin GPIO_PIN_11
#define TxNRF_NSS_GPIO_Port GPIOE
#define TxNRF_SCK_Pin GPIO_PIN_12
#define TxNRF_SCK_GPIO_Port GPIOE
#define TxNRF_MISO_Pin GPIO_PIN_13
#define TxNRF_MISO_GPIO_Port GPIOE
#define TxNRF_MOSI_Pin GPIO_PIN_14
#define TxNRF_MOSI_GPIO_Port GPIOE
#define TxNRF_IRQ_Pin GPIO_PIN_15
#define TxNRF_IRQ_GPIO_Port GPIOE
#define TxNRF_CE_Pin GPIO_PIN_10
#define TxNRF_CE_GPIO_Port GPIOB
#define SP_PWM_Pin GPIO_PIN_11
#define SP_PWM_GPIO_Port GPIOB
#define SP_DIR_Pin GPIO_PIN_12
#define SP_DIR_GPIO_Port GPIOB
#define SP_Sleep_Pin GPIO_PIN_13
#define SP_Sleep_GPIO_Port GPIOB
#define M3_PWM_Pin GPIO_PIN_14
#define M3_PWM_GPIO_Port GPIOB
#define M4_PWM_Pin GPIO_PIN_15
#define M4_PWM_GPIO_Port GPIOB
#define M3_FF2_Pin GPIO_PIN_5
#define M3_FF2_GPIO_Port GPIOG
#define M3_FF1_Pin GPIO_PIN_6
#define M3_FF1_GPIO_Port GPIOG
#define M3_CLimit_Pin GPIO_PIN_7
#define M3_CLimit_GPIO_Port GPIOG
#define M3_DIR_Pin GPIO_PIN_8
#define M3_DIR_GPIO_Port GPIOG
#define M3_ENA_Pin GPIO_PIN_6
#define M3_ENA_GPIO_Port GPIOC
#define M3_ENB_Pin GPIO_PIN_7
#define M3_ENB_GPIO_Port GPIOC
#define M3_TACHO_Pin GPIO_PIN_8
#define M3_TACHO_GPIO_Port GPIOC
#define M4_CLimit_Pin GPIO_PIN_9
#define M4_CLimit_GPIO_Port GPIOC
#define M4_ENA_Pin GPIO_PIN_8
#define M4_ENA_GPIO_Port GPIOA
#define M4_ENB_Pin GPIO_PIN_9
#define M4_ENB_GPIO_Port GPIOA
#define M4_DIR_Pin GPIO_PIN_10
#define M4_DIR_GPIO_Port GPIOA
#define M4_TACHO_Pin GPIO_PIN_11
#define M4_TACHO_GPIO_Port GPIOA
#define M4_FF1_Pin GPIO_PIN_10
#define M4_FF1_GPIO_Port GPIOC
#define M4_FF2_Pin GPIO_PIN_11
#define M4_FF2_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_12
#define Buzzer_GPIO_Port GPIOC
#define LED_0_Pin GPIO_PIN_0
#define LED_0_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_2
#define LED_2_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOD
#define LED_4_Pin GPIO_PIN_4
#define LED_4_GPIO_Port GPIOD
#define LED_5_Pin GPIO_PIN_5
#define LED_5_GPIO_Port GPIOD
#define LED_6_Pin GPIO_PIN_6
#define LED_6_GPIO_Port GPIOD
#define LED_7_Pin GPIO_PIN_7
#define LED_7_GPIO_Port GPIOD
#define BACK_KEY_Pin GPIO_PIN_10
#define BACK_KEY_GPIO_Port GPIOG
#define UP_KEY_Pin GPIO_PIN_11
#define UP_KEY_GPIO_Port GPIOG
#define ENTER_KEY_Pin GPIO_PIN_12
#define ENTER_KEY_GPIO_Port GPIOG
#define DOWN_KEY_Pin GPIO_PIN_13
#define DOWN_KEY_GPIO_Port GPIOG
#define Charge_En_Pin GPIO_PIN_15
#define Charge_En_GPIO_Port GPIOG
#define M2_ENA_Pin GPIO_PIN_6
#define M2_ENA_GPIO_Port GPIOB
#define M2_ENB_Pin GPIO_PIN_7
#define M2_ENB_GPIO_Port GPIOB
#define M2_TACHO_Pin GPIO_PIN_9
#define M2_TACHO_GPIO_Port GPIOB
#define M2_DIR_Pin GPIO_PIN_0
#define M2_DIR_GPIO_Port GPIOE
#define M2_CLimitE1_Pin GPIO_PIN_1
#define M2_CLimitE1_GPIO_Port GPIOE

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

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
