/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_VSYNC_Pin GPIO_PIN_9
#define LCD_VSYNC_GPIO_Port GPIOI
#define LCD_HSYNC_Pin GPIO_PIN_10
#define LCD_HSYNC_GPIO_Port GPIOI
#define FMC_A0_Pin GPIO_PIN_0
#define FMC_A0_GPIO_Port GPIOF
#define FMC_A1_Pin GPIO_PIN_1
#define FMC_A1_GPIO_Port GPIOF
#define FMC_A2_Pin GPIO_PIN_2
#define FMC_A2_GPIO_Port GPIOF
#define FMC_A3_Pin GPIO_PIN_3
#define FMC_A3_GPIO_Port GPIOF
#define FMC_A4_Pin GPIO_PIN_4
#define FMC_A4_GPIO_Port GPIOF
#define FMC_A5_Pin GPIO_PIN_5
#define FMC_A5_GPIO_Port GPIOF
#define LCD_DE_Pin GPIO_PIN_10
#define LCD_DE_GPIO_Port GPIOF
#define FMC_SDNWE_Pin GPIO_PIN_0
#define FMC_SDNWE_GPIO_Port GPIOC
#define ETH_MDC_Pin GPIO_PIN_1
#define ETH_MDC_GPIO_Port GPIOC
#define FMC_SDNE0_Pin GPIO_PIN_2
#define FMC_SDNE0_GPIO_Port GPIOC
#define FMC_SDCKE0_Pin GPIO_PIN_3
#define FMC_SDCKE0_GPIO_Port GPIOC
#define ETH_REF_CLK_Pin GPIO_PIN_1
#define ETH_REF_CLK_GPIO_Port GPIOA
#define ETH_MDIO_Pin GPIO_PIN_2
#define ETH_MDIO_GPIO_Port GPIOA
#define OV5640_PCLK_Pin GPIO_PIN_6
#define OV5640_PCLK_GPIO_Port GPIOA
#define ETH_CRS_DV_Pin GPIO_PIN_7
#define ETH_CRS_DV_GPIO_Port GPIOA
#define ETH_RXD0_Pin GPIO_PIN_4
#define ETH_RXD0_GPIO_Port GPIOC
#define ETH_RXD1_Pin GPIO_PIN_5
#define ETH_RXD1_GPIO_Port GPIOC
#define FMC_SDNRAS_Pin GPIO_PIN_11
#define FMC_SDNRAS_GPIO_Port GPIOF
#define FMC_A6_Pin GPIO_PIN_12
#define FMC_A6_GPIO_Port GPIOF
#define FMC_A7_Pin GPIO_PIN_13
#define FMC_A7_GPIO_Port GPIOF
#define FMC_A8_Pin GPIO_PIN_14
#define FMC_A8_GPIO_Port GPIOF
#define FMC_A9_Pin GPIO_PIN_15
#define FMC_A9_GPIO_Port GPIOF
#define FMC_A10_Pin GPIO_PIN_0
#define FMC_A10_GPIO_Port GPIOG
#define FMC_A11_Pin GPIO_PIN_1
#define FMC_A11_GPIO_Port GPIOG
#define FMC_D4_Pin GPIO_PIN_7
#define FMC_D4_GPIO_Port GPIOE
#define FMC_D5_Pin GPIO_PIN_8
#define FMC_D5_GPIO_Port GPIOE
#define FMC_D6_Pin GPIO_PIN_9
#define FMC_D6_GPIO_Port GPIOE
#define FMC_D7_Pin GPIO_PIN_10
#define FMC_D7_GPIO_Port GPIOE
#define FMC_D8_Pin GPIO_PIN_11
#define FMC_D8_GPIO_Port GPIOE
#define FMC_D9_Pin GPIO_PIN_12
#define FMC_D9_GPIO_Port GPIOE
#define FMC_D10_Pin GPIO_PIN_13
#define FMC_D10_GPIO_Port GPIOE
#define FMC_D11_Pin GPIO_PIN_14
#define FMC_D11_GPIO_Port GPIOE
#define FMC_D12_Pin GPIO_PIN_15
#define FMC_D12_GPIO_Port GPIOE
#define ETH_TX_EN_Pin GPIO_PIN_11
#define ETH_TX_EN_GPIO_Port GPIOB
#define OV5640_HREF_Pin GPIO_PIN_8
#define OV5640_HREF_GPIO_Port GPIOH
#define LCD_R3_Pin GPIO_PIN_9
#define LCD_R3_GPIO_Port GPIOH
#define LCD_R4_Pin GPIO_PIN_10
#define LCD_R4_GPIO_Port GPIOH
#define LCD_R5_Pin GPIO_PIN_11
#define LCD_R5_GPIO_Port GPIOH
#define LCD_R6_Pin GPIO_PIN_12
#define LCD_R6_GPIO_Port GPIOH
#define FMC_D13_Pin GPIO_PIN_8
#define FMC_D13_GPIO_Port GPIOD
#define FMC_D14_Pin GPIO_PIN_9
#define FMC_D14_GPIO_Port GPIOD
#define FMC_D15_Pin GPIO_PIN_10
#define FMC_D15_GPIO_Port GPIOD
#define FMC_D0_Pin GPIO_PIN_14
#define FMC_D0_GPIO_Port GPIOD
#define FMC_D1_Pin GPIO_PIN_15
#define FMC_D1_GPIO_Port GPIOD
#define FMC_A12_Pin GPIO_PIN_2
#define FMC_A12_GPIO_Port GPIOG
#define FMC_BA0_Pin GPIO_PIN_4
#define FMC_BA0_GPIO_Port GPIOG
#define FMC_BA1_Pin GPIO_PIN_5
#define FMC_BA1_GPIO_Port GPIOG
#define LCD_R7_Pin GPIO_PIN_6
#define LCD_R7_GPIO_Port GPIOG
#define LCD_CLK_Pin GPIO_PIN_7
#define LCD_CLK_GPIO_Port GPIOG
#define FMC_SDCLK_Pin GPIO_PIN_8
#define FMC_SDCLK_GPIO_Port GPIOG
#define OV5640_D0_Pin GPIO_PIN_6
#define OV5640_D0_GPIO_Port GPIOC
#define OV5640_D1_Pin GPIO_PIN_7
#define OV5640_D1_GPIO_Port GPIOC
#define OV5640_D2_Pin GPIO_PIN_8
#define OV5640_D2_GPIO_Port GPIOC
#define OV5640_D3_Pin GPIO_PIN_9
#define OV5640_D3_GPIO_Port GPIOC
#define OV5640_XCLK_Pin GPIO_PIN_8
#define OV5640_XCLK_GPIO_Port GPIOA
#define LCD_G2_Pin GPIO_PIN_13
#define LCD_G2_GPIO_Port GPIOH
#define LCD_G3_Pin GPIO_PIN_14
#define LCD_G3_GPIO_Port GPIOH
#define LCD_G4_Pin GPIO_PIN_15
#define LCD_G4_GPIO_Port GPIOH
#define LCD_G5_Pin GPIO_PIN_0
#define LCD_G5_GPIO_Port GPIOI
#define LCD_G6_Pin GPIO_PIN_1
#define LCD_G6_GPIO_Port GPIOI
#define LCD_G7_Pin GPIO_PIN_2
#define LCD_G7_GPIO_Port GPIOI
#define OV5640_RESET_Pin GPIO_PIN_15
#define OV5640_RESET_GPIO_Port GPIOA
#define OV5640_D4_Pin GPIO_PIN_11
#define OV5640_D4_GPIO_Port GPIOC
#define FMC_D2_Pin GPIO_PIN_0
#define FMC_D2_GPIO_Port GPIOD
#define FMC_D3_Pin GPIO_PIN_1
#define FMC_D3_GPIO_Port GPIOD
#define OV5640_D5_Pin GPIO_PIN_3
#define OV5640_D5_GPIO_Port GPIOD
#define LCD_B3_Pin GPIO_PIN_11
#define LCD_B3_GPIO_Port GPIOG
#define ETH_TXD0_Pin GPIO_PIN_13
#define ETH_TXD0_GPIO_Port GPIOG
#define ETH_TXD1_Pin GPIO_PIN_14
#define ETH_TXD1_GPIO_Port GPIOG
#define FMC_SDNCAS_Pin GPIO_PIN_15
#define FMC_SDNCAS_GPIO_Port GPIOG
#define OV5640_SDA_Pin GPIO_PIN_3
#define OV5640_SDA_GPIO_Port GPIOB
#define OV5640_SCL_Pin GPIO_PIN_4
#define OV5640_SCL_GPIO_Port GPIOB
#define LCD_BL_Pin GPIO_PIN_5
#define LCD_BL_GPIO_Port GPIOB
#define OV5640_VSYNC_Pin GPIO_PIN_7
#define OV5640_VSYNC_GPIO_Port GPIOB
#define OV5640_D6_Pin GPIO_PIN_8
#define OV5640_D6_GPIO_Port GPIOB
#define OV5640_D7_Pin GPIO_PIN_9
#define OV5640_D7_GPIO_Port GPIOB
#define FMC_NBL0_Pin GPIO_PIN_0
#define FMC_NBL0_GPIO_Port GPIOE
#define FMC_NBL1_Pin GPIO_PIN_1
#define FMC_NBL1_GPIO_Port GPIOE
#define LCD_B4_Pin GPIO_PIN_4
#define LCD_B4_GPIO_Port GPIOI
#define LCD_B5_Pin GPIO_PIN_5
#define LCD_B5_GPIO_Port GPIOI
#define LCD_B6_Pin GPIO_PIN_6
#define LCD_B6_GPIO_Port GPIOI
#define LCD_B7_Pin GPIO_PIN_7
#define LCD_B7_GPIO_Port GPIOI

/* USER CODE BEGIN Private defines */
#define SDRAM_BASE           (0xC0000000UL)
#define LCD_WIDTH            1024U
#define LCD_HEIGHT           600U
#define LCD_FB_BYTES         (LCD_WIDTH * LCD_HEIGHT * 2U)
#define LCD_FB_ADDR          (SDRAM_BASE)
#define LCD_FB_BACK          (SDRAM_BASE + LCD_FB_BYTES)
/* 缓冲按传感器满幅留着。平时 LCD 看 RGB565，右键拍照才出 JPEG。 */
#define CAM_FULL_W           2592U
#define CAM_FULL_H           1944U
#define CAM_MAX_BYTES        (CAM_FULL_W * CAM_FULL_H * 2U)
#define CAM_BUF0             (LCD_FB_BACK + LCD_FB_BYTES)
#define CAM_BUF1             (CAM_BUF0 + CAM_MAX_BYTES)
/* 一次 DMA 不超过 65535 个字。JPEG 长度不固定，收到场同步后按 FF D8..FF D9 截。 */
#define CAM_JPEG_CAP         (200U * 1024U)
/* 按下右键后把一帧 JPEG 拷到这里，网页刷新时重复发送，采集可以继续。 */
#define PHOTO_MAX            CAM_JPEG_CAP
#define PHOTO_ADDR           (CAM_BUF1 + CAM_MAX_BYTES)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
