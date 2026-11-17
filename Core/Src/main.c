/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ov5640.h"
#include "meter_locate.h"
#include "dial_read.h"
#include "dial_path_debug.h"
#include "speed_track.h"
#include "star_track.h"
#include "net_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_STAR_CELL        24U
/* 0：不画星空，减轻每帧运算。星空函数仍保留。 */
#define LCD_STARS_ON         0U
/* 1：背光和 LTDC 打开。屏幕只扫自己的显存。 */
#define LCD_PANEL_ON         1U
/* 网页先关掉。采集和液晶不依赖它。 */
#define NET_APP_ON           0U
/* 1：整屏 4x3 共 12 格纯色，用来核对分辨率和颜色。摄像头画面先不贴。 */
#define LCD_COLOR_GRID_ON    0U
/* 0：送摄像头原画。二值函数和阈值仍保留。 */
#define CAM_BIN_ON           0U
/* 1：在实时画面上画出红指针的圆和尖角，并跟上角度。梅花轮先不跟。 */
#define METER_LOCATE_ON       1U
/* 1: run the new dial-read pipeline (seam + rectify + 4-dial reading) on each LCD frame. */
#define DIAL_READ_ON           1U
/* 功能模式：0 四指针读体积（源文件不改，只作为可切换模式保留），1 升针转速跟踪，
 * 2 梅花轮齿频（star_track.c）。切回升针测速只改下面一行为 METER_MODE_SPEED。 */
#define METER_MODE_VOLUME      0U
#define METER_MODE_SPEED       1U
#define METER_MODE_STAR        2U
#define METER_MODE_DEFAULT     METER_MODE_STAR
#define MODE_REQUEST_NONE      0xFFFFFFFFUL
volatile uint32_t g_meter_mode = METER_MODE_DEFAULT;
volatile uint32_t g_meter_request = MODE_REQUEST_NONE; /* 0/1/2；调试写此值与按键走同一路径 */
volatile uint32_t g_meter_switching;
volatile uint32_t g_meter_switch_n;
volatile uint32_t g_meter_epoch;
volatile uint32_t g_meter_status; /* 0启动, 1停采集, 2复位算法, 3全幅初始化, 4运行, 0xE错误 */
volatile uint32_t g_meter_last_key; /* 1左 PC13, 2下 PH2, 3右 PH3；上 PA0 不切模式 */
volatile uint32_t g_meter_key_n;
volatile uint32_t g_meter_key_raw;
volatile uint32_t g_meter_key_stable;
volatile uint32_t g_meter_key_ticks;
volatile uint32_t g_meter_req_source; /* 1物理键, 2调试请求 */
volatile uint32_t g_meter_last_from;
volatile uint32_t g_meter_last_to;
volatile uint32_t g_meter_last_source;
static volatile uint32_t meter_keys_ready;
/* Snapshot + work sit after the JPEG photo buffer, not on CAM_BUF0. */
#define DIAL_SNAP_ADDR         (((PHOTO_ADDR + PHOTO_MAX) + 7UL) & ~7UL)
#define DIAL_WORK_ADDR         (DIAL_SNAP_ADDR + LCD_FB_BYTES)
/* RAW snapshot lease. Only the completed back buffer is copied, before
 * any HUD paint. DIAL_SNAP is private to the consumer until Run returns. */
typedef struct
{
  uint32_t request, ready, busy, frame_gen, paint_gen;
  uint32_t snap_gen, snap_stamp, source, copy_us, copy_n;
  uint32_t bad_copy, painted_skip, before_hud_n;
  uint32_t used_gen, used_stamp, used_seq, hash_saved, hash_used, hash_bad;
  uint32_t hold_gen0, hold_gen1, live_hud_n, used_frame0, used_frame1;
} meter_raw_diag_t;
volatile meter_raw_diag_t g_meter_raw_diag;
/* Optional diagnostics only: preserve one failed RAW input in unused SDRAM.
 * It never holds the live camera, LCD, or next recognition snapshot. */
#define DIAL_FAILED_RAW_ADDR 0xC1E00000UL
typedef struct {
 uint32_t request,ready,version,seq,run_seq,gen,stamp,hash,addr,bytes,copies,epoch;
} meter_failed_raw_t;
volatile meter_failed_raw_t g_meter_failed_raw;

/* 快门邮箱在 SDRAM 末段固定地址，该区已配成非缓存，J-Link 写入后 CPU 立刻能读到。
 * 0：连续捕获并识别。非 0：冻结当前画面，只识别这一帧，直到复位。 */
#define SHUTTER_MAIL_ADDR      0xC1FFF000UL
#define SHUTTER_MAIL_MAGIC     0x53485554UL
#define SHUTTER_FLAG_ADDR      (SHUTTER_MAIL_ADDR + 4UL)
#define shutter_flag           (*(volatile uint32_t *)SHUTTER_FLAG_ADDR)
/* SDCLK=100MHz，COUNT=64*1000*100/8192-20 */
#define SDRAM_REFRESH_COUNT  761U
#define SDRAM_TIMEOUT_MS     100U

#define SDRAM_MODEREG_BURST_LENGTH_1          0x0000U
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   0x0000U
#define SDRAM_MODEREG_CAS_LATENCY_2           0x0020U
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD 0x0000U
#define SDRAM_MODEREG_WRITEBURST_SINGLE       0x0200U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* 1 开窗，并按当前行周期恢复 50Hz 硬件消纹。2 手动曝光，3 手动增益，这两项会关掉消纹。 */
uint8_t f_send = 0;

uint16_t width = 1024;
uint16_t height = 600;
/* 相对摄像头画面中心。0,0 是正中，向右、向下为正。 */
int16_t x = 0;
int16_t y = 0;
uint32_t shutter_us = 1000;
uint16_t gain = 0x00F0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* 描述符和 heth 在 ethernetif.c，放在 D2 SRAM。这里只引用。 */
extern ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT];
extern ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT];
extern ETH_TxPacketConfigTypeDef TxConfig;
extern ETH_HandleTypeDef heth;

DCMI_HandleTypeDef hdcmi;

LTDC_HandleTypeDef hltdc;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_DCMI_Init(void);
static void MX_ETH_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void MPU_Config(void);
static void SDRAM_InitSequence(void);
static void LCD_DemoInit(void);
static void LCD_DemoFrame(void);
static void LCD_PresentLive(void);
static void Shutter_Init(void);
static void Shutter_Service(uint8_t allow_present);
static int CamLcdFrozen(void);
static void DialRead_Service(void);
static void SpeedTrack_Service(void);
static void StarMode_Service(void);
static void CAM_Init(void);
static void CAM_Start(void);
static int CAM_Kick(void);
static void CAM_Service(void);
static void CamDbg_Mark(uint16_t step);
static void CamDbg_SnapLive(void);
static void CamDbg_SnapSensor(void);
static void CAM_Apply(void);
static void CAM_ScaleToLcd(void);
void MeterKeys_Tick(void); /* SysTick 采样去抖，IRQ 只发请求不切相机 */
static void MeterMode_Service(void);
static void CAM_TakeShot(void);
static int LCD_BlitFit(void);
static int LCD_BlitCamera(void);
static void WEB_Publish(void);
void CAM_ReleaseFrame(void);
static void CAM_TraceUpdate(void);
static void MeterLocate_Service(void);
static void MeterLocate_Resume(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* 0xC0000000 默认是 Device。按 Device 访问 FMC SDRAM 会读错，LTDC 扫出来就是白底横线 */
  MPU_Config();
  SCB_EnableICache();
  HAL_EnableCompensationCell();
  for (uint32_t wait = 0U; wait < 1000000U; wait++)
  {
    if (READ_BIT(SYSCFG->CCCSR, SYSCFG_CCCSR_READY) != 0U)
    {
      break;
    }
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_DCMI_Init();
  MX_ETH_Init();
  /* USER CODE BEGIN 2 */
  {
    uint32_t n = ((uint32_t)LCD_WIDTH * (uint32_t)LCD_HEIGHT) / 2U;
    uint32_t i;
    volatile uint32_t *fb;
    /* 开机把前后两块显存都铺成蓝屏，避免扫到 SDRAM 残留画面。 */
    fb = (volatile uint32_t *)LCD_FB_ADDR;
    for (i = 0U; i < n; i++)
    {
      fb[i] = 0x001F001FUL;
    }
    fb = (volatile uint32_t *)LCD_FB_BACK;
    for (i = 0U; i < n; i++)
    {
      fb[i] = 0x001F001FUL;
    }
  }
  if (LCD_PANEL_ON == 0U)
  {
    __HAL_LTDC_DISABLE(&hltdc);
    HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);
  }
  LCD_DemoInit();
  MX_RTC_Init();
  MX_TIM2_Init();
  CAM_Init();
  Shutter_Init();
  meter_keys_ready = 1U;
  if (NET_APP_ON != 0U)
  {
    NET_Init();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    MeterMode_Service();
    Shutter_Service(1U);
    CAM_ScaleToLcd();
    CAM_Service();
    OV5640_FocusSweep();
    LCD_DemoFrame();
    if (g_meter_mode == METER_MODE_VOLUME)
    {
      DialRead_Service();
    }
    else if (g_meter_mode == METER_MODE_STAR)
    {
      StarMode_Service();
    }
    else
    {
      SpeedTrack_Service();
    }
    if (NET_APP_ON != 0U)
    {
      NET_Poll();
    }
    CAM_TraceUpdate();
    if (f_send == 1U)
    {
      f_send = 0U;
      OV5640_ShowOnLcd(x, y, width, height);
      CAM_Apply();
    }
    else if (f_send == 2U)
    {
      f_send = 0U;
      OV5640_SetExposure(shutter_us);
    }
    else if (f_send == 3U)
    {
      f_send = 0U;
      OV5640_SetGain(gain);
    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */
  /* 网页功能关着时不要初始化网口，避免 PHY 等待把摄像头启动卡住。 */
  if (NET_APP_ON == 0U)
  {
    return;
  }
  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1536;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  /* ATK-7016 / 1024x600：HSW 20，HBP 140，HFP 160，VSW 3，VBP 20，VFP 12。寄存器是周期减 1。 */
  hltdc.Init.HorizontalSync = 19;
  hltdc.Init.VerticalSync = 2;
  hltdc.Init.AccumulatedHBP = 159;
  hltdc.Init.AccumulatedVBP = 22;
  hltdc.Init.AccumulatedActiveW = 1183;
  hltdc.Init.AccumulatedActiveH = 622;
  hltdc.Init.TotalWidth = 1343;
  hltdc.Init.TotalHeigh = 634;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 1024;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 600;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0xC0000000;
  pLayerCfg.ImageWidth = 1024;
  pLayerCfg.ImageHeight = 600;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */
  __HAL_LTDC_LAYER_DISABLE(&hltdc, 1);
  __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&hltdc);
  /* USER CODE END LTDC_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 8;
  SdramTiming.SelfRefreshTime = 7;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 2;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  SDRAM_InitSequence();
  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OV5640_RESET_GPIO_Port, OV5640_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OV5640_SDA_Pin|OV5640_SCL_Pin|LCD_BL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : OV5640_XCLK_Pin */
  GPIO_InitStruct.Pin = OV5640_XCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(OV5640_XCLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OV5640_RESET_Pin */
  GPIO_InitStruct.Pin = OV5640_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OV5640_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OV5640_SDA_Pin OV5640_SCL_Pin */
  GPIO_InitStruct.Pin = OV5640_SDA_Pin|OV5640_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static uint8_t sin_lut[256];
static uint8_t cam_ok;
static volatile uint8_t cam_hold;
static volatile uint8_t cam_ready;
static uint8_t cam_jpeg_hold;
static volatile uint8_t cam_write_idx;
static volatile uint8_t cam_done_idx;
static volatile uint8_t cam_restart;
static volatile uint8_t cam_arm;
static volatile uint8_t shot_request;
static uint8_t cam_out_jpeg;
static uint32_t cam_dma_words;
static volatile uint32_t cam_cap_len[2];

static void CAM_Service(void)
{
  if (cam_restart != 0U)
  {
    cam_restart = 0U;
    CAM_Apply();
    return;
  }
  if (cam_arm != 0U)
  {
    cam_arm = 0U;
    CAM_Start();
  }
  CamDbg_SnapLive();
}
static uint16_t cam_w;
static uint16_t cam_h;
static volatile uint16_t cam_row;
static volatile uint16_t cam_dma_count;
static volatile uint16_t cam_frame_count;
static volatile uint16_t cam_chunks;
static uint16_t cam_chunks_need;
/* [0] ok|ready<<8|hold<<16|row<<24  [1] w|h<<16  [2] 摄像头缓冲首字  [3] 送去编码的画面中心 */
volatile uint32_t cam_trace[4];
/* 一行 1024 个 RGB565，DMA 按 32 位收，长度是宽的一半。两块缓冲轮流，和例程一样。 */
static uint32_t cam_line_buf[2][LCD_WIDTH / 2U] __attribute__((section(".bss.dma_line"), aligned(32)));
static volatile uint16_t cam_curline;
/* 定位时冻结 LCD 显存，行中断仍收数但不覆盖这一帧。 */
static volatile uint8_t meter_hold_fb;
/* 快门已置位：画面冻结，只识别这一帧。 */
static volatile uint8_t shutter_armed;
static uint8_t shutter_done;
static volatile uint8_t meter_want_snap;
static volatile uint8_t meter_frame_ready;
static uint8_t meter_phase;
static uint32_t meter_next_ms;
static meter_pointer_t meter_ptrs[METER_MAX_POINTERS];
static uint8_t meter_nptrs;
static uint8_t meter_skip_frames;
static uint8_t meter_refind_cd;


typedef struct
{
  int16_t x;
  int16_t y;
  uint8_t reach;
  uint8_t core;
  uint8_t hue;
  uint8_t gain;
  uint8_t phase;
} LCD_Star;

static const uint8_t layer_step[4] = { 3U, 5U, 7U, 9U };
static uint8_t layer_phase[4];
static uint8_t layer_peak[4];
static LCD_Star lcd_stars[(LCD_WIDTH / LCD_STAR_CELL) * (LCD_HEIGHT / LCD_STAR_CELL)];
static uint16_t lcd_star_count;
static volatile uint8_t lcd_stars_on = LCD_STARS_ON;
static volatile uint32_t lcd_draw;
static volatile uint8_t cam_draw_full;
static volatile uint32_t cam_stamp_us;
static volatile uint16_t cam_ox;
static volatile uint16_t cam_oy;
static speed_state_t g_speed;
/* J-Link 监视：裁窗步骤、关键量和 DCMI/DMA/传感器寄存器。字段放全局，随时可钩。 */
typedef struct
{
  uint16_t frames_pre;
  int16_t  kick_ret;
  uint32_t stamp_pre;
  uint32_t dcmi_cr;
  uint32_t dcmi_sr;
  uint32_t dcmi_ris;
  uint32_t dma_cr;
  uint32_t dma_ndtr;
  uint16_t rb_sys;
  uint16_t rb_out_w;
  uint16_t rb_hts;
  uint16_t rb_vts;
  uint16_t step;
  uint16_t rb_out_h;
  uint16_t rb_x0;
  uint16_t rb_y0;
  uint16_t rb_x1;
  uint16_t rb_y1;
  uint8_t  rb_sub;
  uint8_t  rb_3820;
  uint8_t  rb_3821;
  uint8_t  rb_dvp;
  uint16_t win_x;
  uint16_t win_y;
  uint16_t win_w;
  uint16_t win_h;
  uint16_t view_x;
  uint16_t view_y;
  uint16_t view_w;
  uint16_t view_h;
  uint16_t cap_w;
  uint16_t cap_h;
  uint16_t crop_sx;
  uint16_t crop_sy;
  uint16_t crop_sw;
  uint16_t crop_sh;
  int32_t  lcd_x0;
  int32_t  lcd_y0;
  uint32_t step_us[8];
  uint16_t frames_now;
  uint16_t curline_now;
  uint32_t stamp_now;
  uint32_t dcmi_sr_now;
  uint32_t dma_ndtr_now;
  uint32_t snap_count;
} cam_dbg_t;
static volatile cam_dbg_t g_cam_dbg;

static void CamDbg_Mark(uint16_t step)
{
  if (step < 8U)
  {
    g_cam_dbg.step_us[step] = TIM2->CNT;
  }
  g_cam_dbg.step = step;
}

static void CamDbg_SnapLive(void)
{
  DMA_Stream_TypeDef *dma = (DMA_Stream_TypeDef *)hdma_dcmi.Instance;

  g_cam_dbg.frames_now = cam_frame_count;
  g_cam_dbg.curline_now = cam_curline;
  g_cam_dbg.stamp_now = cam_stamp_us;
  g_cam_dbg.dcmi_cr = hdcmi.Instance->CR;
  g_cam_dbg.dcmi_sr_now = hdcmi.Instance->SR;
  g_cam_dbg.dcmi_ris = hdcmi.Instance->RISR;
  g_cam_dbg.dma_cr = (dma != 0) ? dma->CR : 0U;
  g_cam_dbg.dma_ndtr_now = (dma != 0) ? dma->NDTR : 0U;
  g_cam_dbg.snap_count++;
}

static void CamDbg_SnapSensor(void)
{
  uint16_t sys = 0U, out_w = 0U, hts = 0U, vts = 0U;
  uint16_t out_h = 0U, x0 = 0U, y0 = 0U, x1 = 0U, y1 = 0U;
  uint8_t sub = 0U, r20 = 0U, r21 = 0U, dvp = 0U;
  DMA_Stream_TypeDef *dma = (DMA_Stream_TypeDef *)hdma_dcmi.Instance;

  g_cam_dbg.kick_ret = 0;
  g_cam_dbg.dcmi_cr = hdcmi.Instance->CR;
  g_cam_dbg.dcmi_sr = hdcmi.Instance->SR;
  g_cam_dbg.dcmi_ris = hdcmi.Instance->RISR;
  g_cam_dbg.dma_cr = (dma != 0) ? dma->CR : 0U;
  g_cam_dbg.dma_ndtr = (dma != 0) ? dma->NDTR : 0U;
  OV5640_SnapRegs(&sys, &out_w, &hts, &vts);
  OV5640_SnapCropRegs(&out_h, &x0, &y0, &x1, &y1, &sub, &r20, &r21, &dvp);
  g_cam_dbg.rb_sys = sys;
  g_cam_dbg.rb_out_w = out_w;
  g_cam_dbg.rb_hts = hts;
  g_cam_dbg.rb_vts = vts;
  g_cam_dbg.rb_out_h = out_h;
  g_cam_dbg.rb_x0 = x0;
  g_cam_dbg.rb_y0 = y0;
  g_cam_dbg.rb_x1 = x1;
  g_cam_dbg.rb_y1 = y1;
  g_cam_dbg.rb_sub = sub;
  g_cam_dbg.rb_3820 = r20;
  g_cam_dbg.rb_3821 = r21;
  g_cam_dbg.rb_dvp = dvp;
  CamDbg_SnapLive();
}
static uint8_t speed_phase;
uint8_t  g_star_phase;      /* 梅花轮模式：0 全幅找轮，1 裁窗后在小窗里再定位，2 测齿频 */
uint16_t g_star_loc_fail;
#define CROP_SETTLE_US  1000000U  /* 裁窗后等自动曝光约 1 s 再开始认针 */
#define FULL_SETTLE_US  2000000U  /* 全幅找针前等自动曝光约 2 s */
uint8_t  g_crop_settling;
uint32_t g_crop_settle_t0;
uint32_t g_cam_settle_us;
static uint8_t dial_hud_ready;
static uint8_t dial_yield_lock;
static dial_report_t dial_hud_rep;
/* 一次识别完成时保存拒绝理由，避免 J-Link 恰好读到下一次 Run 清空中的工作诊断。 */
volatile struct
{
  uint32_t seq, status, flags;
  uint8_t n_dials, n_scale_bad, n_cand, miss_place;
  uint8_t reason[4];
  int16_t cx[4], cy[4];
  dial_dbg_cand_t cand[8];
} g_meter_volume_diag;
uint16_t g_meter_volume_seq;

volatile uint32_t lcd_frame_addr = LCD_FB_ADDR;
volatile uint32_t web_frame_addr;
volatile uint16_t web_frame_w;
volatile uint16_t web_frame_h;
volatile uint32_t web_jpeg_len;
extern volatile uint32_t web_encode_addr;
extern volatile uint8_t net_need_snap;
volatile uint16_t web_snap_seq;

uint8_t CAM_HasFrame(void)
{
  return cam_ready;
}

static void CAM_TraceUpdate(void)
{
  const volatile uint32_t *src = (cam_done_idx == 0U)
      ? (const volatile uint32_t *)CAM_BUF0
      : (const volatile uint32_t *)CAM_BUF1;
  uint32_t words = cam_cap_len[cam_done_idx] / 4U;
  uint32_t acc = 0U;
  uint32_t i;

  cam_trace[0] = (uint32_t)cam_ok | ((uint32_t)cam_ready << 8) |
                 ((uint32_t)(cam_frame_count & 0xFFU) << 16) |
                 ((uint32_t)(cam_dma_count & 0xFFU) << 24);
  cam_trace[1] = (uint32_t)cam_w | ((uint32_t)cam_h << 16);
  if (words > 16U)
  {
    for (i = 0U; i < 16U; i++)
    {
      acc |= src[(words / 16U) * i];
    }
  }
  cam_trace[2] = acc;
  cam_trace[3] = web_jpeg_len;
}
static RNG_HandleTypeDef hrng;

static void LCD_BuildTables(void)
{
  uint32_t i;

  for (i = 0U; i < 256U; i++)
  {
    uint32_t quadrant = i >> 6;
    uint32_t x = i & 63U;
    uint32_t degree;
    uint32_t rise;
    uint32_t span;
    uint32_t amp;

    if ((quadrant & 1U) != 0U)
    {
      x = 63U - x;
    }
    degree = (x * 90U) / 63U;
    rise = 4U * degree * (180U - degree);
    span = 40500U - (degree * (180U - degree));
    amp = (span == 0U) ? 0U : ((rise * 255U) / span);
    if (amp > 255U)
    {
      amp = 255U;
    }
    sin_lut[i] = (quadrant >= 2U) ? (uint8_t)(128U - (amp >> 1)) : (uint8_t)(128U + (amp >> 1));
  }
}

static void LCD_RngInit(void)
{
  RCC_PeriphCLKInitTypeDef clk = {0};

  __HAL_RCC_HSI48_ENABLE();
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY) == RESET)
  {
  }
  clk.PeriphClockSelection = RCC_PERIPHCLK_RNG;
  clk.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_RCC_RNG_CLK_ENABLE();
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_DISABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
}

static uint8_t LCD_RandomPeak(void)
{
  uint32_t rnd = 0U;

  if (HAL_RNG_GenerateRandomNumber(&hrng, &rnd) != HAL_OK)
  {
    rnd = HAL_GetTick();
  }
  return (uint8_t)(128U + (rnd & 0x7FU));
}

static uint32_t LCD_Hash(int16_t x, int16_t y)
{
  uint32_t h = (uint32_t)(x + 40) * 2246822519U;

  h ^= (uint32_t)(y + 40) * 3266489917U;
  h ^= h >> 16;
  return h;
}

static void LCD_BuildStars(void)
{
  int16_t cy;
  int16_t cx;

  lcd_star_count = 0U;
  for (cy = 0; cy < (int16_t)(LCD_HEIGHT / LCD_STAR_CELL); cy++)
  {
    for (cx = 0; cx < (int16_t)(LCD_WIDTH / LCD_STAR_CELL); cx++)
    {
      uint32_t h = LCD_Hash(cx, cy);
      uint32_t kind = h % 10U;
      LCD_Star *star;

      if ((h & 0x200U) == 0U)
      {
        continue;
      }
      star = &lcd_stars[lcd_star_count++];

      uint32_t jitter = (h >> 18) & 0x0FU;
      uint32_t reach;

      star->x = (int16_t)((cx * (int16_t)LCD_STAR_CELL) + 2 + (int16_t)((h >> 8) % 18U));
      star->y = (int16_t)((cy * (int16_t)LCD_STAR_CELL) + 2 + (int16_t)((h >> 16) % 18U));
      star->phase = (uint8_t)(h & 3U);
      star->hue = (uint8_t)(h >> 24);
      star->gain = (uint8_t)(48U + ((h >> 4) & 0xCFU));
      if (kind <= 3U)
      {
        star->reach = 0U;
        star->core = 1U;
      }
      else if (kind <= 6U)
      {
        star->reach = 9U;
        star->core = 2U;
      }
      else if (kind <= 8U)
      {
        star->reach = 20U;
        star->core = 3U;
      }
      else
      {
        star->reach = 36U;
        star->core = 4U;
      }
      reach = (uint32_t)star->reach + (((jitter & 7U) * star->reach) / 8U);
      if (reach > 48U)
      {
        reach = 48U;
      }
      star->reach = (uint8_t)reach;
      star->core = (uint8_t)(star->core + ((jitter >> 3) & 1U));
    }
  }
}

static uint16_t LCD_Color(uint8_t hue, uint16_t glow)
{
  uint32_t u = glow;
  uint32_t region = (uint32_t)hue / 43U;
  uint32_t f = ((uint32_t)hue % 43U) * 255U / 42U;
  uint32_t q = 255U - f;
  uint32_t r;
  uint32_t g;
  uint32_t b;
  uint32_t white;

  if (u > 255U)
  {
    u = 255U;
  }
  if (region == 0U)
  {
    r = 255U;
    g = f;
    b = 0U;
  }
  else if (region == 1U)
  {
    r = q;
    g = 255U;
    b = 0U;
  }
  else if (region == 2U)
  {
    r = 0U;
    g = 255U;
    b = f;
  }
  else if (region == 3U)
  {
    r = 0U;
    g = q;
    b = 255U;
  }
  else if (region == 4U)
  {
    r = f;
    g = 0U;
    b = 255U;
  }
  else
  {
    r = 255U;
    g = 0U;
    b = q;
  }
  r = (r * u) / 255U;
  g = (g * u) / 255U;
  b = (b * u) / 255U;
  if (u > 140U)
  {
    white = ((u - 140U) * 255U) / 115U;
    r += ((255U - r) * white) / 255U;
    g += ((255U - g) * white) / 255U;
    b += ((255U - b) * white) / 255U;
  }
  return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

static uint32_t LCD_Level(uint16_t pix)
{
  return (((uint32_t)(pix >> 11) & 0x1FU) << 1) +
         ((uint32_t)(pix >> 5) & 0x3FU) +
         (((uint32_t)pix & 0x1FU) << 1);
}

static void LCD_Put(int16_t x, int16_t y, uint8_t hue, uint16_t glow)
{
  volatile uint16_t *pix;
  uint16_t neu;

  if (((uint16_t)x >= LCD_WIDTH) || ((uint16_t)y >= LCD_HEIGHT) || (glow < 3U))
  {
    return;
  }
  pix = (volatile uint16_t *)lcd_draw + (((uint32_t)y * LCD_WIDTH) + (uint32_t)x);
  neu = LCD_Color(hue, glow);
  if (LCD_Level(neu) <= LCD_Level(*pix))
  {
    return;
  }
  *pix = neu;
}

static void LCD_DrawStar(const LCD_Star *star, uint16_t scale)
{
  int16_t core = (int16_t)star->core;
  int16_t halo = (int16_t)(core * 3);
  int16_t reach = (int16_t)star->reach;
  int16_t dy;
  int16_t dx;
  int16_t along;
  int16_t dir;

  for (dy = (int16_t)-halo; dy <= halo; dy++)
  {
    for (dx = (int16_t)-halo; dx <= halo; dx++)
    {
      int16_t r2 = (int16_t)((dx * dx) + (dy * dy));
      int16_t glow;

      if (r2 <= (core * core))
      {
        glow = (int16_t)(255 - ((r2 * 160) / ((core * core) + 1)));
      }
      else if (r2 <= (halo * halo))
      {
        glow = (int16_t)(48 - ((r2 * 48) / ((halo * halo) + 1)));
      }
      else
      {
        continue;
      }
      LCD_Put((int16_t)(star->x + dx), (int16_t)(star->y + dy), star->hue,
              (uint16_t)(((uint16_t)glow * scale) >> 8));
    }
  }

  for (along = 1; along < reach; along++)
  {
    int16_t half_q8 = (int16_t)(((reach - along) * 640) / reach);
    int16_t half = (int16_t)((half_q8 + 255) >> 8);
    uint16_t along_fade = (uint16_t)(((reach - along) * 255) / reach);
    int16_t across;

    for (across = (int16_t)-half; across <= half; across++)
    {
      int16_t across_q8 = (int16_t)(((across < 0) ? (int16_t)-across : across) << 8);
      uint16_t across_fade;
      uint16_t glow;

      if (across_q8 > half_q8)
      {
        continue;
      }
      across_fade = (uint16_t)((((uint16_t)half_q8 - (uint16_t)across_q8) * 255U) /
                               ((uint16_t)half_q8 + 1U));
      glow = (uint16_t)(((along_fade * across_fade) >> 8) * scale >> 8);
      for (dir = 0; dir < 4; dir++)
      {
        int16_t sx = star->x;
        int16_t sy = star->y;

        if (dir < 2)
        {
          sx = (int16_t)(sx + ((dir == 0) ? along : (int16_t)-along));
          sy = (int16_t)(sy + across);
        }
        else
        {
          sx = (int16_t)(sx + across);
          sy = (int16_t)(sy + ((dir == 2) ? along : (int16_t)-along));
        }
        LCD_Put(sx, sy, star->hue, glow);
      }
    }
  }
}

static void LCD_FillBackground(void)
{
  /* RGB565 纯蓝，两像素打包。摄像头没起来时保持蓝底，用来确认 LCD 已工作。 */
  volatile uint32_t *fb = (volatile uint32_t *)lcd_draw;
  uint32_t i;
  uint32_t n = (LCD_WIDTH * LCD_HEIGHT) / 2U;
  uint32_t blue = 0x001F001FUL;

  for (i = 0U; i < n; i++)
  {
    fb[i] = blue;
  }
}

static void VolumeRaw_Capture(void);

/* 返回 0 表示已经在场消隐里切到 shown，可以开始往另一块缓冲写。 */
static int LCD_Present(void)
{
  uint32_t shown = lcd_draw;
  uint32_t spin = 20000000U;

  __DSB();
  if (LCD_PANEL_ON != 0U)
  {
    if (HAL_LTDC_SetAddress_NoReload(&hltdc, shown, 0) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING) != HAL_OK)
    {
      Error_Handler();
    }
    /* 必须等到 VBR 真正完成。超时就先不换指针，避免一边显示一边把这块刷黑。 */
    while (((hltdc.Instance->SRCR & LTDC_SRCR_VBR) != 0U) && (spin > 0U))
    {
      spin--;
    }
    if ((hltdc.Instance->SRCR & LTDC_SRCR_VBR) != 0U)
    {
      return -1;
    }
  }
  lcd_frame_addr = shown;
  lcd_draw = (shown == LCD_FB_ADDR) ? LCD_FB_BACK : LCD_FB_ADDR;
  return 0;
}

static void Shutter_Init(void)
{
  volatile uint32_t *mail = (volatile uint32_t *)SHUTTER_MAIL_ADDR;
  mail[0] = SHUTTER_MAIL_MAGIC;
  mail[1] = 0U;
  shutter_armed = 0U;
  shutter_done = 0U;
}

/* 快门或定位冻结时，不再把新的镜头画面写到液晶或换屏。 */
static int CamLcdFrozen(void)
{
  /* Volume must acquire a fresh RAW frame before honoring a shutter hold. */
  return ((meter_hold_fb != 0U) || (shutter_armed != 0U)
      || ((g_meter_mode != METER_MODE_VOLUME) && (shutter_flag != 0U))) ? 1 : 0;
}

static void Shutter_Service(uint8_t allow_present)
{
  if (shutter_flag != 0U)
  {
    if (shutter_armed == 0U)
    {
      if (g_meter_mode == METER_MODE_VOLUME)
      {
        /* A trigger during sliced Run waits for its private buffer lease to
         * end. Never overwrite Run's input or freeze a painted front frame
         * and then use it as recognition input. */
        if (g_meter_raw_diag.busy != 0U) { return; }
        if (g_meter_raw_diag.ready == 0U) { g_meter_raw_diag.request = 1U; }
        VolumeRaw_Capture();
        if (g_meter_raw_diag.ready == 0U) { return; }
      }
      /* 若后缓冲已经写满，先换上这一帧再冻结，更接近按下快门的那一瞬。
       * 正在拷贝识别帧时不允许换屏，避免把正在复制的画面撕开。 */
      if ((allow_present != 0U) && (cam_draw_full == 1U) &&
          (cam_out_jpeg == 0U) && (LCD_PANEL_ON != 0U))
      {
        if (LCD_Present() == 0)
        {
          cam_draw_full = 2U;
        }
      }
      shutter_armed = 1U;
      shutter_done = 0U;
    }
  }
  else if (shutter_armed != 0U)
  {
    shutter_armed = 0U;
    shutter_done = 0U;
    /* 丢弃冻结期间留下的后缓冲，等下一场完整画面再上屏。 */
    if (cam_draw_full == 1U)
    {
      cam_draw_full = 2U;
    }
  }
}

static void LCD_PresentLive(void)
{
  if (shutter_armed != 0U)
  {
    return;
  }
  if (cam_draw_full != 1U)
  {
    return;
  }
  if (g_meter_mode == METER_MODE_VOLUME)
  {
    VolumeRaw_Capture(); /* MUST precede all green/blue/text overlays */
    if (dial_hud_ready != 0U)
    {
      DialRead_PaintHud((uint16_t *)lcd_draw, (uint16_t)LCD_WIDTH, (uint16_t)LCD_HEIGHT,
                        &dial_hud_rep);
      g_meter_raw_diag.paint_gen = g_meter_raw_diag.frame_gen;
      g_meter_raw_diag.live_hud_n++;
    }
  }
  else if (g_meter_mode == METER_MODE_STAR)
  {
    dial_time_t now;
    dial_get_time(&now);
    StarTrack_Paint((uint16_t *)lcd_draw, (uint16_t)LCD_WIDTH, (uint16_t)LCD_HEIGHT,
                    (int16_t)cam_ox, (int16_t)cam_oy, cam_stamp_us, &now, g_star_phase);
  }
  else
  {
    dial_time_t now;
    dial_get_time(&now);
    if ((speed_phase != 0U) && (g_need_refine == 0U))
    {
      const uint16_t *rgb = (const uint16_t *)lcd_draw;
      rgb = rgb + (((uint32_t)cam_oy * LCD_WIDTH) + (uint32_t)cam_ox);
      if (g_speed.hub_locked == 0)
      {
        uint32_t wb = DialRead_WorkBytes((uint16_t)LCD_WIDTH, (uint16_t)LCD_HEIGHT);
        if ((cam_w >= 80U) && (cam_h >= 80U) &&
            ((DIAL_WORK_ADDR + wb) <= (SDRAM_BASE + 0x2000000UL)))
        {
          (void)SpeedTrack_FindNeedle(rgb, (uint16_t)LCD_WIDTH, cam_w, cam_h,
                                      (uint8_t *)DIAL_WORK_ADDR, wb, &g_speed);
        }
      }
      SpeedTrack_Tick(rgb, (uint16_t)LCD_WIDTH, cam_w, cam_h, cam_stamp_us, &g_speed);
    }
    SpeedTrack_Paint((uint16_t *)lcd_draw, (uint16_t)LCD_WIDTH, (uint16_t)LCD_HEIGHT,
                     (int16_t)cam_ox, (int16_t)cam_oy, &g_speed, &now);
  }
  if (LCD_Present() == 0)
  {
    cam_draw_full = 2U;
  }
}

static void LCD_DemoInit(void)
{
  uint32_t tick = HAL_GetTick() / 100U;
  uint32_t i;

  lcd_draw = LCD_FB_ADDR;
  LCD_BuildTables();
  LCD_RngInit();
  for (i = 0U; i < 4U; i++)
  {
    layer_phase[i] = (uint8_t)(tick * layer_step[i]);
    layer_peak[i] = LCD_RandomPeak();
  }
  LCD_BuildStars();
  if (LCD_PANEL_ON == 0U)
  {
    return;
  }
  if (lcd_stars_on == 0U)
  {
    LCD_FillBackground();
    lcd_draw = LCD_FB_BACK;
    LCD_FillBackground();
    /* 前缓冲给屏幕显示，摄像头写到后缓冲，写完一整帧再交换。 */
  }
  LCD_DemoFrame();
}

static void LCD_ClearBoth(void)
{
  uint32_t current = lcd_draw;

  LCD_FillBackground();
  lcd_draw = (current == LCD_FB_ADDR) ? LCD_FB_BACK : LCD_FB_ADDR;
  LCD_FillBackground();
  lcd_draw = current;
}

static void LCD_FocusMark(void)
{
  uint16_t color;
  volatile uint16_t *pix = (volatile uint16_t *)lcd_draw;
  uint32_t i;

  switch (OV5640_FocusState())
  {
    case 1U:
      color = 0x001FU;
      break;
    case 2U:
      color = 0x07E0U;
      break;
    case 3U:
      color = 0xFFE0U;
      break;
    default:
      color = 0xF800U;
      break;
  }
  for (i = 0U; i < (LCD_WIDTH * 12U); i++)
  {
    pix[i] = color;
  }
}

static void LCD_DrawColorGrid(void)
{
  /* 上：白黄青绿。中：品红红蓝黑。下：橙紫灰棕。每格 256x200，铺满 1024x600。 */
  static const uint16_t colors[12] =
  {
    0xFFFFU, 0xFFE0U, 0x07FFU, 0x07E0U,
    0xF81FU, 0xF800U, 0x001FU, 0x0000U,
    0xFD20U, 0x8010U, 0xC618U, 0x8200U
  };
  uint32_t col_w = LCD_WIDTH / 4U;
  uint32_t row_h = LCD_HEIGHT / 3U;
  uint32_t row;
  uint32_t col;

  for (row = 0U; row < 3U; row++)
  {
    for (col = 0U; col < 4U; col++)
    {
      uint16_t color = colors[(row * 4U) + col];
      uint32_t word = ((uint32_t)color << 16) | (uint32_t)color;
      uint32_t y;

      for (y = 0U; y < row_h; y++)
      {
        volatile uint32_t *dst = (volatile uint32_t *)lcd_draw;
        uint32_t x;

        dst += (((row * row_h) + y) * (LCD_WIDTH / 2U)) + (col * (col_w / 2U));
        for (x = 0U; x < (col_w / 2U); x++)
        {
          dst[x] = word;
        }
      }
    }
  }
}

static void LCD_ShowColorGrid(void)
{
  uint32_t shown;

  lcd_draw = LCD_FB_ADDR;
  LCD_DrawColorGrid();
  lcd_draw = LCD_FB_BACK;
  LCD_DrawColorGrid();
  lcd_draw = LCD_FB_ADDR;
  shown = lcd_draw;
  __DSB();
  lcd_frame_addr = shown;
  if (LCD_PANEL_ON != 0U)
  {
    (void)HAL_LTDC_SetAddress_NoReload(&hltdc, shown, 0);
    (void)HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING);
  }
}

static void LCD_DemoFrame(void)
{
  uint32_t tick = HAL_GetTick() / 100U;
  uint16_t alpha[4];
  uint32_t i;
  uint8_t draw = 0U;

  if (lcd_stars_on != 0U)
  {
    LCD_FillBackground();
    for (i = 0U; i < 4U; i++)
    {
      uint8_t phase = (uint8_t)(tick * layer_step[i]);
      uint8_t delta = (uint8_t)(phase - layer_phase[i]);
      uint8_t dist = (uint8_t)(192U - layer_phase[i]);

      if ((delta != 0U) && (dist != 0U) && (dist <= delta))
      {
        layer_peak[i] = LCD_RandomPeak();
      }
      layer_phase[i] = phase;
      alpha[i] = (uint16_t)(((uint32_t)sin_lut[phase] * layer_peak[i]) >> 8);
    }
    for (i = 0U; i < lcd_star_count; i++)
    {
      LCD_DrawStar(&lcd_stars[i],
                   (uint16_t)(((uint32_t)alpha[lcd_stars[i].phase] * lcd_stars[i].gain) >> 8));
    }
    draw = 1U;
  }
  CAM_TakeShot();
  if (LCD_PANEL_ON == 0U)
  {
    uint32_t primask;

    /* 等这一张拍完之前留下刚到的帧。平时清掉，缓冲只留最新的一场。 */
    if (shot_request == 0U)
    {
      primask = __get_PRIMASK();
      __disable_irq();
      cam_ready = 0U;
      if (primask == 0U)
      {
        __enable_irq();
      }
    }
    return;
  }
  /* 拍照这几帧是 JPEG，不能当 RGB565 贴上屏幕。 */
  if (cam_out_jpeg != 0U)
  {
    return;
  }
  if (LCD_COLOR_GRID_ON != 0U)
  {
    static uint8_t grid_shown;

    if (grid_shown == 0U)
    {
      LCD_ShowColorGrid();
      grid_shown = 1U;
    }
    return;
  }
  if (lcd_stars_on == 0U)
  {
    /* 摄像头写后缓冲，整帧完成后立刻换上实时画面。
     * 读数在独立缓冲里分片计算，通过 dial_yield 回到这里换屏。 */
    LCD_PresentLive();
    WEB_Publish();
  }
  if (draw != 0U)
  {
    if (CAM_BIN_ON != 0U)
    {
      volatile uint16_t *mark = (volatile uint16_t *)lcd_draw;
      uint16_t color = NET_StatusColor();
      uint32_t y;
      uint32_t x;

      LCD_FocusMark();
      for (y = (LCD_HEIGHT - 32U); y < (LCD_HEIGHT - 8U); y++)
      {
        for (x = (LCD_WIDTH - 40U); x < (LCD_WIDTH - 8U); x++)
        {
          mark[(y * LCD_WIDTH) + x] = color;
        }
      }
    }
    LCD_Present();
  }
}

/* 原始亮度 0..255 和 CAM_BIN_LEVEL 比。阈值越高，留下的黑色越多。 */
#define CAM_BIN_LEVEL 24U
/* 整帧快门，单位微秒。1ms 是这一帧的总曝光，不是每一行 1ms。 */
#define CAM_SHUTTER_US 1000U
/* 传感器增益寄存器 0x350B。固定值，不走自动增益。 */
#define CAM_GAIN 0xF0U

const uint32_t cam_shutter_us = CAM_SHUTTER_US;
const uint8_t cam_gain = CAM_GAIN;

static uint16_t CAM_BinaryPix(uint16_t pix)
{
  uint32_t r = ((uint32_t)(pix >> 11) & 0x1FU) << 3;
  uint32_t g = ((uint32_t)(pix >> 5) & 0x3FU) << 2;
  uint32_t b = ((uint32_t)pix & 0x1FU) << 3;
  uint32_t y = ((r * 77U) + (g * 150U) + (b * 29U)) >> 8;

  return (y >= CAM_BIN_LEVEL) ? 0xFFFFU : 0x0000U;
}

/* 传感器 JPEG 从 FF D8 到 FF D9。长度不按宽乘高算。 */
static int CAM_TakeJpeg(const uint8_t *buf, uint32_t len, uint32_t *off, uint32_t *size)
{
  uint32_t i;
  uint32_t j;

  if ((buf == 0) || (len < 4U))
  {
    return 0;
  }
  if (len > CAM_JPEG_CAP)
  {
    len = CAM_JPEG_CAP;
  }
  for (i = 0U; ((i + 1U) < len) && (i < 512U); i++)
  {
    if ((buf[i] == 0xFFU) && (buf[i + 1U] == 0xD8U))
    {
      break;
    }
  }
  if ((i >= 512U) || ((i + 1U) >= len))
  {
    return 0;
  }
  for (j = i + 2U; (j + 1U) < len; j++)
  {
    if ((buf[j] == 0xFFU) && (buf[j + 1U] == 0xD9U))
    {
      *off = i;
      *size = (j + 2U) - i;
      return 1;
    }
  }
  return 0;
}

/* 右键按下后，把下一帧完整 JPEG 拷到独立缓冲。网页发送期间不覆盖。 */
static void CAM_TakeShot(void)
{
  const uint8_t *base;
  uint32_t len;
  uint32_t off;
  uint32_t size;
  uint8_t idx;
  static uint8_t jpeg_miss;

  if (cam_out_jpeg == 0U)
  {
    if (shot_request == 0U)
    {
      return;
    }
    /* 平时是 RGB565。右键这一下才切到 JPEG，拍完再切回来。 */
    cam_out_jpeg = 1U;
    jpeg_miss = 0U;
    cam_ready = 0U;
    CAM_Apply();
    return;
  }
  if ((shot_request == 0U) || (cam_ready == 0U) || (web_encode_addr != 0U))
  {
    return;
  }
  idx = cam_done_idx;
  len = cam_cap_len[idx];
  base = (const uint8_t *)((idx == 0U) ? CAM_BUF0 : CAM_BUF1);
  if ((CAM_TakeJpeg(base, len, &off, &size) == 0) || (size < 4U) || (size > PHOTO_MAX))
  {
    cam_ready = 0U;
    jpeg_miss++;
    if (jpeg_miss > 8U)
    {
      shot_request = 0U;
      cam_out_jpeg = 0U;
      CAM_Apply();
    }
    return;
  }
  memcpy((void *)PHOTO_ADDR, base + off, size);
  web_frame_addr = PHOTO_ADDR;
  web_jpeg_len = size;
  web_frame_w = cam_w;
  web_frame_h = cam_h;
  web_snap_seq++;
  shot_request = 0U;
  cam_ready = 0U;
  cam_out_jpeg = 0U;
  CAM_Apply();
}

/* 网页直接发传感器吐出的 JPEG。发出去之前这块缓冲不能被采集盖掉。 */
static void WEB_Publish(void)
{
  const uint8_t *base;
  uint32_t len;
  uint32_t off;
  uint32_t size;
  uint8_t idx;
  uint16_t frame_no;
  static uint16_t held_frame = 0xFFFFU;

  if ((net_need_snap == 0U) || (cam_ok == 0U) || (cam_ready == 0U) || (cam_out_jpeg == 0U))
  {
    return;
  }
  if ((web_encode_addr != 0U) || (cam_jpeg_hold != 0U))
  {
    return;
  }

  frame_no = cam_frame_count;
  if (held_frame == frame_no)
  {
    return;
  }

  idx = cam_done_idx;
  len = cam_cap_len[idx];
  base = (const uint8_t *)((idx == 0U) ? CAM_BUF0 : CAM_BUF1);
  if (CAM_TakeJpeg(base, len, &off, &size) == 0)
  {
    cam_ready = 0U;
    return;
  }
  web_frame_w = cam_w;
  web_frame_h = cam_h;
  web_frame_addr = (uint32_t)base + off;
  web_jpeg_len = size;
  held_frame = frame_no;
  web_snap_seq++;
  net_need_snap = 0U;
  cam_jpeg_hold = 1U;
}

void CAM_ReleaseFrame(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  cam_jpeg_hold = 0U;
  cam_ready = 0U;
  web_jpeg_len = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static int LCD_BlitFit(void)
{
  uint32_t view_w = cam_w;
  uint32_t dst_w;
  uint32_t dst_h;
  uint32_t x0;
  uint32_t y0;
  uint32_t y;
  uint8_t idx;
  uint32_t primask;
  const uint16_t *src;
  volatile uint16_t *dst;

  /* 最后 8 列是传感器的白边，缩小前先丢掉。 */
  if (view_w > 8U)
  {
    view_w -= 8U;
  }
  if (((uint64_t)view_w * LCD_HEIGHT) >= ((uint64_t)cam_h * LCD_WIDTH))
  {
    dst_w = LCD_WIDTH;
    dst_h = (uint32_t)(((uint64_t)cam_h * LCD_WIDTH) / view_w);
  }
  else
  {
    dst_h = LCD_HEIGHT;
    dst_w = (uint32_t)(((uint64_t)view_w * LCD_HEIGHT) / cam_h);
  }
  if (dst_w < 2U)
  {
    dst_w = 2U;
  }
  if (dst_h == 0U)
  {
    dst_h = 1U;
  }
  dst_w &= ~1U;
  if (dst_w > LCD_WIDTH)
  {
    dst_w = LCD_WIDTH;
  }
  if (dst_h > LCD_HEIGHT)
  {
    dst_h = LCD_HEIGHT;
  }
  x0 = (LCD_WIDTH - dst_w) / 2U;
  y0 = (LCD_HEIGHT - dst_h) / 2U;
  idx = cam_done_idx;
  src = (const uint16_t *)((idx == 0U) ? CAM_BUF0 : CAM_BUF1);
  dst = (volatile uint16_t *)lcd_draw;
  for (y = 0U; y < dst_h; y++)
  {
    uint32_t sy = (uint32_t)(((uint64_t)y * cam_h) / dst_h);
    uint32_t x;
    volatile uint16_t *row = dst + ((y0 + y) * LCD_WIDTH) + x0;
    const uint16_t *srow = src + (sy * cam_w);

    for (x = 0U; x < dst_w; x++)
    {
      uint32_t sx = (uint32_t)(((uint64_t)x * view_w) / dst_w);

      row[x] = srow[sx];
    }
  }
  primask = __get_PRIMASK();
  __disable_irq();
  if (cam_done_idx == idx)
  {
    cam_ready = 0U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
  return 1;
}

static int LCD_BlitCamera(void)
{
  uint32_t y;
  uint32_t x0;
  uint32_t y0;
  uint32_t row_words;
  uint8_t idx;
  uint32_t primask;
  const uint32_t *src;
  volatile uint32_t *dst;

  if ((cam_ok == 0U) || (cam_ready == 0U) || (cam_w == 0U) || (cam_h == 0U) || (cam_out_jpeg != 0U))
  {
    return 0;
  }
  if ((cam_w > LCD_WIDTH) || (cam_h > LCD_HEIGHT))
  {
    return LCD_BlitFit();
  }

  idx = cam_done_idx;
  x0 = (LCD_WIDTH - cam_w) / 2U;
  y0 = (LCD_HEIGHT - cam_h) / 2U;
  row_words = cam_w / 2U;
  src = (const uint32_t *)((idx == 0U) ? CAM_BUF0 : CAM_BUF1);
  dst = (volatile uint32_t *)lcd_draw;
  dst += (y0 * (LCD_WIDTH / 2U)) + (x0 / 2U);
  for (y = 0U; y < cam_h; y++)
  {
    uint32_t x;

    if (CAM_BIN_ON != 0U)
    {
      for (x = 0U; x < row_words; x++)
      {
        uint32_t word = src[x];
        uint16_t lo = CAM_BinaryPix((uint16_t)word);
        uint16_t hi = CAM_BinaryPix((uint16_t)(word >> 16));

        dst[x] = ((uint32_t)hi << 16) | (uint32_t)lo;
      }
    }
    else
    {
      for (x = 0U; x < row_words; x++)
      {
        dst[x] = src[x];
      }
    }
    if (row_words >= 5U)
    {
      uint32_t good = dst[row_words - 5U];
      uint16_t pix = (uint16_t)(good >> 16);
      uint32_t fill = ((uint32_t)pix << 16) | (uint32_t)pix;

      dst[row_words - 4U] = fill;
      dst[row_words - 3U] = fill;
      dst[row_words - 2U] = fill;
      dst[row_words - 1U] = fill;
    }
    src += row_words;
    dst += LCD_WIDTH / 2U;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  if (cam_done_idx == idx)
  {
    cam_ready = 0U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
  return 1;
}

static uint8_t cam_dma_inited;
static uint16_t cam_vw;
static uint16_t cam_vh;
static uint8_t cam_cap_bank;
static volatile uint8_t cam_scale_ready;
static volatile uint8_t cam_scale_bank;
static volatile uint16_t cam_scale_w;
static volatile uint16_t cam_scale_h;
static volatile uint16_t cam_scale_vw;
static volatile uint16_t cam_scale_vh;
static volatile uint16_t cam_scale_ox;
static volatile uint16_t cam_scale_oy;

/* No yield/present inside this copy: cam_draw_full==1 leases lcd_draw
 * to the main loop. The line ISR only writes at state 0; frame callbacks
 * leave state 1 alone. IRQs/camera continue, but cannot recycle this buffer.
 * SDRAM is MPU non-cacheable and D-cache is not enabled; no invalidate of
 * CPU-produced DMA-line copies is required (and could discard real writes). */
static uint32_t VolumeRaw_Hash(const uint32_t *p)
{
  uint32_t i, h = 2166136261U;
  for (i = 0U; i < LCD_FB_BYTES / 4U; i++) { h = (h ^ p[i]) * 16777619U; }
  return h;
}

static void VolumeRaw_Capture(void)
{
  uint32_t src, gen, stamp, t0, i, h = 2166136261U;
  const uint32_t *p;
  uint32_t *d = (uint32_t *)DIAL_SNAP_ADDR;
  if ((g_meter_mode != METER_MODE_VOLUME) || (g_meter_switching != 0U)
      || (g_meter_raw_diag.request == 0U) || (g_meter_raw_diag.ready != 0U)
      || (g_meter_raw_diag.busy != 0U) || (cam_draw_full != 1U)) { return; }
  /* Volume restores full-frame 1024x600 RGB565, one sensor row per LCD row.
   * Never mistake an old/scaled/HUD-painted LCD surface for camera RAW. */
  if ((cam_out_jpeg != 0U) || (cam_w != LCD_WIDTH) || (cam_h != LCD_HEIGHT)
      || (cam_vw != cam_w) || (cam_vh != cam_h) || (cam_ox != 0U) || (cam_oy != 0U))
  { return; }
  gen = g_meter_raw_diag.frame_gen;
  if ((gen == 0U) || (g_meter_raw_diag.paint_gen == gen))
  { g_meter_raw_diag.painted_skip++; return; }
  src = lcd_draw; stamp = cam_stamp_us; t0 = TIM2->CNT;
  p = (const uint32_t *)src;
  __DMB();
  for (i = 0U; i < LCD_FB_BYTES / 4U; i++)
  {
    uint32_t v = p[i]; d[i] = v; h = (h ^ v) * 16777619U;
  }
  __DMB();
  g_meter_raw_diag.hold_gen0 = gen;
  g_meter_raw_diag.hold_gen1 = g_meter_raw_diag.frame_gen;
  if ((cam_draw_full != 1U) || (lcd_draw != src) || (cam_stamp_us != stamp)
      || (g_meter_raw_diag.frame_gen != gen) || (g_meter_raw_diag.paint_gen == gen))
  { g_meter_raw_diag.bad_copy++; return; }
  g_meter_raw_diag.source = src;
  g_meter_raw_diag.snap_gen = gen;
  g_meter_raw_diag.snap_stamp = stamp;
  g_meter_raw_diag.hash_saved = h;
  g_meter_raw_diag.copy_us = TIM2->CNT - t0;
  g_meter_raw_diag.copy_n++;
  g_meter_raw_diag.before_hud_n++;
  g_meter_raw_diag.request = 0U;
  __DMB();
  g_meter_raw_diag.ready = 1U; /* publish only after the complete copy */
}

static void CAM_LoadView(void)
{
  uint16_t vx;
  uint16_t vy;
  uint16_t vw;
  uint16_t vh;
  uint16_t cw;
  uint16_t ch;

  OV5640_GetView(&vx, &vy, &vw, &vh);
  OV5640_GetCapture(&cw, &ch);
  cam_ox = vx;
  cam_oy = vy;
  cam_vw = vw;
  cam_vh = vh;
  cam_w = cw;
  cam_h = ch;
  /* 速度模式裁窗后，1:1 实时画面固定放在屏幕右上角，左侧给测量列表。 */
  if (((g_meter_mode == METER_MODE_SPEED) || (g_meter_mode == METER_MODE_STAR)) &&
      (cam_w == cam_vw) && (cam_h == cam_vh) &&
      (cam_w < LCD_WIDTH) && (cam_h < LCD_HEIGHT) &&
      (cam_w >= 80U) && (cam_h >= 80U))
  {
    cam_ox = (uint16_t)(LCD_WIDTH - cam_w);
    cam_oy = 0U;
  }
}

/* 放大不能放在行中断里。300 行拉成 600 行时，写屏幕比行周期长，垂直方向会丢行，画面被压扁。 */
static void CAM_ScaleToLcd(void)
{
  uint16_t sw;
  uint16_t sh;
  uint16_t dw;
  uint16_t dh;
  uint16_t ox;
  uint16_t oy;
  uint16_t y;
  uint32_t step;
  const uint16_t *src;
  uint8_t bank;

  if (cam_scale_ready == 0U)
  {
    return;
  }
  if (CamLcdFrozen() != 0)
  {
    cam_scale_ready = 0U;
    return;
  }
  bank = cam_scale_bank;
  sw = cam_scale_w;
  sh = cam_scale_h;
  dw = cam_scale_vw;
  dh = cam_scale_vh;
  ox = cam_scale_ox;
  oy = cam_scale_oy;
  if ((sw < 2U) || (sh < 2U) || (dw < 2U) || (dh < 2U) ||
      ((uint32_t)ox + dw > LCD_WIDTH) || ((uint32_t)oy + dh > LCD_HEIGHT))
  {
    cam_scale_ready = 0U;
    return;
  }
  step = ((uint32_t)sw << 16) / dw;
  src = (const uint16_t *)(CAM_BUF0 + ((uint32_t)bank * LCD_FB_BYTES));
  for (y = 0U; y < dh; y++)
  {
    uint32_t sy = ((uint32_t)y * sh) / dh;
    const uint16_t *srow = src + (sy * sw);
    volatile uint16_t *drow = (volatile uint16_t *)(LCD_FB_ADDR +
                                                    ((((uint32_t)oy + y) * LCD_WIDTH) + ox) * 2U);
    uint32_t acc = 0U;
    uint16_t x;

    for (x = 0U; x < dw; x++)
    {
      drow[x] = srow[acc >> 16];
      acc += step;
    }
  }
  cam_scale_ready = 0U;
  if ((meter_want_snap != 0U) && (meter_hold_fb == 0U))
  {
    meter_want_snap = 0U;
    meter_hold_fb = 1U;
    meter_frame_ready = 1U;
  }
}

/* 一行 DMA 完成就贴到屏幕上对应的那一行。画面按比例居中，四周留黑。 */
static void CAM_RgbLineIsr(void)
{
  DMA_Stream_TypeDef *dma = DMA1_Stream1;
  const uint32_t *src;
  volatile uint32_t *dst;
  uint32_t i;
  uint16_t line;
  uint16_t y;

  if ((DMA1->LISR & DMA_LISR_TCIF1) == 0U)
  {
    return;
  }
  DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |
                DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
  if (g_meter_switching != 0U) { return; }
  src = ((dma->CR & DMA_SxCR_CT) != 0U) ? cam_line_buf[0] : cam_line_buf[1];
  if (g_meter_mode == METER_MODE_STAR)
  {
    /* 梅花轮：每一行都按圆环采样，不管这一帧是否上屏，帧率按传感器算。 */
    StarTrack_IsrLine(src);
  }
  line = cam_curline;
  if ((cam_w == cam_vw) && (cam_h == cam_vh))
  {
    y = (uint16_t)(cam_oy + line);
    if ((line < cam_h) && (y < LCD_HEIGHT))
    {
      if ((CamLcdFrozen() == 0) && (cam_draw_full == 0U))
      {
        dst = (volatile uint32_t *)(lcd_draw + ((uint32_t)y * LCD_WIDTH * 2U) +
                                    ((uint32_t)cam_ox * 2U));
        for (i = 0U; i < ((uint32_t)cam_w / 2U); i++)
        {
          dst[i] = src[i];
        }
        cam_curline = (uint16_t)(line + 1U);
      }
    }
  }
  else if ((line < cam_h) && (cam_w >= 2U) && ((cam_w & 1U) == 0U))
  {
    uint32_t *cap = (uint32_t *)(CAM_BUF0 + ((uint32_t)cam_cap_bank * LCD_FB_BYTES) +
                                 ((uint32_t)line * cam_w * 2U));
    uint32_t n = (uint32_t)cam_w / 2U;

    for (i = 0U; i < n; i++)
    {
      cap[i] = src[i];
    }
    cam_curline = (uint16_t)(line + 1U);
    if (cam_curline >= cam_h)
    {
      if (cam_scale_ready == 0U)
      {
        cam_scale_bank = cam_cap_bank;
        cam_scale_w = cam_w;
        cam_scale_h = cam_h;
        cam_scale_vw = cam_vw;
        cam_scale_vh = cam_vh;
        cam_scale_ox = cam_ox;
        cam_scale_oy = cam_oy;
        cam_scale_ready = 1U;
        cam_cap_bank ^= 1U;
      }
      cam_curline = 0U;
    }
  }
  cam_dma_count++;
}

/* 场中断落在消隐里。把按行 DMA 重新对准行缓冲开头，下一场从像素 0 开始收。
 * 只停 DMA，不停 DCMI，避免连续模式把下一场一起掐掉。 */
static void CAM_ResyncRgbDma(void)
{
  DMA_Stream_TypeDef *dma = (DMA_Stream_TypeDef *)hdma_dcmi.Instance;
  uint32_t spin;
  uint32_t ndtr = (uint32_t)cam_w / 2U;

  cam_curline = 0U;
  if ((dma == 0) || (ndtr == 0U))
  {
    return;
  }
  __HAL_DMA_DISABLE_IT(&hdma_dcmi, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT | DMA_IT_FE);
  dma->CR &= ~DMA_SxCR_EN;
  for (spin = 0U; spin < 8000U; spin++)
  {
    if ((dma->CR & DMA_SxCR_EN) == 0U)
    {
      break;
    }
  }
  dma->NDTR = ndtr;
  dma->M0AR = (uint32_t)cam_line_buf[0];
  dma->M1AR = (uint32_t)cam_line_buf[1];
  dma->CR &= ~DMA_SxCR_CT;
  DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |
                DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
  hdma_dcmi.ErrorCode = HAL_DMA_ERROR_NONE;
  hdma_dcmi.State = HAL_DMA_STATE_BUSY;
  __HAL_UNLOCK(&hdma_dcmi);
  dma->FCR &= ~DMA_SxFCR_FEIE;
  __HAL_DMA_ENABLE_IT(&hdma_dcmi, DMA_IT_TC);
  dma->CR |= DMA_SxCR_EN;
}

/* 正点原子 RGB 屏：双缓冲按行收，DCMI 不停。一场开始时行号回到 0。 */
static int CAM_StartRgbLine(void)
{
  DMA_Stream_TypeDef *dma;
  uint32_t spin;

  CAM_LoadView();
  if ((cam_w < 2U) || (cam_w > LCD_WIDTH) || ((cam_w & 1U) != 0U) ||
      (cam_vw < 2U) || (cam_vw > LCD_WIDTH))
  {
    cam_w = LCD_WIDTH;
    cam_vw = LCD_WIDTH;
    cam_ox = 0U;
  }
  if ((cam_h < 2U) || (cam_h > LCD_HEIGHT) || (cam_vh < 2U) || (cam_vh > LCD_HEIGHT))
  {
    cam_h = LCD_HEIGHT;
    cam_vh = LCD_HEIGHT;
    cam_oy = 0U;
  }
  /* 不要在重启采集时整块刷黑。刷 1024x600 会占很久 SDRAM，LTDC 容易欠载，整屏就低频闪黑闪白。 */
  cam_curline = 0U;
  cam_cap_bank = 0U;
  cam_scale_ready = 0U;
  cam_row = 0U;
  hdcmi.Instance->CR &= ~DCMI_CR_CAPTURE;
  for (spin = 0U; spin < 2000U; spin++)
  {
    if ((hdcmi.Instance->CR & DCMI_CR_CAPTURE) == 0U)
    {
      break;
    }
  }
  __HAL_DCMI_DISABLE(&hdcmi);
  hdcmi.Instance->CR &= ~(DCMI_CR_CROP | DCMI_CR_JPEG | DCMI_CR_CM);
  hdcmi.State = HAL_DCMI_STATE_READY;
  __HAL_UNLOCK(&hdcmi);
  dma = (DMA_Stream_TypeDef *)hdma_dcmi.Instance;
  if (dma != 0)
  {
    dma->CR &= ~DMA_SxCR_EN;
    for (spin = 0U; spin < 2000U; spin++)
    {
      if ((dma->CR & DMA_SxCR_EN) == 0U)
      {
        break;
      }
    }
  }
  __HAL_UNLOCK(&hdma_dcmi);
  (void)HAL_DMA_DeInit(&hdma_dcmi);
  hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_dcmi.Init.Mode = DMA_CIRCULAR;
  hdma_dcmi.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  hdma_dcmi.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;
  hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dcmi.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dcmi.Init.Request = DMA_REQUEST_DCMI;
  if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
  {
    return -1;
  }
  __HAL_LINKDMA(&hdcmi, DMA_Handle, hdma_dcmi);
  __HAL_UNLOCK(&hdma_dcmi);
  if (HAL_DMAEx_MultiBufferStart(&hdma_dcmi, (uint32_t)&DCMI->DR,
                                 (uint32_t)cam_line_buf[0], (uint32_t)cam_line_buf[1],
                                 (uint32_t)cam_w / 2U) != HAL_OK)
  {
    return -1;
  }
  __HAL_DMA_ENABLE_IT(&hdma_dcmi, DMA_IT_TC);
  /* 满突发时 FIFO 空错误会一直进中断。这个错误不影响这一行的搬运。 */
  ((DMA_Stream_TypeDef *)hdma_dcmi.Instance)->FCR &= ~DMA_SxFCR_FEIE;
  DMA1->LIFCR = DMA_LIFCR_CFEIF1 | DMA_LIFCR_CTCIF1;
  /* CWSIZE 写入的是个数减 1。8 位口上一个 RGB565 像素是两个像素时钟。
   * 行尾多出来的时钟在这里丢掉，避免按像素计数的行 DMA 和 HSYNC 慢慢错位，画面横漂。 */
  if ((cam_w >= 2U) && (cam_h >= 2U))
  {
    (void)HAL_DCMI_ConfigCrop(&hdcmi, 0U, 0U,
                              ((uint32_t)cam_w * 2U) - 1U,
                              (uint32_t)cam_h - 1U);
    (void)HAL_DCMI_EnableCrop(&hdcmi);
  }
  __HAL_DCMI_ENABLE(&hdcmi);
  __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_LINE | DCMI_IT_VSYNC | DCMI_IT_ERR | DCMI_IT_OVR);
  __HAL_DCMI_CLEAR_FLAG(&hdcmi, DCMI_FLAG_FRAMERI | DCMI_FLAG_OVRRI | DCMI_FLAG_ERRRI |
                                DCMI_FLAG_LINERI | DCMI_FLAG_VSYNCRI);
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
  hdcmi.Instance->CR |= DCMI_CR_CAPTURE;
  return 0;
}

static int CAM_Kick(void)
{
  uint32_t dest = (cam_write_idx == 0U) ? CAM_BUF0 : CAM_BUF1;

  if (hdcmi.Init.JPEGMode != DCMI_JPEG_ENABLE)
  {
    return CAM_StartRgbLine();
  }
  uint32_t length = CAM_JPEG_CAP / 4U;
  uint32_t mode = DCMI_MODE_CONTINUOUS;
  uint32_t spin;

  if (hdcmi.Init.JPEGMode != DCMI_JPEG_ENABLE)
  {
    length = ((uint32_t)cam_w * (uint32_t)cam_h) / 2U;
    /* 连续采集，场同步一到就交这一幅。快照会在 DMA 长度和实际帧对不上时一直等下去。 */
    mode = DCMI_MODE_CONTINUOUS;
    {
      uint32_t chunk = length;
      uint32_t count = 1U;

      while (chunk > 0xFFFFU)
      {
        chunk >>= 1;
        count <<= 1;
      }
      cam_chunks_need = (count < 2U) ? 1U : (uint16_t)count;
      cam_chunks = 0U;
    }
  }

  (void)cam_line_buf;
  cam_row = 0U;
  hdcmi.Instance->CR &= ~DCMI_CR_CAPTURE;
  for (spin = 0U; spin < 2000U; spin++)
  {
    if ((hdcmi.Instance->CR & DCMI_CR_CAPTURE) == 0U)
    {
      break;
    }
  }
  __HAL_DCMI_DISABLE(&hdcmi);
  hdcmi.State = HAL_DCMI_STATE_READY;
  __HAL_UNLOCK(&hdcmi);
  __HAL_UNLOCK(&hdma_dcmi);
  if (cam_dma_inited == 0U)
  {
    hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dcmi.Init.Mode = DMA_NORMAL;
    hdma_dcmi.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_dcmi.Init.MemBurst = DMA_MBURST_INC4;
    hdma_dcmi.Init.PeriphBurst = DMA_PBURST_INC4;
    cam_dma_inited = 1U;
  }
  if (hdma_dcmi.State != HAL_DMA_STATE_READY)
  {
    (void)HAL_DMA_DeInit(&hdma_dcmi);
    if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
    {
      return -1;
    }
  }
  __HAL_LINKDMA(&hdcmi, DMA_Handle, hdma_dcmi);
  cam_dma_words = length;
  if (hdcmi.Init.JPEGMode != DCMI_JPEG_ENABLE)
  {
    /* 8 位口上一个 RGB565 像素是两个时钟。多出来的行尾时钟在这里丢掉，避免拆成横纹。 */
    (void)HAL_DCMI_ConfigCrop(&hdcmi, 0U, 0U, (uint32_t)cam_w * 2U, (uint32_t)cam_h);
    (void)HAL_DCMI_EnableCrop(&hdcmi);
  }
  else
  {
    (void)HAL_DCMI_DisableCrop(&hdcmi);
  }
  if (HAL_DCMI_Start_DMA(&hdcmi, mode, dest, length) != HAL_OK)
  {
    return -1;
  }
  /* 行中断和溢出中断都不开。停 DMA 时的溢出不能当成故障去重配传感器。
     场同步要马上开：JPEG 比缓冲短，RGB565 也得靠它在帧尾收尾。 */
  __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_LINE | DCMI_IT_VSYNC | DCMI_IT_ERR | DCMI_IT_OVR);
  __HAL_DCMI_CLEAR_FLAG(&hdcmi, DCMI_FLAG_FRAMERI | DCMI_FLAG_OVRRI | DCMI_FLAG_ERRRI |
                                DCMI_FLAG_LINERI | DCMI_FLAG_VSYNCRI);
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
  /* 满突发时 FIFO 空错误会一直进中断，把场同步饿死，画面就停在第一帧。这个错误不影响搬运。 */
  ((DMA_Stream_TypeDef *)hdma_dcmi.Instance)->FCR &= ~DMA_SxFCR_FEIE;
  DMA1->LIFCR = DMA_LIFCR_CFEIF1;
  return 0;
}

static void CAM_Start(void)
{
  if (CAM_Kick() != 0)
  {
    cam_arm = 1U;
  }
}

void CAM_DmaIsr(void)
{
  if (hdcmi.Init.JPEGMode != DCMI_JPEG_ENABLE)
  {
    CAM_RgbLineIsr();
    return;
  }
  if ((DMA1->LISR & DMA_LISR_TCIF1) != 0U)
  {
    cam_chunks++;
  }
  cam_dma_count++;
  HAL_DMA_IRQHandler(&hdma_dcmi);
}

/* 场同步落在消隐里。只把 DMA 指到下一块缓冲，DCMI 不停，避免连续模式把下一场一起掐掉。 */
static void CAM_RearmRgb(uint32_t dest)
{
  DMA_Stream_TypeDef *dma = (DMA_Stream_TypeDef *)hdma_dcmi.Instance;
  uint32_t length = ((uint32_t)cam_w * (uint32_t)cam_h) / 2U;
  uint32_t chunk = length;
  uint32_t count = 1U;
  uint32_t spin;

  if (dma == 0)
  {
    return;
  }
  __HAL_DMA_DISABLE_IT(&hdma_dcmi, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT | DMA_IT_FE);
  dma->CR &= ~DMA_SxCR_EN;
  for (spin = 0U; spin < 8000U; spin++)
  {
    if ((dma->CR & DMA_SxCR_EN) == 0U)
    {
      break;
    }
  }
  while (chunk > 0xFFFFU)
  {
    chunk >>= 1;
    count <<= 1;
  }
  hdcmi.pBuffPtr = dest;
  hdcmi.XferSize = chunk;
  cam_dma_words = length;
  cam_chunks = 0U;
  if (count < 2U)
  {
    hdcmi.XferCount = 0U;
    hdcmi.XferTransferNumber = 0U;
    cam_chunks_need = 1U;
    dma->CR &= ~DMA_SxCR_DBM;
    dma->NDTR = length;
    dma->M0AR = dest;
  }
  else
  {
    hdcmi.XferCount = count - 2U;
    hdcmi.XferTransferNumber = hdcmi.XferCount;
    cam_chunks_need = (uint16_t)count;
    dma->CR |= DMA_SxCR_DBM;
    dma->NDTR = chunk;
    dma->M0AR = dest;
    dma->M1AR = dest + (chunk * 4U);
  }
  DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |
                DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
  hdma_dcmi.ErrorCode = HAL_DMA_ERROR_NONE;
  hdma_dcmi.State = HAL_DMA_STATE_BUSY;
  __HAL_UNLOCK(&hdma_dcmi);
  __HAL_DMA_ENABLE_IT(&hdma_dcmi, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
  dma->FCR &= ~DMA_SxFCR_FEIE;
  dma->CR |= DMA_SxCR_EN;
  __HAL_DCMI_CLEAR_FLAG(&hdcmi, DCMI_FLAG_FRAMERI | DCMI_FLAG_OVRRI | DCMI_FLAG_ERRRI);
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *dcmi)
{
  DMA_Stream_TypeDef *dma = (DMA_Stream_TypeDef *)hdma_dcmi.Instance;
  uint32_t spin;
  uint32_t left;
  uint32_t got;
  uint8_t idx;

  (void)dcmi;
  if (g_meter_switching != 0U) { return; }
  cam_frame_count++;
  if (hdcmi.Init.JPEGMode != DCMI_JPEG_ENABLE)
  {
    if (g_meter_mode == METER_MODE_STAR)
    {
      StarTrack_IsrFrame(TIM2->CNT, cam_h);
    }
    /* 2：上一场已经上屏，这一场开始再写后缓冲。1：后缓冲写满，等主循环换屏。 */
    if ((cam_draw_full == 2U) && (CamLcdFrozen() == 0))
    {
      cam_draw_full = 0U;
    }
    else if ((cam_draw_full == 0U) && (cam_curline >= cam_h) && (cam_h != 0U))
    {
      cam_stamp_us = TIM2->CNT;
      if (g_meter_mode == METER_MODE_VOLUME)
      {
        g_meter_raw_diag.frame_gen++;
        if (g_meter_raw_diag.frame_gen == 0U) { g_meter_raw_diag.frame_gen++; }
      }
      __DMB();
      cam_draw_full = 1U;
    }
    CAM_ResyncRgbDma();
    __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
    return;
  }
  hdcmi.Instance->CR &= ~DCMI_CR_CAPTURE;
  for (spin = 0U; spin < 4000U; spin++)
  {
    if ((hdcmi.Instance->CR & DCMI_CR_CAPTURE) == 0U)
    {
      break;
    }
  }
  if (dma != 0)
  {
    /* 先关掉 DMA 中断再停流，避免半帧被当成错误去重开传感器。 */
    __HAL_DMA_DISABLE_IT(&hdma_dcmi, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT | DMA_IT_FE);
    dma->CR &= ~DMA_SxCR_EN;
    for (spin = 0U; spin < 4000U; spin++)
    {
      if ((dma->CR & DMA_SxCR_EN) == 0U)
      {
        break;
      }
    }
    dma->CR &= ~DMA_SxCR_DBM;
    left = dma->NDTR;
    __HAL_DMA_CLEAR_FLAG(&hdma_dcmi, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_dcmi));
    __HAL_DMA_CLEAR_FLAG(&hdma_dcmi, __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_dcmi));
    __HAL_DMA_CLEAR_FLAG(&hdma_dcmi, __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_dcmi));
    __HAL_DMA_CLEAR_FLAG(&hdma_dcmi, __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_dcmi));
    __HAL_DMA_CLEAR_FLAG(&hdma_dcmi, __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_dcmi));
    hdma_dcmi.ErrorCode = HAL_DMA_ERROR_NONE;
    hdma_dcmi.State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(&hdma_dcmi);
  }
  else
  {
    left = cam_dma_words;
  }
  if (left > cam_dma_words)
  {
    left = cam_dma_words;
  }
  got = (cam_dma_words - left) * 4U;
  if (hdcmi.Init.JPEGMode != DCMI_JPEG_ENABLE)
  {
    /* 整幅 RGB565 比一个 DMA 计数器长，长度按窗口算，不看当前这一段的剩余。 */
    got = (uint32_t)cam_w * (uint32_t)cam_h * 2U;
  }
  if (cam_hold == 0U)
  {
    idx = cam_write_idx;
    cam_cap_len[idx] = got;
    if (cam_ready == 0U)
    {
      cam_done_idx = idx;
      cam_ready = 1U;
      cam_write_idx = (uint8_t)(idx ^ 1U);
    }
  }
  hdcmi.State = HAL_DCMI_STATE_READY;
  __HAL_UNLOCK(&hdcmi);
  /* RGB565 必须在本中断里把下一帧的 DMA 拉起来。等主循环贴完再开，传感器已经在吐下一帧，会溢出然后整表重配，画面就停住。 */
  if ((cam_hold == 0U) && (hdcmi.Init.JPEGMode != DCMI_JPEG_ENABLE))
  {
    if (CAM_Kick() != 0)
    {
      cam_arm = 1U;
    }
    else
    {
      cam_arm = 0U;
    }
  }
  else
  {
    /* 重开 DMA 放到主循环。这里用 HAL_GetTick 会死等，SysTick 比本中断优先级低。 */
    cam_arm = 1U;
  }
}

static void CAM_Apply(void)
{
  uint32_t primask;

  if (cam_ok == 0U)
  {
    return;
  }
  MeterLocate_Resume();
  primask = __get_PRIMASK();
  __disable_irq();
  cam_hold = 1U;
  if (primask == 0U)
  {
    __enable_irq();
  }
  (void)HAL_DCMI_Stop(&hdcmi);
  if (cam_out_jpeg != 0U)
  {
    hdcmi.Init.JPEGMode = DCMI_JPEG_ENABLE;
    hdcmi.Instance->CR |= DCMI_CR_JPEG;
  }
  else
  {
    hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
    hdcmi.Instance->CR &= ~DCMI_CR_JPEG;
  }
  if (cam_out_jpeg != 0U)
  {
    OV5640_SetJpeg(1U);
    OV5640_ApplyView();
    OV5640_GetSize(&cam_w, &cam_h);
  }
  else
  {
    OV5640_SetJpeg(0U);
    OV5640_RgbLcdPreview(LCD_WIDTH, LCD_HEIGHT);
    CAM_LoadView();
  }
  HAL_Delay(30);
  CAM_Start();
  cam_hold = 0U;
}


/* 在画面顶部画一条状态色条，方便在屏上看到定位进度。 */
static void MeterLocate_Banner(uint16_t color)
{
  volatile uint16_t *fb = (volatile uint16_t *)LCD_FB_ADDR;
  uint32_t x;
  uint32_t y;

  for (y = 8U; y < 22U; y++)
  {
    for (x = 8U; x < 220U; x++)
    {
      fb[(y * LCD_WIDTH) + x] = color;
    }
  }
}

static void MeterLocate_Resume(void)
{
  uint8_t i;

  meter_hold_fb = 0U;
  meter_want_snap = 0U;
  meter_frame_ready = 0U;
  meter_phase = 0U;
  meter_nptrs = 0U;
  meter_skip_frames = 10U;
  meter_refind_cd = 0U;
  for (i = 0U; i < METER_MAX_POINTERS; i++)
  {
    meter_ptrs[i].locked = 0U;
    meter_ptrs[i].have_ang = 0U;
  }
}

/* RTC calendar (Asia/Shanghai, 24-hour). Time is loaded once per firmware
 * stamp so a later reset keeps ticking if the backup domain stays powered. */
RTC_HandleTypeDef hrtc;
static uint8_t rtc_ready;

#ifndef RTC_SET_YEAR
#define RTC_SET_YEAR  2026U
#define RTC_SET_MON  11U
#define RTC_SET_DAY  11U
#define RTC_SET_HOUR  11U
#define RTC_SET_MIN  10U
#define RTC_SET_SEC  40U
#define RTC_SET_WDAY  RTC_WEEKDAY_WEDNESDAY
#define RTC_SET_STAMP  0x9BA264C6UL
#endif

void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc_)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_PeriphCLKInitTypeDef per = {0};

  (void)hrtc_;
  /* PWR clock is already on from SystemClock_Config. */
  HAL_PWR_EnableBkUpAccess();

  osc.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  osc.LSEState = RCC_LSE_ON;
  osc.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&osc) == HAL_OK)
  {
    per.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  }
  else
  {
    osc.OscillatorType = RCC_OSCILLATORTYPE_LSI;
    osc.LSIState = RCC_LSI_ON;
    osc.LSEState = RCC_LSE_OFF;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
      return;
    }
    per.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  }
  per.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  if (HAL_RCCEx_PeriphCLKConfig(&per) != HAL_OK)
  {
    return;
  }
  __HAL_RCC_RTC_ENABLE();
}

static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    rtc_ready = 0U;
    return;
  }
  rtc_ready = 1U;

  /* New stamp: this flash asked to 授时, so load the compiled Beijing time. */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != RTC_SET_STAMP)
  {
    sTime.Hours = (uint8_t)RTC_SET_HOUR;
    sTime.Minutes = (uint8_t)RTC_SET_MIN;
    sTime.Seconds = (uint8_t)RTC_SET_SEC;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    sDate.WeekDay = (uint8_t)RTC_SET_WDAY;
    sDate.Month = (uint8_t)RTC_SET_MON;
    sDate.Date = (uint8_t)RTC_SET_DAY;
    sDate.Year = (uint8_t)(RTC_SET_YEAR - 2000U);
    (void)HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    (void)HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, RTC_SET_STAMP);
  }
}

void dial_get_time(dial_time_t *t)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  if (rtc_ready == 0U)
  {
    t->year = (uint16_t)RTC_SET_YEAR;
    t->mon = (uint8_t)RTC_SET_MON;
    t->day = (uint8_t)RTC_SET_DAY;
    t->hour = (uint8_t)RTC_SET_HOUR;
    t->min = (uint8_t)RTC_SET_MIN;
    t->sec = (uint8_t)RTC_SET_SEC;
    t->msec = 0U;
    return;
  }
  (void)HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  (void)HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  t->year = (uint16_t)(2000U + (uint16_t)sDate.Year);
  t->mon = sDate.Month;
  t->day = sDate.Date;
  t->hour = sTime.Hours;
  t->min = sTime.Minutes;
  t->sec = sTime.Seconds;
  {
    uint32_t frac = (uint32_t)sTime.SecondFraction;
    uint32_t sub = (uint32_t)sTime.SubSeconds;
    if ((frac == 0U) || (sub > frac))
    {
      t->msec = 0U;
    }
    else
    {
      t->msec = (uint16_t)(((frac - sub) * 1000U) / (frac + 1U));
    }
  }
}

/* SUSPECT frame archive (SD / QSPI flash). Name is YYYYMMDD_HHMMSS. */
int dial_save_suspect_frame(const uint16_t *frame, uint16_t w, uint16_t h, const char *name)
{
  (void)frame;
  (void)w;
  (void)h;
  (void)name;
  /* TODO: write RGB565 frame to SD/Flash as <name>.raw or .bmp */
  return -1;
}

/* TIM2 free-runs at 1 MHz (1 µs per count). Volume HUD interval still uses RTC.
 * Speed tracking uses this stamp only; do not use RTC for flow or tooth rate. */
TIM_HandleTypeDef htim2;

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
}

static void MX_TIM2_Init(void)
{
  uint32_t pclk = HAL_RCC_GetPCLK1Freq();
  uint32_t tclk = pclk;

  /* APB1 prescaler is not 1, so TIM2 kernel clock is 2 * PCLK1. */
  if (HAL_RCC_GetHCLKFreq() != pclk)
  {
    tclk = pclk * 2U;
  }
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (tclk / 1000000U) - 1U;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFFU;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    return;
  }
  (void)HAL_TIM_Base_Start(&htim2);
}

static uint32_t tim2_us(void)
{
  return __HAL_TIM_GET_COUNTER(&htim2);
}

static uint32_t tim2_ms(void)
{
  return __HAL_TIM_GET_COUNTER(&htim2) / 1000U;
}

/* Milliseconds since midnight, from the same RTC fields as the HUD date. */
static uint32_t dial_tod_ms(const dial_time_t *t)
{
  return (((uint32_t)t->hour * 60U + (uint32_t)t->min) * 60U + (uint32_t)t->sec) * 1000U
         + (uint32_t)t->msec;
}

static void dial_fill_interval(dial_report_t *rep, uint32_t dt, uint8_t have)
{
  char tmp[12];
  char *q;
  uint32_t v;
  int n;
  int i;

  if (rep == 0)
  {
    return;
  }
  if (have == 0U)
  {
    rep->line3[0] = 0;
    rep->interval_ms = 0U;
    return;
  }
  rep->interval_ms = dt;
  q = rep->line1;
  while (*q != 0)
  {
    q++;
  }
  /* leave a visible gap after the timestamp on the same line */
  for (i = 0; i < 4; i++)
  {
    *q++ = ' ';
  }
  v = dt;
  n = 0;
  if (v == 0U)
  {
    tmp[n++] = '0';
  }
  else
  {
    while ((v != 0U) && (n < 10))
    {
      tmp[n++] = (char)('0' + (v % 10U));
      v /= 10U;
    }
  }
  while (n > 0)
  {
    n--;
    *q++ = tmp[n];
  }
  *q++ = ' ';
  *q++ = 'm';
  *q++ = 's';
  /* fallback / track code after ms: 0 = fast path ok */
  *q++ = ' ';
  *q++ = ' ';
  v = (uint32_t)rep->track_fb;
  n = 0;
  if (v == 0U)
  {
    tmp[n++] = '0';
  }
  else
  {
    while ((v != 0U) && (n < 10))
    {
      tmp[n++] = (char)('0' + (v % 10U));
      v /= 10U;
    }
  }
  while (n > 0)
  {
    n--;
    *q++ = tmp[n];
  }
  *q = 0;
  /* keep line3 empty so nothing is drawn on a third row */
  rep->line3[0] = 0;
}

void dial_yield(void)
{
  static uint32_t last_ms;
  uint32_t now;

  if (dial_yield_lock != 0U)
  {
    return;
  }
  /* Keep the live picture moving, but do not rescale and flip the LCD on
   * every algorithm row: that stretched a few seconds of compute past 20 s. */
  now = HAL_GetTick();
  if ((now - last_ms) < 33U)
  {
    return;
  }
  last_ms = now;
  dial_yield_lock = 1U;
  Shutter_Service(0U);
  CAM_ScaleToLcd();
  LCD_PresentLive();
  dial_yield_lock = 0U;
}

static void DialRead_FinishReport(dial_report_t *work_rep, dial_time_t *prev_ts, uint8_t *have_prev)
{
  uint8_t k;
  char *br;
  /* 只适配“新测试”的显示序号，不改四指针算法内部累计诊断计数。 */
  if (g_meter_volume_seq < 999U) { g_meter_volume_seq++; } else { g_meter_volume_seq = 1U; }
  br = strchr(work_rep->line1, '[');
  if ((br != 0) && (br[1] != 0) && (br[2] != 0) && (br[3] != 0) && (br[4] == ']'))
  {
    br[1] = (char)('0' + ((g_meter_volume_seq / 100U) % 10U));
    br[2] = (char)('0' + ((g_meter_volume_seq / 10U) % 10U));
    br[3] = (char)('0' + (g_meter_volume_seq % 10U));
  }
  g_meter_volume_diag.seq = g_meter_volume_seq;
  g_meter_volume_diag.status = (uint32_t)work_rep->status;
  g_meter_volume_diag.flags = work_rep->suspect_flags;
  g_meter_volume_diag.n_dials = work_rep->n_dials;
  g_meter_volume_diag.n_scale_bad = work_rep->n_scale_bad;
  g_meter_volume_diag.n_cand = g_dial_dbg.n_cand;
  g_meter_volume_diag.miss_place = g_dial_dbg.miss_place;
  for (k = 0U; k < 4U; k++)
  {
    g_meter_volume_diag.reason[k] = g_dial_dbg.last_reason[k];
    g_meter_volume_diag.cx[k] = g_dial_dbg.last_cx[k];
    g_meter_volume_diag.cy[k] = g_dial_dbg.last_cy[k];
  }
  for (k = 0U; k < 8U; k++) { g_meter_volume_diag.cand[k] = g_dial_dbg.cand[k]; }

  if (*have_prev != 0U)
  {
    uint32_t now_ms = dial_tod_ms(&work_rep->ts);
    uint32_t prev_ms = dial_tod_ms(prev_ts);
    uint32_t dt;
    if (now_ms >= prev_ms)
    {
      dt = now_ms - prev_ms;
    }
    else
    {
      dt = (24U * 60U * 60U * 1000U) - prev_ms + now_ms;
    }
    dial_fill_interval(work_rep, dt, 1U);
  }
  else
  {
    dial_fill_interval(work_rep, 0U, 0U);
    *have_prev = 1U;
  }
  dial_hud_rep = *work_rep;
  *prev_ts = work_rep->ts;
  dial_hud_ready = 1U;
}


static void LCD_FillBothBlack(void)
{
  uint32_t cur = lcd_draw;
  lcd_draw = LCD_FB_ADDR;
  LCD_FillBackground();
  lcd_draw = LCD_FB_BACK;
  LCD_FillBackground();
  lcd_draw = cur;
}

/* 升针常在居中预览的左缘之外。速度模式把全幅向左平移，第一刀仍在预览内裁。 */
#define SPEED_PREVIEW_SRC_X  (240)

static void SpeedTrack_PreviewLeft(void)
{
  OV5640_ShowOnLcd((int16_t)SPEED_PREVIEW_SRC_X, 0, LCD_WIDTH, LCD_HEIGHT);
}

static void SpeedTrack_OpenFull(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  cam_hold = 1U;
  if (primask == 0U)
  {
    __enable_irq();
  }
  (void)HAL_DCMI_Stop(&hdcmi);
  cam_arm = 0U;
  cam_curline = 0U;
  cam_draw_full = 2U;
  LCD_FillBothBlack();
  ov5640_pclk_hz = 2625000000UL / (6U * 4U);
  ov5640_native_fps = 50U;
  OV5640_PowerCycle();
  if (OV5640_SensorInit() != 0)
  {
    cam_ok = 0U;
    cam_hold = 0U;
    speed_phase = 0U;
    g_crop_settling = 0U;
    CamDbg_Mark(8U);
    return;
  }
  OV5640_RgbLcdPreview(LCD_WIDTH, LCD_HEIGHT);
  SpeedTrack_PreviewLeft();
  CAM_LoadView();
  HAL_Delay(20);
  CAM_Start();
  OV5640_FocusLoad();
  HAL_Delay(20);
  CamDbg_SnapSensor();
  HAL_Delay(80);
  CamDbg_SnapLive();
  cam_hold = 0U;
  SpeedTrack_Reset(&g_speed);
  speed_phase = 0U;
  g_crop_settling = 1U;
  g_crop_settle_t0 = TIM2->CNT;
  g_cam_settle_us = FULL_SETTLE_US;
  CamDbg_Mark(8U);
}

static void SpeedTrack_ApplyRefine(int16_t rox, int16_t roy, uint16_t rnw, uint16_t rnh)
{
  uint16_t win_x = 0U, win_y = 0U, win_w = 0U, win_h = 0U;
  uint16_t view_x = 0U, view_y = 0U, view_w = 0U, view_h = 0U;
  uint16_t cap_w = 0U, cap_h = 0U;
  int32_t sx;
  int32_t sy;
  int32_t sw;
  int32_t sh;

  OV5640_GetWindow(&win_x, &win_y, &win_w, &win_h);
  OV5640_GetView(&view_x, &view_y, &view_w, &view_h);
  OV5640_GetCapture(&cap_w, &cap_h);
  if (cap_w < 1U) { cap_w = 1U; }
  if (cap_h < 1U) { cap_h = 1U; }
  sx = (int32_t)win_x + (((int32_t)rox * (int32_t)win_w) / (int32_t)cap_w);
  sy = (int32_t)win_y + (((int32_t)roy * (int32_t)win_h) / (int32_t)cap_h);
  sw = ((int32_t)rnw * (int32_t)win_w) / (int32_t)cap_w;
  sh = ((int32_t)rnh * (int32_t)win_h) / (int32_t)cap_h;
  if (sx < 0) { sx = 0; }
  if (sy < 0) { sy = 0; }
  if (sw < 80) { sw = 80; }
  if (sh < 80) { sh = 80; }
  if (sw > (int32_t)LCD_WIDTH) { sw = (int32_t)LCD_WIDTH; }
  if (sh > (int32_t)LCD_HEIGHT) { sh = (int32_t)LCD_HEIGHT; }
  sw &= ~1;
  sh &= ~1;
  g_cam_dbg.crop_sx = (uint16_t)sx;
  g_cam_dbg.crop_sy = (uint16_t)sy;
  g_cam_dbg.crop_sw = (uint16_t)sw;
  g_cam_dbg.crop_sh = (uint16_t)sh;
  g_speed.crop.x = 0U;
  g_speed.crop.y = 0U;
  g_speed.crop.w = rnw;
  g_speed.crop.h = rnh;
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    cam_hold = 1U;
    if (primask == 0U)
    {
      __enable_irq();
    }
  }
  (void)HAL_DCMI_Stop(&hdcmi);
  cam_arm = 0U;
  cam_curline = 0U;
  cam_draw_full = 2U;
  LCD_FillBothBlack();
  OV5640_PowerCycle();
  OV5640_RgbLcdNativeWindow((uint16_t)sx, (uint16_t)sy,
                            (uint16_t)sw, (uint16_t)sh);
  CAM_LoadView();
  HAL_Delay(20);
  CAM_Start();
  OV5640_FocusLoad();
  HAL_Delay(20);
  CamDbg_SnapSensor();
  HAL_Delay(80);
  CamDbg_SnapLive();
  cam_hold = 0U;
  SpeedTrack_Shift(&g_speed, (int16_t)(-rox), (int16_t)(-roy));
}

static void SpeedTrack_ApplyCrop(void)
{
  OV5640_GetWindow((uint16_t *)&g_cam_dbg.win_x, (uint16_t *)&g_cam_dbg.win_y,
                   (uint16_t *)&g_cam_dbg.win_w, (uint16_t *)&g_cam_dbg.win_h);
  OV5640_GetView((uint16_t *)&g_cam_dbg.view_x, (uint16_t *)&g_cam_dbg.view_y,
                 (uint16_t *)&g_cam_dbg.view_w, (uint16_t *)&g_cam_dbg.view_h);
  OV5640_GetCapture((uint16_t *)&g_cam_dbg.cap_w, (uint16_t *)&g_cam_dbg.cap_h);
  g_cam_dbg.lcd_x0 = (int32_t)g_speed.crop.x;
  g_cam_dbg.lcd_y0 = (int32_t)g_speed.crop.y;
  if (g_cam_dbg.cap_w == 0U)
  {
    g_cam_dbg.cap_w = 1U;
  }
  if (g_cam_dbg.cap_h == 0U)
  {
    g_cam_dbg.cap_h = 1U;
  }
  g_cam_dbg.crop_sw = (uint16_t)(((uint32_t)g_speed.crop.w * (uint32_t)g_cam_dbg.win_w)
                                 / (uint32_t)g_cam_dbg.cap_w);
  g_cam_dbg.crop_sh = (uint16_t)(((uint32_t)g_speed.crop.h * (uint32_t)g_cam_dbg.win_h)
                                 / (uint32_t)g_cam_dbg.cap_h);
  g_cam_dbg.crop_sw = (uint16_t)(g_cam_dbg.crop_sw & (uint16_t)~1U);
  g_cam_dbg.crop_sh = (uint16_t)(g_cam_dbg.crop_sh & (uint16_t)~1U);
  /* 预览左右是镜像的：LCD 左边对应传感器窗口的高端。 */
  {
    int32_t sx;
    sx = (int32_t)g_cam_dbg.win_x + (int32_t)g_cam_dbg.win_w
         - (((g_cam_dbg.lcd_x0 - (int32_t)g_cam_dbg.view_x) * (int32_t)g_cam_dbg.win_w)
            / (int32_t)g_cam_dbg.cap_w)
         - (int32_t)g_cam_dbg.crop_sw;
    if (sx < 0) { sx = 0; }
    g_cam_dbg.crop_sx = (uint16_t)sx;
  }
  g_cam_dbg.crop_sy = (uint16_t)((int32_t)g_cam_dbg.win_y
      + (((g_cam_dbg.lcd_y0 - (int32_t)g_cam_dbg.view_y) * (int32_t)g_cam_dbg.win_h)
         / (int32_t)g_cam_dbg.cap_h));
  if (g_cam_dbg.crop_sw < 80U)
  {
    g_cam_dbg.crop_sw = 80U;
  }
  if (g_cam_dbg.crop_sh < 80U)
  {
    g_cam_dbg.crop_sh = 80U;
  }
  if (g_cam_dbg.crop_sw > LCD_WIDTH)
  {
    g_cam_dbg.crop_sw = (uint16_t)LCD_WIDTH;
  }
  if (g_cam_dbg.crop_sh > LCD_HEIGHT)
  {
    g_cam_dbg.crop_sh = (uint16_t)LCD_HEIGHT;
  }
  /* 先停采集，硬件掉电复位后直接按小窗初始化。不要在开采集之后再刷黑。
   * 裁窗用的窗口和步骤时间都放在 g_cam_dbg，J-Link 随时可读。 */
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    cam_hold = 1U;
    if (primask == 0U)
    {
      __enable_irq();
    }
  }
  g_cam_dbg.frames_pre = cam_frame_count;
  g_cam_dbg.stamp_pre = cam_stamp_us;
  CamDbg_Mark(1U);
  (void)HAL_DCMI_Stop(&hdcmi);
  cam_arm = 0U;
  cam_curline = 0U;
  cam_draw_full = 2U;
  LCD_FillBothBlack();
  OV5640_PowerCycle();
  CamDbg_Mark(2U);
  OV5640_RgbLcdNativeWindow(g_cam_dbg.crop_sx, g_cam_dbg.crop_sy,
                            g_cam_dbg.crop_sw, g_cam_dbg.crop_sh);
  CamDbg_Mark(3U);
  CAM_LoadView();
  HAL_Delay(20);
  CamDbg_Mark(4U);
  CAM_Start();
  OV5640_FocusLoad();
  HAL_Delay(20);
  CamDbg_Mark(5U);
  CamDbg_SnapSensor();
  HAL_Delay(80);
  CamDbg_Mark(6U);
  CamDbg_SnapLive();
  cam_hold = 0U;
  SpeedTrack_Shift(&g_speed, (int16_t)(-(int16_t)g_speed.crop.x),
                   (int16_t)(-(int16_t)g_speed.crop.y));
  /* 第一刀已对准升针附近。不再二次改摄像头窗口。
   * 裁完先让自动曝光走约 1 s，再开始在本窗里认针。 */
  g_need_refine = 1U;
  g_crop_settling = 1U;
  g_crop_settle_t0 = TIM2->CNT;
  g_cam_settle_us = CROP_SETTLE_US;
  speed_phase = 1U;
  CamDbg_Mark(7U);
}

/* 转速模式：在实时画面上直接找升针，不跑表盘算法，不跟梅花轮。裁窗后沿轨迹圆跟前沿。 */
static void SpeedTrack_Service(void)
{
  uint32_t wb;

  if (g_meter_mode != METER_MODE_SPEED)
  {
    return;
  }
  if ((cam_ok == 0U) || (cam_out_jpeg != 0U))
  {
    return;
  }
  if (g_need_relocate != 0U)
  {
    SpeedTrack_OpenFull();
    return;
  }
  if (g_crop_settling != 0U)
  {
    uint32_t need = g_cam_settle_us;
    if (need == 0U)
    {
      need = (speed_phase != 0U) ? CROP_SETTLE_US : FULL_SETTLE_US;
    }
    if ((TIM2->CNT - g_crop_settle_t0) < need)
    {
      return;
    }
    g_crop_settling = 0U;
  }
  if (speed_phase != 0U)
  {
    if (g_need_refine != 0U)
    {
      const uint16_t *rgb;
      int ok = -1;

      wb = DialRead_WorkBytes((uint16_t)LCD_WIDTH, (uint16_t)LCD_HEIGHT);
      if ((cam_w >= 80U) && (cam_h >= 80U) &&
          ((DIAL_WORK_ADDR + wb) <= (SDRAM_BASE + 0x2000000UL)))
      {
        rgb = (const uint16_t *)lcd_frame_addr;
        rgb = rgb + (((uint32_t)cam_oy * LCD_WIDTH) + (uint32_t)cam_ox);
        ok = SpeedTrack_FindNeedle(rgb, (uint16_t)LCD_WIDTH, cam_w, cam_h,
                                   (uint8_t *)DIAL_WORK_ADDR, wb, &g_speed);
      }
      if ((ok == 0) && ((int)g_speed.needle.cx <= (((int)cam_w * 2) / 3)))
      {
        g_need_refine = 0U;
      }
    }
    return;
  }
  wb = DialRead_WorkBytes((uint16_t)LCD_WIDTH, (uint16_t)LCD_HEIGHT);
  if ((DIAL_WORK_ADDR + wb) > (SDRAM_BASE + 0x2000000UL))
  {
    return;
  }
  if (SpeedTrack_FindNeedle((const uint16_t *)lcd_frame_addr, (uint16_t)LCD_WIDTH,
                            (uint16_t)LCD_WIDTH, (uint16_t)LCD_HEIGHT,
                            (uint8_t *)DIAL_WORK_ADDR, wb, &g_speed) != 0)
  {
    return;
  }
  /* 左移预览里四根针都在时，最左一根才是升针。 */
  if (g_n_needles < 4U)
  {
    return;
  }
  SpeedTrack_FinishLocate(&g_speed, (uint16_t)LCD_WIDTH, (uint16_t)LCD_HEIGHT);
  if (g_speed.loc_ok == 0U)
  {
    return;
  }
  SpeedTrack_ApplyCrop();
}


/* ===== 梅花轮齿频模式（METER_MODE_STAR）。升针模式的函数不动，这里另写一份。 ===== */
#define STAR_CROP_MARGIN  24   /* 裁窗在齿顶圆外再留 24 像素（小窗里会再定位一次） */
/* 实测：HTS 2303 × VTS 1023 时帧完成中断约 9.59 帧/s，传感器行场时钟约 22.6 MHz，
 * 不是 ov5640.c 默认假设的 109 MHz。梅花轮小窗按实测值算 50Hz 消纹步长，VTS 取最小，帧率约 45 帧/s。 */
#define STAR_PCLK_HZ      22585000UL
#define STAR_NATIVE_FPS   120U

static void StarMode_OpenFull(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  cam_hold = 1U;
  if (primask == 0U)
  {
    __enable_irq();
  }
  StarTrack_Reset();
  ov5640_pclk_hz = 2625000000UL / (6U * 4U);
  ov5640_native_fps = 50U;
  (void)HAL_DCMI_Stop(&hdcmi);
  cam_arm = 0U;
  cam_curline = 0U;
  cam_draw_full = 2U;
  LCD_FillBothBlack();
  OV5640_PowerCycle();
  if (OV5640_SensorInit() != 0)
  {
    cam_ok = 0U;
    cam_hold = 0U;
    g_star_phase = 0U;
    g_crop_settling = 0U;
    return;
  }
  OV5640_RgbLcdPreview(LCD_WIDTH, LCD_HEIGHT);
  SpeedTrack_PreviewLeft();
  CAM_LoadView();
  HAL_Delay(20);
  CAM_Start();
  OV5640_FocusLoad();
  HAL_Delay(20);
  CamDbg_SnapSensor();
  HAL_Delay(80);
  CamDbg_SnapLive();
  cam_hold = 0U;
  g_star_phase = 0U;
  g_star_loc_fail = 0U;
  g_crop_settling = 1U;
  g_crop_settle_t0 = TIM2->CNT;
  g_cam_settle_us = FULL_SETTLE_US;
}

/* 按全幅 LCD 坐标里的矩形裁传感器窗口，1:1 输出到屏幕右上角。换算同升针模式。 */
static void StarMode_ApplyCrop(uint16_t lx, uint16_t ly, uint16_t lw, uint16_t lh)
{
  OV5640_GetWindow((uint16_t *)&g_cam_dbg.win_x, (uint16_t *)&g_cam_dbg.win_y,
                   (uint16_t *)&g_cam_dbg.win_w, (uint16_t *)&g_cam_dbg.win_h);
  OV5640_GetView((uint16_t *)&g_cam_dbg.view_x, (uint16_t *)&g_cam_dbg.view_y,
                 (uint16_t *)&g_cam_dbg.view_w, (uint16_t *)&g_cam_dbg.view_h);
  OV5640_GetCapture((uint16_t *)&g_cam_dbg.cap_w, (uint16_t *)&g_cam_dbg.cap_h);
  g_cam_dbg.lcd_x0 = (int32_t)lx;
  g_cam_dbg.lcd_y0 = (int32_t)ly;
  if (g_cam_dbg.cap_w == 0U) { g_cam_dbg.cap_w = 1U; }
  if (g_cam_dbg.cap_h == 0U) { g_cam_dbg.cap_h = 1U; }
  g_cam_dbg.crop_sw = (uint16_t)(((uint32_t)lw * (uint32_t)g_cam_dbg.win_w) / (uint32_t)g_cam_dbg.cap_w);
  g_cam_dbg.crop_sh = (uint16_t)(((uint32_t)lh * (uint32_t)g_cam_dbg.win_h) / (uint32_t)g_cam_dbg.cap_h);
  g_cam_dbg.crop_sw = (uint16_t)(g_cam_dbg.crop_sw & (uint16_t)~1U);
  g_cam_dbg.crop_sh = (uint16_t)(g_cam_dbg.crop_sh & (uint16_t)~1U);
  {
    int32_t sx;
    sx = (int32_t)g_cam_dbg.win_x + (int32_t)g_cam_dbg.win_w
         - (((g_cam_dbg.lcd_x0 - (int32_t)g_cam_dbg.view_x) * (int32_t)g_cam_dbg.win_w)
            / (int32_t)g_cam_dbg.cap_w)
         - (int32_t)g_cam_dbg.crop_sw;
    if (sx < 0) { sx = 0; }
    g_cam_dbg.crop_sx = (uint16_t)sx;
  }
  g_cam_dbg.crop_sy = (uint16_t)((int32_t)g_cam_dbg.win_y
      + (((g_cam_dbg.lcd_y0 - (int32_t)g_cam_dbg.view_y) * (int32_t)g_cam_dbg.win_h)
         / (int32_t)g_cam_dbg.cap_h));
  if (g_cam_dbg.crop_sw < 80U) { g_cam_dbg.crop_sw = 80U; }
  if (g_cam_dbg.crop_sh < 80U) { g_cam_dbg.crop_sh = 80U; }
  if (g_cam_dbg.crop_sw > LCD_WIDTH) { g_cam_dbg.crop_sw = (uint16_t)LCD_WIDTH; }
  if (g_cam_dbg.crop_sh > LCD_HEIGHT) { g_cam_dbg.crop_sh = (uint16_t)LCD_HEIGHT; }
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    cam_hold = 1U;
    if (primask == 0U)
    {
      __enable_irq();
    }
  }
  StarTrack_Disarm();
  g_cam_dbg.frames_pre = cam_frame_count;
  g_cam_dbg.stamp_pre = cam_stamp_us;
  CamDbg_Mark(1U);
  (void)HAL_DCMI_Stop(&hdcmi);
  cam_arm = 0U;
  cam_curline = 0U;
  cam_draw_full = 2U;
  LCD_FillBothBlack();
  OV5640_PowerCycle();
  CamDbg_Mark(2U);
  ov5640_pclk_hz = STAR_PCLK_HZ;
  ov5640_native_fps = STAR_NATIVE_FPS;
  OV5640_RgbLcdNativeWindow(g_cam_dbg.crop_sx, g_cam_dbg.crop_sy,
                            g_cam_dbg.crop_sw, g_cam_dbg.crop_sh);
  CamDbg_Mark(3U);
  CAM_LoadView();
  HAL_Delay(20);
  CamDbg_Mark(4U);
  CAM_Start();
  OV5640_FocusLoad();
  HAL_Delay(20);
  CamDbg_Mark(5U);
  CamDbg_SnapSensor();
  HAL_Delay(80);
  CamDbg_Mark(6U);
  CamDbg_SnapLive();
  cam_hold = 0U;
  /* 裁完先让自动曝光走约 1 s，再在小窗里重新定位梅花轮。 */
  g_crop_settling = 1U;
  g_crop_settle_t0 = TIM2->CNT;
  g_cam_settle_us = CROP_SETTLE_US;
  g_star_phase = 1U;
  g_star_loc_fail = 0U;
  CamDbg_Mark(7U);
}

static void StarMode_Service(void)
{
  static int16_t prev_cx;
  static int16_t prev_cy;
  static uint8_t prev_n;
  static uint8_t prev_ok;
  static uint32_t entry_epoch;
  uint32_t wb;
  if (entry_epoch != g_meter_epoch) { entry_epoch = g_meter_epoch; prev_ok = 0U; }

  if (g_meter_mode != METER_MODE_STAR)
  {
    return;
  }
  if ((cam_ok == 0U) || (cam_out_jpeg != 0U))
  {
    return;
  }
  if (g_crop_settling != 0U)
  {
    uint32_t need = g_cam_settle_us;
    if (need == 0U)
    {
      need = FULL_SETTLE_US;
    }
    if ((TIM2->CNT - g_crop_settle_t0) < need)
    {
      return;
    }
    g_crop_settling = 0U;
  }
  wb = DialRead_WorkBytes((uint16_t)LCD_WIDTH, (uint16_t)LCD_HEIGHT);
  if ((DIAL_WORK_ADDR + wb) > (SDRAM_BASE + 0x2000000UL))
  {
    return;
  }
  if (g_star_phase == 0U)
  {
    uint16_t sx1 = 0U, sy1 = 0U;
    int32_t half, x0, y0, x1, y1;
    StarTrack_HudRect(&sx1, &sy1);
    if (StarTrack_Locate((const uint16_t *)lcd_frame_addr, (uint16_t)LCD_WIDTH,
                         (uint16_t)LCD_WIDTH, (uint16_t)LCD_HEIGHT, 2U,
                         (uint8_t *)DIAL_WORK_ADDR, wb, sx1, sy1) != 0)
    {
      g_star_loc_fail++;
      prev_ok = 0U;
      return;
    }
    /* 连续两帧位置一致才裁窗，避免半帧或曝光未稳。 */
    if ((prev_ok == 0U) || (prev_n != g_star.n_teeth) ||
        ((g_star.cx - prev_cx) > 6) || ((prev_cx - g_star.cx) > 6) ||
        ((g_star.cy - prev_cy) > 6) || ((prev_cy - g_star.cy) > 6))
    {
      prev_ok = 1U;
      prev_cx = g_star.cx;
      prev_cy = g_star.cy;
      prev_n = g_star.n_teeth;
      return;
    }
    prev_ok = 0U;
    g_star.full_cx = g_star.cx;
    g_star.full_cy = g_star.cy;
    g_star.full_r = g_star.r_out;
    half = (int32_t)g_star.r_out + STAR_CROP_MARGIN;
    x0 = (int32_t)g_star.cx - half;
    y0 = (int32_t)g_star.cy - half;
    x1 = (int32_t)g_star.cx + half;
    y1 = (int32_t)g_star.cy + half;
    if (x0 < 0) { x0 = 0; }
    if (y0 < 0) { y0 = 0; }
    if (x1 > (int32_t)LCD_WIDTH) { x1 = (int32_t)LCD_WIDTH; }
    if (y1 > (int32_t)LCD_HEIGHT) { y1 = (int32_t)LCD_HEIGHT; }
    x0 &= ~1;
    y0 &= ~1;
    /* 小窗坐标里的位置要在裁完后重新找，先清掉全幅的标记。 */
    g_star.found = 0U;
    StarMode_ApplyCrop((uint16_t)x0, (uint16_t)y0,
                       (uint16_t)((x1 - x0) & ~1), (uint16_t)((y1 - y0) & ~1));
    return;
  }
  if (g_star_phase == 1U)
  {
    const uint16_t *rgb = (const uint16_t *)lcd_frame_addr;
    rgb = rgb + (((uint32_t)cam_oy * LCD_WIDTH) + (uint32_t)cam_ox);
    if ((cam_w >= 80U) && (cam_h >= 80U) &&
        (StarTrack_Locate(rgb, (uint16_t)LCD_WIDTH, cam_w, cam_h, 1U,
                          (uint8_t *)DIAL_WORK_ADDR, wb, 0U, 0U) == 0) &&
        (StarTrack_Arm(cam_w, cam_h) == 0))
    {
      g_star_phase = 2U;
      g_star_loc_fail = 0U;
      return;
    }
    g_star.found = 0U;
    g_star_loc_fail++;
    if (g_star_loc_fail > 150U)
    {
      StarMode_OpenFull();
    }
    return;
  }
  if (StarTrack_Tick(TIM2->CNT) != 0)
  {
    /* 1.5 s 没有有效帧：回到小窗重新定位。 */
    StarTrack_Disarm();
    g_star.found = 0U;
    g_star_phase = 1U;
    g_star_loc_fail = 0U;
  }
}

static void DialRead_Service(void)
{
  static dial_time_t prev_ts;
  static uint8_t have_prev;
  static dial_report_t work_rep;
  static uint32_t entry_epoch;
  uint32_t wb;
  uint32_t debug_raw_copied = 0U;
  if (entry_epoch != g_meter_epoch)
  {
    entry_epoch = g_meter_epoch;
    have_prev = 0U;
    memset(&prev_ts, 0, sizeof(prev_ts));
    memset(&work_rep, 0, sizeof(work_rep));
  }
  if (g_crop_settling != 0U)
  {
    if ((TIM2->CNT - g_crop_settle_t0) < g_cam_settle_us) { return; }
    g_crop_settling = 0U;
  }

  if ((DIAL_READ_ON == 0U) || (cam_ok == 0U) || (cam_out_jpeg != 0U))
  {
    return;
  }
  wb = DialRead_WorkBytes((uint16_t)LCD_WIDTH, (uint16_t)LCD_HEIGHT);
  if ((DIAL_WORK_ADDR + wb) > (SDRAM_BASE + 0x2000000UL))
  {
    return;
  }
  if ((shutter_flag != 0U) && (shutter_armed == 0U)) { return; }
  if ((shutter_armed != 0U) && (shutter_done != 0U)) { return; }
  if (g_meter_raw_diag.ready == 0U)
  {
    g_meter_raw_diag.request = 1U;
    return; /* next complete frame is copied by the pre-HUD handoff */
  }
  __DMB();
  g_meter_raw_diag.hash_used = VolumeRaw_Hash((const uint32_t *)DIAL_SNAP_ADDR);
  if (g_meter_raw_diag.hash_used != g_meter_raw_diag.hash_saved)
  {
    g_meter_raw_diag.hash_bad++;
    g_meter_raw_diag.ready = 0U;
    g_meter_raw_diag.request = 1U;
    return;
  }
  g_meter_raw_diag.busy = 1U;
  g_meter_raw_diag.ready = 0U;
  g_meter_raw_diag.used_gen = g_meter_raw_diag.snap_gen;
  g_meter_raw_diag.used_stamp = g_meter_raw_diag.snap_stamp;
  g_meter_raw_diag.used_frame0 = cam_frame_count;
  if ((g_meter_failed_raw.request != 0U) && (g_meter_failed_raw.ready == 0U)
      && (DIAL_WORK_ADDR + wb <= DIAL_FAILED_RAW_ADDR)
      && (DIAL_FAILED_RAW_ADDR + LCD_FB_BYTES < SHUTTER_MAIL_ADDR))
  {
    memcpy((void *)DIAL_FAILED_RAW_ADDR,(const void *)DIAL_SNAP_ADDR,LCD_FB_BYTES);
    __DMB();
    g_meter_failed_raw.copies++;
    debug_raw_copied = 1U;
  }
  /* Run and its cooperative yields may refresh any number of LCD frames.
   * request stays clear, so none can overwrite this private RAW snapshot. */
  (void)DialRead_Run((const uint16_t *)DIAL_SNAP_ADDR,
                     (uint16_t *)DIAL_SNAP_ADDR,
                     (uint16_t)LCD_WIDTH, (uint16_t)LCD_HEIGHT,
                     (uint8_t *)DIAL_WORK_ADDR, wb, &work_rep);
  g_meter_raw_diag.used_frame1 = cam_frame_count;
  DialRead_FinishReport(&work_rep, &prev_ts, &have_prev);
  g_meter_raw_diag.used_seq = g_meter_volume_seq;
  if ((debug_raw_copied != 0U) && ((g_dial_dbg.miss_place & 1U) != 0U))
  {
    g_meter_failed_raw.version++;
    if (g_dial_path_failure.record.run_seq == g_dial_dbg.seq)
    { g_dial_path_held = g_dial_path_failure; }
    g_meter_failed_raw.seq=g_meter_volume_seq;g_meter_failed_raw.run_seq=g_dial_dbg.seq;
    g_meter_failed_raw.gen=g_meter_raw_diag.used_gen;g_meter_failed_raw.stamp=g_meter_raw_diag.used_stamp;
    g_meter_failed_raw.hash=g_meter_raw_diag.hash_used;g_meter_failed_raw.epoch=g_meter_epoch;
    g_meter_failed_raw.addr=DIAL_FAILED_RAW_ADDR;g_meter_failed_raw.bytes=LCD_FB_BYTES;
    __DMB();
    g_meter_failed_raw.request=0U;g_meter_failed_raw.ready=1U;
    __DMB();g_meter_failed_raw.version++;
  }
  g_meter_raw_diag.busy = 0U;
  if (shutter_armed != 0U)
  {
    /* Display-only paint; never recycled as input after shutter release. */
    DialRead_PaintHud((uint16_t *)lcd_frame_addr, (uint16_t)LCD_WIDTH,
                     (uint16_t)LCD_HEIGHT, &dial_hud_rep);
    shutter_done = 1U;
  }
  else
  {
    shutter_done = 0U;
    g_meter_raw_diag.request = 1U;
  }
}

static void MeterLocate_Service(void)
{
  volatile uint16_t *fb;
  int alive;
  int found;

  if ((METER_LOCATE_ON == 0U) || (cam_ok == 0U) || (cam_out_jpeg != 0U))
  {
    return;
  }
  fb = (volatile uint16_t *)lcd_draw;
  if (fb == 0)
  {
    return;
  }
  if (DIAL_READ_ON != 0U)
  {
    static dial_report_t dial_rep;
    uint32_t wb = DialRead_WorkBytes(LCD_WIDTH, LCD_HEIGHT);
    if (wb <= CAM_MAX_BYTES)
    {
      (void)DialRead_Run((const uint16_t *)fb, (uint16_t *)fb,
                         LCD_WIDTH, LCD_HEIGHT,
                         (uint8_t *)CAM_BUF0, CAM_MAX_BYTES, &dial_rep);
    }
    return;
  }
  if (meter_skip_frames > 0U)
  {
    meter_skip_frames--;
    return;
  }
  alive = 0;
  if (meter_nptrs > 0U)
  {
    alive = MeterLocate_Track((const uint16_t *)fb, LCD_WIDTH, LCD_HEIGHT,
                              CAM_BUF0, CAM_MAX_BYTES,
                              meter_ptrs, meter_nptrs);
    if (alive < 2)
    {
      meter_nptrs = 0U;
      alive = 0;
    }
  }
  if (alive < (int)METER_MAX_POINTERS)
  {
    if ((alive == 0) || (meter_refind_cd == 0U))
    {
      meter_pointer_t found_ptrs[METER_MAX_POINTERS];
      found = MeterLocate_Find((const uint16_t *)fb, LCD_WIDTH, LCD_HEIGHT,
                               CAM_BUF0, CAM_MAX_BYTES,
                               found_ptrs, METER_MAX_POINTERS);
      if (found > alive)
      {
        uint8_t k;
        for (k = 0U; k < (uint8_t)found; k++)
        {
          meter_ptrs[k] = found_ptrs[k];
        }
        meter_nptrs = (uint8_t)found;
        alive = found;
      }
      meter_refind_cd = (alive >= (int)METER_MAX_POINTERS) ? 0U : 8U;
    }
    else
    {
      meter_refind_cd--;
    }
  }
  if (meter_nptrs > 0U)
  {
    MeterLocate_DrawPointers(fb, LCD_WIDTH, LCD_HEIGHT, meter_ptrs, meter_nptrs);
  }
}

/* 正式按键定义沿用板上原程序 GPIO：左 PC13 / 下 PH2 / 右 PH3 均低有效。
 * 上键 PA0 高有效，不参与三个模式，也不再改相机的缩放。每次释放后重按（含同模式）重开测试。 */
void MeterKeys_Tick(void)
{
  static uint8_t raw_last, stable;
  static uint32_t raw_since;
  uint32_t now = HAL_GetTick();
  uint8_t raw = 0U, pressed;
  if (meter_keys_ready == 0U) { return; }
  g_meter_key_ticks++;
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) { raw |= 1U; }
  if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_2) == GPIO_PIN_RESET) { raw |= 2U; }
  if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_3) == GPIO_PIN_RESET) { raw |= 4U; }
  g_meter_key_raw = raw;
  if (raw != raw_last) { raw_last = raw; raw_since = now; return; }
  if ((uint32_t)(now - raw_since) < 30U) { return; }
  pressed = (uint8_t)(raw & (uint8_t)~stable);
  stable = raw;
  g_meter_key_stable = stable;
  /* 多键同时按不选；按住不会重复触发。 */
  if ((pressed == 0U) || ((raw != 1U) && (raw != 2U) && (raw != 4U))) { return; }
  g_meter_last_key = (raw == 1U) ? 1U : ((raw == 2U) ? 2U : 3U);
  g_meter_key_n++;
  g_meter_req_source = 1U;
  __DMB();
  g_meter_request = (raw == 1U) ? METER_MODE_VOLUME : ((raw == 2U) ? METER_MODE_SPEED : METER_MODE_STAR);
}

/* SpeedTrack_Reset 保留的旧列表由运行时“新测试”入口清除；算法本身不改。 */
extern uint8_t g_meas_i;
extern int32_t g_meas_flow_e2[10], g_meas_accum[10];
extern uint32_t g_meas_dt_us[10];
extern uint16_t g_meas_seq[10];
extern uint8_t g_mw_target;
extern uint32_t g_mw_lap_us;

static void MeterMode_Service(void)
{
  uint32_t want;
  uint32_t primask = __get_PRIMASK();
  /* 原子取出请求，SysTick 只负责投递，主循环负责相机重启。 */
  __disable_irq();
  want = g_meter_request;
  g_meter_request = MODE_REQUEST_NONE;
  if (want != MODE_REQUEST_NONE)
  {
    g_meter_last_source = (g_meter_req_source == 1U) ? 1U : 2U;
    g_meter_req_source = 0U;
  }
  if (primask == 0U) { __enable_irq(); }
  if (want == MODE_REQUEST_NONE) { return; }
  if (want > METER_MODE_STAR) { g_meter_status = 0xEU; return; }
  g_meter_last_from = g_meter_mode;
  g_meter_last_to = want;
  g_meter_status = 1U;
  /* 切换仅在主循环执行。先封住高优先级行 DMA 与帧回调，再停止 DMA/DCMI。
   * 中断保持可用以让 HAL 超时/传感器延时工作，但旧回调不再写显存/采样表。 */
  primask = __get_PRIMASK();
  __disable_irq();
  g_meter_switching = 1U;
  cam_hold = 1U;
  __DMB();
  if (primask == 0U) { __enable_irq(); }
  StarTrack_Disarm();
  (void)HAL_DCMI_Stop(&hdcmi);
  __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_FRAME | DCMI_IT_OVR | DCMI_IT_ERR);
  DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |
                DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
  __HAL_DCMI_CLEAR_FLAG(&hdcmi, DCMI_FLAG_FRAMERI | DCMI_FLAG_OVRRI | DCMI_FLAG_ERRRI);
  HAL_NVIC_ClearPendingIRQ(DCMI_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);
  g_meter_mode = want;
  g_meter_status = 2U;
  g_meter_epoch++;
  g_meter_switch_n++;
  /* 显式重开测试不同于普通丢失重定位：本次序号、列表与上一时间戳全清。 */
  StarTrack_NewTest();
  SpeedTrack_Reset(&g_speed);
  g_rev_seq = 0U; g_meas_n = 0U; g_meas_i = 0U;
  g_flow_avg_e2 = 0; g_flow_rep_e4 = 0; g_frame_dt_us = 0U;
  g_mw_target = 0U; g_mw_lap_us = 0U;
  memset(g_meas_flow_e2, 0, sizeof(g_meas_flow_e2));
  memset(g_meas_accum, 0, sizeof(g_meas_accum));
  memset(g_meas_dt_us, 0, sizeof(g_meas_dt_us));
  memset(g_meas_seq, 0, sizeof(g_meas_seq));
  DialRead_ResetTrack();
  memset(&dial_hud_rep, 0, sizeof(dial_hud_rep));
  dial_hud_ready = 0U;
  g_meter_volume_seq = 0U;
  memset((void *)&g_meter_volume_diag, 0, sizeof(g_meter_volume_diag));
  speed_phase = 0U; g_star_phase = 0U; g_star_loc_fail = 0U;
  shutter_armed = 0U; shutter_done = 0U;
  g_meter_raw_diag.request = 0U; g_meter_raw_diag.ready = 0U;
  g_meter_raw_diag.busy = 0U; g_meter_raw_diag.used_seq = 0U;
  g_meter_failed_raw.request=0U;g_meter_failed_raw.ready=0U;
  cam_arm = 0U; cam_restart = 0U; cam_ready = 0U;
  cam_scale_ready = 0U; cam_out_jpeg = 0U; cam_curline = 0U; cam_draw_full = 2U;
  meter_hold_fb = 0U; meter_want_snap = 0U; meter_frame_ready = 0U;
  shot_request = 0U; f_send = 0U;
  ov5640_pclk_hz = 2625000000UL / (6U * 4U);
  ov5640_native_fps = 50U;
  g_meter_status = 3U;
  LCD_FillBothBlack();
  OV5640_PowerCycle();
  cam_ok = 0U;
  CAM_Init(); /* 全幅、默认自动曝光：volume 居中；speed/star 左移，随后各自原裁窗流程 */
  if (cam_ok != 0U) { CamDbg_SnapSensor(); }
  g_crop_settling = 1U;
  g_cam_settle_us = FULL_SETTLE_US;
  g_crop_settle_t0 = TIM2->CNT;
  cam_draw_full = 2U;
  cam_hold = 0U;
  __DMB();
  g_meter_switching = 0U;
  g_meter_status = (cam_ok != 0U) ? 4U : 0xEU;
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *dcmi)
{
  (void)dcmi;
  if (g_meter_switching != 0U) { return; }
  /* 预览是连续 RGB565。溢出只重开 DMA，不要重写传感器，否则画面会停在上一帧。 */
  if (hdcmi.Init.JPEGMode != DCMI_JPEG_ENABLE)
  {
    hdcmi.State = HAL_DCMI_STATE_READY;
    __HAL_UNLOCK(&hdcmi);
    cam_arm = 1U;
    return;
  }
  cam_restart = 1U;
}

void HAL_DCMI_MspInit(DCMI_HandleTypeDef *dcmi)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_DCMI_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF13_DCMI;

  gpio.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11;
  HAL_GPIO_Init(GPIOC, &gpio);
  gpio.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOD, &gpio);
  gpio.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOH, &gpio);

  hdma_dcmi.Instance = DMA1_Stream1;
  hdma_dcmi.Init.Request = DMA_REQUEST_DCMI;
  hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dcmi.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_dcmi.Init.Mode = DMA_NORMAL;
  hdma_dcmi.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_dcmi.Init.MemBurst = DMA_MBURST_INC4;
  hdma_dcmi.Init.PeriphBurst = DMA_PBURST_INC4;
  if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(dcmi, DMA_Handle, hdma_dcmi);

  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  HAL_NVIC_SetPriority(DCMI_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);
}

static void CAM_Init(void)
{
  if (OV5640_SensorInit() != 0)
  {
    cam_ok = 0U;
    return;
  }
  /* NativeWindow 会保留驱动的软件窗口，切模式必须还原它，而非只复位传感器。 */
  OV5640_SetNativeOutput(0U);
  OV5640_ShowOnLcd(0, 0, LCD_WIDTH, LCD_HEIGHT);
  OV5640_RgbLcdPreview(LCD_WIDTH, LCD_HEIGHT);
  if ((g_meter_mode == METER_MODE_SPEED) || (g_meter_mode == METER_MODE_STAR))
  {
    SpeedTrack_PreviewLeft();
  }
  CAM_LoadView();

  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    cam_ok = 0U;
    return;
  }
  cam_ok = 1U;
  if (g_meter_switching == 0U) { g_meter_status = 4U; }
  cam_hold = 1U;
  CAM_Start();
  OV5640_FocusLoad();
  cam_hold = 0U;
  meter_next_ms = HAL_GetTick() + 1500U;
  if ((g_meter_mode == METER_MODE_SPEED) || (g_meter_mode == METER_MODE_STAR))
  {
    /* 上电后摄像头同样要先自动曝光，再开始全幅找针。 */
    g_crop_settling = 1U;
    g_crop_settle_t0 = TIM2->CNT;
    g_cam_settle_us = FULL_SETTLE_US;
  }
}

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef mpu = {0};

  HAL_MPU_Disable();
  mpu.Enable = MPU_REGION_ENABLE;
  mpu.Number = MPU_REGION_NUMBER0;
  mpu.BaseAddress = SDRAM_BASE;
  mpu.Size = MPU_REGION_SIZE_32MB;
  mpu.SubRegionDisable = 0x00;
  mpu.TypeExtField = MPU_TEX_LEVEL1;
  mpu.AccessPermission = MPU_REGION_FULL_ACCESS;
  mpu.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  mpu.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  mpu.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  mpu.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&mpu);
  mpu.Number = MPU_REGION_NUMBER1;
  mpu.BaseAddress = 0x30000000UL;
  mpu.Size = MPU_REGION_SIZE_64KB;
  mpu.TypeExtField = MPU_TEX_LEVEL1;
  mpu.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  mpu.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  mpu.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&mpu);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void SDRAM_InitSequence(void)
{
  FMC_SDRAM_CommandTypeDef command = {0};
  FMC_SDRAM_TimingTypeDef timing = {0};
  uint32_t mode;

  /* 阿波罗 H743 SDRAM 实验：CAS2，读突发开，读通道延时 1，时序 2/8/7/7/2/2/2 */
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  (void)FMC_SDRAM_Init(hsdram1.Instance, &hsdram1.Init);

  timing.LoadToActiveDelay = 2;
  timing.ExitSelfRefreshDelay = 8;
  timing.SelfRefreshTime = 7;
  timing.RowCycleDelay = 7;
  timing.WriteRecoveryTime = 2;
  timing.RPDelay = 2;
  timing.RCDDelay = 2;
  (void)FMC_SDRAM_Timing_Init(hsdram1.Instance, &timing, FMC_SDRAM_BANK1);

  command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  command.AutoRefreshNumber = 1;
  command.ModeRegisterDefinition = 0;
  if (HAL_SDRAM_SendCommand(&hsdram1, &command, SDRAM_TIMEOUT_MS) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(1);

  command.CommandMode = FMC_SDRAM_CMD_PALL;
  if (HAL_SDRAM_SendCommand(&hsdram1, &command, SDRAM_TIMEOUT_MS) != HAL_OK)
  {
    Error_Handler();
  }

  command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  command.AutoRefreshNumber = 8;
  if (HAL_SDRAM_SendCommand(&hsdram1, &command, SDRAM_TIMEOUT_MS) != HAL_OK)
  {
    Error_Handler();
  }

  mode = SDRAM_MODEREG_BURST_LENGTH_1 |
         SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL |
         SDRAM_MODEREG_CAS_LATENCY_2 |
         SDRAM_MODEREG_OPERATING_MODE_STANDARD |
         SDRAM_MODEREG_WRITEBURST_SINGLE;
  command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  command.AutoRefreshNumber = 1;
  command.ModeRegisterDefinition = mode;
  if (HAL_SDRAM_SendCommand(&hsdram1, &command, SDRAM_TIMEOUT_MS) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_SDRAM_ProgramRefreshRate(&hsdram1, SDRAM_REFRESH_COUNT) != HAL_OK)
  {
    Error_Handler();
  }
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
#ifdef USE_FULL_ASSERT
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
