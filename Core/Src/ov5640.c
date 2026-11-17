#include "ov5640.h"
#include "main.h"
#include "E:/百度网盘下载/正点原子/【正点原子】阿波罗V2 STM32H743开发板/4，程序源码/4，程序源码/2，标准例程-HAL库版本/2，标准例程-HAL库版本/实验39 摄像头实验/Drivers/BSP/OV5640/ov5640af.h"

/* SCCB 挂在 PB3/PB4，原理图上是开漏加外部 4.7k。这两脚没有 I2C 复用。 */
#define SCCB_PORT GPIOB
#define SCCB_SDA  GPIO_PIN_3
#define SCCB_SCL  GPIO_PIN_4
#define SCCB_ADDR 0x78U

#define OV5640_ARRAY_W 2624U
#define OV5640_ARRAY_H 1944U
#define OV5640_ARRAY_Y 4U
#define OV5640_PAN_STEP 64

typedef struct
{
  uint16_t width;
  uint16_t height;
  uint8_t pll;
} OV5640_Gear;

/* 前 6 档宽高不超过 1024x600，能整块 1:1 放在屏幕上。后面两档比屏幕大，↑↓ 不切过去。 */
#define OV5640_GEAR_LCD 6U

/* 窗口在传感器上 1:1 裁剪。宽画面的行周期要留够，DMA 才跟得上。 */
static const OV5640_Gear ov5640_gear[OV5640_GEAR_COUNT] =
{
  { 160U, 120U, 0x41U },
  { 320U, 240U, 0x41U },
  { 480U, 272U, 0x51U },
  { 640U, 480U, 0x61U },
  { 800U, 480U, 0x61U },
  { 1024U, 600U, 0x61U },
  { 1280U, 720U, 0x41U },
  { 1600U, 1200U, 0x41U },
};

/* 上电 1024x600，分频 0x61。硬件 1:1 开窗，铺满屏幕。 */
static uint8_t ov5640_level = 5U;
static uint8_t ov5640_jpeg;
static int16_t ov5640_pan;
static volatile uint16_t ov5640_rb_w;
static volatile uint16_t ov5640_rb_hts;
static volatile uint8_t ov5640_rb_sub;
static volatile uint32_t ov5640_rb_exp;
static volatile uint8_t ov5640_rb_bright;
extern const uint32_t cam_shutter_us;
extern const uint8_t cam_gain;

static uint32_t sccb_delay_n = 8000U;

static void SCCB_Delay(void)
{
  volatile uint32_t count = sccb_delay_n;

  while (count > 0U)
  {
    count--;
  }
}

static void SCCB_Sda(uint8_t high)
{
  HAL_GPIO_WritePin(SCCB_PORT, SCCB_SDA, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void SCCB_Scl(uint8_t high)
{
  HAL_GPIO_WritePin(SCCB_PORT, SCCB_SCL, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void SCCB_Start(void)
{
  SCCB_Sda(1U);
  SCCB_Scl(1U);
  SCCB_Delay();
  SCCB_Sda(0U);
  SCCB_Delay();
  SCCB_Scl(0U);
}

static void SCCB_Stop(void)
{
  SCCB_Sda(0U);
  SCCB_Delay();
  SCCB_Scl(1U);
  SCCB_Delay();
  SCCB_Sda(1U);
  SCCB_Delay();
}

static uint8_t SCCB_WriteByte(uint8_t data)
{
  uint8_t bit;
  uint8_t ack;

  for (bit = 0U; bit < 8U; bit++)
  {
    SCCB_Sda((data & 0x80U) ? 1U : 0U);
    data <<= 1;
    SCCB_Delay();
    SCCB_Scl(1U);
    SCCB_Delay();
    SCCB_Scl(0U);
  }
  SCCB_Sda(1U);
  SCCB_Delay();
  SCCB_Scl(1U);
  SCCB_Delay();
  ack = (HAL_GPIO_ReadPin(SCCB_PORT, SCCB_SDA) == GPIO_PIN_RESET) ? 1U : 0U;
  SCCB_Scl(0U);
  return ack;
}

static uint8_t SCCB_ReadByte(uint8_t ack)
{
  uint8_t bit;
  uint8_t data = 0U;

  SCCB_Sda(1U);
  for (bit = 0U; bit < 8U; bit++)
  {
    data <<= 1;
    SCCB_Scl(1U);
    SCCB_Delay();
    if (HAL_GPIO_ReadPin(SCCB_PORT, SCCB_SDA) == GPIO_PIN_SET)
    {
      data |= 1U;
    }
    SCCB_Scl(0U);
    SCCB_Delay();
  }
  SCCB_Sda(ack ? 0U : 1U);
  SCCB_Delay();
  SCCB_Scl(1U);
  SCCB_Delay();
  SCCB_Scl(0U);
  SCCB_Sda(1U);
  return data;
}

static int SCCB_WriteReg(uint16_t reg, uint8_t value)
{
  uint32_t primask = __get_PRIMASK();
  int result = 0;

  /* 行中断会把这一位的时序拉开，传感器不应答，固件就写不进去。 */
  __disable_irq();
  SCCB_Start();
  if (SCCB_WriteByte(SCCB_ADDR) == 0U)
  {
    result = -1;
  }
  else if (SCCB_WriteByte((uint8_t)(reg >> 8)) == 0U)
  {
    result = -1;
  }
  else if (SCCB_WriteByte((uint8_t)reg) == 0U)
  {
    result = -1;
  }
  else if (SCCB_WriteByte(value) == 0U)
  {
    result = -1;
  }
  SCCB_Stop();
  if (primask == 0U)
  {
    __enable_irq();
  }
  return result;
}

static int SCCB_ReadReg(uint16_t reg, uint8_t *value)
{
  uint32_t primask = __get_PRIMASK();
  int result = 0;

  __disable_irq();
  SCCB_Start();
  if (SCCB_WriteByte(SCCB_ADDR) == 0U)
  {
    result = -1;
  }
  else if (SCCB_WriteByte((uint8_t)(reg >> 8)) == 0U)
  {
    result = -1;
  }
  else if (SCCB_WriteByte((uint8_t)reg) == 0U)
  {
    result = -1;
  }
  else
  {
    SCCB_Stop();
    SCCB_Start();
    if (SCCB_WriteByte(SCCB_ADDR | 0x01U) == 0U)
    {
      result = -1;
    }
    else
    {
      *value = SCCB_ReadByte(0U);
    }
  }
  SCCB_Stop();
  if (primask == 0U)
  {
    __enable_irq();
  }
  return result;
}

static int OV5640_WriteList(const uint16_t *table, uint32_t count)
{
  uint32_t i;

  for (i = 0U; i < count; i++)
  {
    uint16_t reg = table[i * 2U];
    uint8_t value = (uint8_t)table[(i * 2U) + 1U];

    if (SCCB_WriteReg(reg, value) != 0)
    {
      return -1;
    }
    if ((reg == 0x3008U) && (value == 0x82U))
    {
      HAL_Delay(20);
    }
  }
  return 0;
}

/* 开发板摄像头掉电走 PCF8574：P2=PWDN（高掉电，低工作），P7=网口复位（低有效，空闲为高）。
   扩展芯片 I2C：PH4=SCL，PH5=SDA，写地址 0x40（A0/A1/A2 接地）。默认先写 0xFF，再拉低 P2。 */
#define PCF_PORT      GPIOH
#define PCF_SCL       GPIO_PIN_4
#define PCF_SDA       GPIO_PIN_5
#define PCF_ADDR      0x40U
#define PCF_PWDN_BIT  2U
#define PCF_ETH_BIT   7U

uint8_t pcf_ok;
uint8_t pcf_shadow = 0xFFU;

static void Pcf_Delay(void)
{
  volatile uint32_t count = 8000U;

  while (count > 0U)
  {
    count--;
  }
}

static void Pcf_Sda(uint8_t high)
{
  HAL_GPIO_WritePin(PCF_PORT, PCF_SDA, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Pcf_Scl(uint8_t high)
{
  HAL_GPIO_WritePin(PCF_PORT, PCF_SCL, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Pcf_Pins(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  gpio.Pin = PCF_SCL | PCF_SDA;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = 0U;
  HAL_GPIO_Init(PCF_PORT, &gpio);
  Pcf_Sda(1U);
  Pcf_Scl(1U);
}

static void Pcf_Start(void)
{
  Pcf_Sda(1U);
  Pcf_Scl(1U);
  Pcf_Delay();
  Pcf_Sda(0U);
  Pcf_Delay();
  Pcf_Scl(0U);
}

static void Pcf_Stop(void)
{
  Pcf_Sda(0U);
  Pcf_Delay();
  Pcf_Scl(1U);
  Pcf_Delay();
  Pcf_Sda(1U);
  Pcf_Delay();
}

static uint8_t Pcf_WriteByte(uint8_t data)
{
  uint8_t bit;
  uint8_t ack;

  for (bit = 0U; bit < 8U; bit++)
  {
    Pcf_Sda((data & 0x80U) ? 1U : 0U);
    data <<= 1;
    Pcf_Delay();
    Pcf_Scl(1U);
    Pcf_Delay();
    Pcf_Scl(0U);
  }
  Pcf_Sda(1U);
  Pcf_Delay();
  Pcf_Scl(1U);
  Pcf_Delay();
  ack = (HAL_GPIO_ReadPin(PCF_PORT, PCF_SDA) == GPIO_PIN_RESET) ? 1U : 0U;
  Pcf_Scl(0U);
  return ack;
}

static int Pcf_Write(uint8_t data)
{
  uint32_t primask = __get_PRIMASK();
  int result = -1;

  __disable_irq();
  Pcf_Start();
  if (Pcf_WriteByte(PCF_ADDR) != 0U)
  {
    if (Pcf_WriteByte(data) != 0U)
    {
      pcf_shadow = data;
      result = 0;
    }
  }
  Pcf_Stop();
  if (primask == 0U)
  {
    __enable_irq();
  }
  return result;
}

static int Pcf_WriteBit(uint8_t bit, uint8_t high)
{
  uint8_t data = pcf_shadow;

  if (high != 0U)
  {
    data = (uint8_t)(data | (uint8_t)(1U << bit));
  }
  else
  {
    data = (uint8_t)(data & (uint8_t)~(1U << bit));
  }
  return Pcf_Write(data);
}

static int Pcf_Init(void)
{
  Pcf_Pins();
  pcf_shadow = 0xFFU;
  pcf_ok = (Pcf_Write(0xFFU) == 0) ? 1U : 0U;
  return (pcf_ok != 0U) ? 0 : -1;
}

static void Cam_Pwdn(uint8_t power_down)
{
  (void)Pcf_WriteBit(PCF_PWDN_BIT, power_down);
}

void OV5640_BoardIo(uint8_t bit, uint8_t high)
{
  if (bit > 7U)
  {
    return;
  }
  (void)Pcf_WriteBit(bit, high);
}

static void OV5640_Pins(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_8;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);

  gpio.Pin = GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = 0U;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_Delay(20);

  (void)Pcf_Init();
  Cam_Pwdn(0U);
  HAL_Delay(5);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(20);

  gpio.Pin = SCCB_SCL;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SCCB_PORT, &gpio);
  gpio.Pin = SCCB_SDA;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  HAL_GPIO_Init(SCCB_PORT, &gpio);
  SCCB_Sda(1U);
  SCCB_Scl(1U);
  HAL_Delay(5);
}

/* ST DVP RGB565 公共表。裁剪窗口由 OV5640_ApplyView 按档位改写。 */
static const uint16_t ov5640_common[] =
{
  0x3103, 0x11, 0x3008, 0x82, 0x3103, 0x03,
  0x3630, 0x36, 0x3631, 0x0e, 0x3632, 0xe2, 0x3633, 0x12, 0x3621, 0xe0,
  0x3704, 0xa0, 0x3703, 0x5a, 0x3715, 0x78, 0x3717, 0x01, 0x370b, 0x60, 0x3705, 0x1a,
  0x3905, 0x02, 0x3906, 0x10, 0x3901, 0x0a, 0x3731, 0x12,
  0x3600, 0x08, 0x3601, 0x33, 0x302d, 0x60, 0x3620, 0x52, 0x371b, 0x20, 0x471c, 0x50,
  0x3a13, 0x43, 0x3a18, 0x00, 0x3a19, 0xf8, 0x3635, 0x13, 0x3636, 0x03, 0x3634, 0x40, 0x3622, 0x01,
  0x3c01, 0x34, 0x3c04, 0x28, 0x3c05, 0x98,
  0x3c06, 0x00, 0x3c07, 0x00, 0x3c08, 0x01, 0x3c09, 0x2c, 0x3c0a, 0x9c, 0x3c0b, 0x40,
  0x3820, 0x00, 0x3821, 0x00, 0x3814, 0x31, 0x3815, 0x31,
  0x3800, 0x00, 0x3801, 0x00, 0x3802, 0x00, 0x3803, 0x04,
  0x3804, 0x0a, 0x3805, 0x3f, 0x3806, 0x07, 0x3807, 0x9b,
  0x3808, 0x03, 0x3809, 0x20, 0x380a, 0x01, 0x380b, 0xe0,
  0x380c, 0x07, 0x380d, 0x90, 0x380e, 0x04, 0x380f, 0x40,
  0x3810, 0x00, 0x3811, 0x10, 0x3812, 0x00, 0x3813, 0x06,
  0x3618, 0x00, 0x3612, 0x29, 0x3708, 0x64, 0x3709, 0x52, 0x370c, 0x03,
  0x3a02, 0x03, 0x3a03, 0xd8, 0x3a08, 0x01, 0x3a09, 0x27, 0x3a0a, 0x00, 0x3a0b, 0xf6,
  0x3a0e, 0x03, 0x3a0d, 0x04, 0x3a14, 0x03, 0x3a15, 0xd8,
  0x4001, 0x02, 0x4004, 0x02, 0x3000, 0x00, 0x3002, 0x00, 0x3004, 0xff, 0x3006, 0xff,
  0x300e, 0x58, 0x302e, 0x00, 0x4740, 0x21, 0x4300, 0x6f, 0x501f, 0x01,
  0x4713, 0x00, 0x4407, 0x10, 0x440e, 0x00, 0x460b, 0x35, 0x460c, 0x23, 0x4837, 0x22, 0x3824, 0x02,
  0x5000, 0xa7, 0x5001, 0xa3,
  0x5180, 0xff, 0x5181, 0xf2, 0x5182, 0x00, 0x5183, 0x14, 0x5184, 0x25, 0x5185, 0x24,
  0x5186, 0x09, 0x5187, 0x09, 0x5188, 0x09, 0x5189, 0x75, 0x518a, 0x54, 0x518b, 0xe0,
  0x518c, 0xb2, 0x518d, 0x42, 0x518e, 0x3d, 0x518f, 0x56, 0x5190, 0x46, 0x5191, 0xf8,
  0x5192, 0x04, 0x5193, 0x70, 0x5194, 0xf0, 0x5195, 0xf0, 0x5196, 0x03, 0x5197, 0x01,
  0x5198, 0x04, 0x5199, 0x12, 0x519a, 0x04, 0x519b, 0x00, 0x519c, 0x06, 0x519d, 0x82,
  0x519e, 0x38,
  0x5381, 0x1c, 0x5382, 0x5a, 0x5383, 0x06, 0x5384, 0x1f, 0x5385, 0x7a, 0x5386, 0x9a,
  0x5387, 0x9c, 0x5388, 0x9a, 0x5389, 0x02, 0x538a, 0x01, 0x538b, 0x98,
  0x5300, 0x08, 0x5301, 0x30, 0x5302, 0x10, 0x5303, 0x00, 0x5304, 0x08, 0x5305, 0x30,
  0x5306, 0x08, 0x5307, 0x16, 0x5308, 0x08, 0x5309, 0x30, 0x530a, 0x04, 0x530b, 0x06,
  0x5480, 0x01, 0x5481, 0x08, 0x5482, 0x14, 0x5483, 0x28, 0x5484, 0x51, 0x5485, 0x65,
  0x5486, 0x71, 0x5487, 0x7d, 0x5488, 0x87, 0x5489, 0x91, 0x548a, 0x9a, 0x548b, 0xaa,
  0x548c, 0xb8, 0x548d, 0xcd, 0x548e, 0xdd, 0x548f, 0xea, 0x5490, 0x1d,
  0x5580, 0x02, 0x5583, 0x40, 0x5584, 0x10, 0x5585, 0x10, 0x5586, 0x24,
  0x5589, 0x10, 0x558a, 0x00, 0x558b, 0xf8,
  0x5800, 0x23, 0x5801, 0x14, 0x5802, 0x0f, 0x5803, 0x0f, 0x5804, 0x12, 0x5805, 0x26,
  0x5806, 0x0c, 0x5807, 0x08, 0x5808, 0x05, 0x5809, 0x05, 0x580a, 0x08, 0x580b, 0x0d,
  0x580c, 0x08, 0x580d, 0x03, 0x580e, 0x00, 0x580f, 0x00, 0x5810, 0x03, 0x5811, 0x09,
  0x5812, 0x07, 0x5813, 0x03, 0x5814, 0x00, 0x5815, 0x01, 0x5816, 0x03, 0x5817, 0x08,
  0x5818, 0x0d, 0x5819, 0x08, 0x581a, 0x05, 0x581b, 0x06, 0x581c, 0x08, 0x581d, 0x0e,
  0x581e, 0x29, 0x581f, 0x17, 0x5820, 0x11, 0x5821, 0x11, 0x5822, 0x15, 0x5823, 0x28,
  0x5824, 0x46, 0x5825, 0x26, 0x5826, 0x08, 0x5827, 0x26, 0x5828, 0x64, 0x5829, 0x26,
  0x582a, 0x24, 0x582b, 0x22, 0x582c, 0x24, 0x582d, 0x24, 0x582e, 0x06, 0x582f, 0x22,
  0x5830, 0x40, 0x5831, 0x42, 0x5832, 0x24, 0x5833, 0x26, 0x5834, 0x24, 0x5835, 0x22,
  0x5836, 0x22, 0x5837, 0x26, 0x5838, 0x44, 0x5839, 0x24, 0x583a, 0x26, 0x583b, 0x28,
  0x583c, 0x42, 0x583d, 0xce, 0x5025, 0x00,
  0x3503, 0x01,
  0x3a0f, 0x30, 0x3a10, 0x28, 0x3a1b, 0x30, 0x3a1e, 0x26, 0x3a11, 0x60, 0x3a1f, 0x14,
  0x3008, 0x02,
};

static const uint16_t ov5640_dvp[] =
{
  0x3017, 0xff, 0x3018, 0xff, 0x302e, 0x00, 0x471c, 0x50, 0x300e, 0x58,
  0x3034, 0x1a, 0x3035, 0x41, 0x3036, 0x69, 0x3037, 0x13, 0x3108, 0x01,
};

static void OV5640_WriteU16(uint16_t reg, uint16_t value)
{
  (void)SCCB_WriteReg(reg, (uint8_t)(value >> 8));
  (void)SCCB_WriteReg((uint16_t)(reg + 1U), (uint8_t)value);
}

void OV5640_GetSize(uint16_t *width, uint16_t *height)
{
  *width = ov5640_gear[ov5640_level].width;
  *height = ov5640_gear[ov5640_level].height;
}

int OV5640_Zoom(int direction)
{
  int next = (int)ov5640_level + direction;

  if ((next < 0) || (next >= (int)OV5640_GEAR_LCD))
  {
    return 0;
  }
  ov5640_level = (uint8_t)next;
  return 1;
}

int OV5640_Pan(int direction)
{
  int32_t limit = ((int32_t)OV5640_ARRAY_W - (int32_t)ov5640_gear[ov5640_level].width) / 2;
  int32_t next = (int32_t)ov5640_pan + (direction * OV5640_PAN_STEP);

  if (next > limit)
  {
    next = limit;
  }
  if (next < -limit)
  {
    next = -limit;
  }
  if (next == (int32_t)ov5640_pan)
  {
    return 0;
  }
  ov5640_pan = (int16_t)next;
  return 1;
}

void OV5640_SetJpeg(uint8_t on)
{
  ov5640_jpeg = (on != 0U) ? 1U : 0U;
}

void OV5640_ApplyView(void)
{
  const OV5640_Gear *gear = &ov5640_gear[ov5640_level];
  int32_t max_x = (int32_t)OV5640_ARRAY_W - (int32_t)gear->width;
  int32_t max_y = (int32_t)OV5640_ARRAY_H - (int32_t)gear->height;
  int32_t x0 = (max_x / 2) + (int32_t)ov5640_pan;
  int32_t y0 = (int32_t)OV5640_ARRAY_Y + (max_y / 2);
  uint16_t x1;
  uint16_t y1;

  if (x0 < 0)
  {
    x0 = 0;
  }
  if (x0 > max_x)
  {
    x0 = max_x;
  }
  uint32_t sysdiv = (uint32_t)gear->pll >> 4;
  uint32_t pclk_hz = 2625000000UL / (6U * sysdiv);
  uint32_t exp_reg;
  uint32_t exp_max_reg;
  /* 预览是传感器 1:1 开窗。宽度超过 640 时行周期至少约两倍宽再加同步时间，
     否则 OV5640 会停住，或者 DMA 跟不上，画面变成横线来回摆。 */
  uint16_t hts;
  uint16_t vts;

  if (ov5640_jpeg != 0U)
  {
    hts = (gear->width > 2400U) ? 3200U : 2400U;
    vts = (uint16_t)(gear->height + 160U);
  }
  else
  {
    hts = (gear->width > 640U) ? (uint16_t)((gear->width * 2U) + 8U) : (uint16_t)(gear->width + 160U);
    if (hts < 0x0790U)
    {
      hts = 0x0790U;
    }
    hts = (uint16_t)(hts + 252U);
    vts = 0x0440U;
  }
  uint16_t exp_max = (uint16_t)(vts - 8U);

  /* 传感器只认行数。把整帧快门换算成行，再写成 1/16 行。长过一帧时收到帧内。 */
  exp_reg = (uint32_t)(((uint64_t)cam_shutter_us * pclk_hz * 16ULL) / ((uint64_t)hts * 1000000ULL));
  exp_max_reg = (uint32_t)exp_max << 4;
  if (exp_reg < 1U)
  {
    exp_reg = 1U;
  }
  if (exp_reg > exp_max_reg)
  {
    exp_reg = exp_max_reg;
  }

  x0 &= ~1;
  y0 &= ~1;
  /* 裁剪窗口比输出多一圈。刚好一样大时，缩放器会旁路，整行时钟都吐出来，画面就成横纹。 */
  x0 -= 16;
  y0 -= 8;
  if (x0 < 0)
  {
    x0 = 0;
  }
  if (y0 < (int32_t)OV5640_ARRAY_Y)
  {
    y0 = (int32_t)OV5640_ARRAY_Y;
  }
  x1 = (uint16_t)(x0 + (int32_t)gear->width + 48 - 1);
  y1 = (uint16_t)(y0 + (int32_t)gear->height + 24 - 1);
  if (x1 >= OV5640_ARRAY_W)
  {
    x1 = (uint16_t)(OV5640_ARRAY_W - 1U);
  }
  if (y1 >= (uint16_t)(OV5640_ARRAY_Y + OV5640_ARRAY_H))
  {
    y1 = (uint16_t)(OV5640_ARRAY_Y + OV5640_ARRAY_H - 1U);
  }

  /* 0x11：逐像素读出。窗口只比输出大一圈，避免整幅抽样。 */
  (void)SCCB_WriteReg(0x3814U, 0x11U);
  (void)SCCB_WriteReg(0x3815U, 0x11U);
  (void)SCCB_WriteReg(0x3820U, 0x00U);
  /* JPEG 时 bit5 必须为 1。bit1/bit2 仍是 0，画面不镜像。 */
  (void)SCCB_WriteReg(0x3821U, (ov5640_jpeg != 0U) ? 0x20U : 0x00U);
  (void)SCCB_WriteReg(0x5001U, 0xA3U);

  /* 组写窗口。输出宽高等于裁剪宽高，这样一行有效像素才等于屏幕上的一行。 */
  (void)SCCB_WriteReg(0x3212U, 0x03U);
  OV5640_WriteU16(0x3800U, (uint16_t)x0);
  OV5640_WriteU16(0x3802U, (uint16_t)y0);
  OV5640_WriteU16(0x3804U, x1);
  OV5640_WriteU16(0x3806U, y1);
  OV5640_WriteU16(0x3808U, gear->width);
  OV5640_WriteU16(0x380AU, gear->height);
  OV5640_WriteU16(0x380CU, hts);
  OV5640_WriteU16(0x380EU, vts);
  OV5640_WriteU16(0x3810U, 16U);
  OV5640_WriteU16(0x3812U, 8U);
  (void)SCCB_WriteReg(0x3212U, 0x13U);
  (void)SCCB_WriteReg(0x3212U, 0xA3U);

  (void)SCCB_WriteReg(0x3814U, 0x11U);
  (void)SCCB_WriteReg(0x3815U, 0x11U);
  /* 组写之外再写一次输出尺寸，避免行周期仍停在旧值。 */
  OV5640_WriteU16(0x3808U, gear->width);
  OV5640_WriteU16(0x380AU, gear->height);
  OV5640_WriteU16(0x380CU, hts);
  (void)SCCB_WriteReg(0x5001U, 0xA3U);
  if (ov5640_jpeg != 0U)
  {
    /* 压缩在传感器里做完。0x30/0x00 是 JPEG 输入，0x3002/0x3006 打开 JPEG 时钟。 */
    (void)SCCB_WriteReg(0x4300U, 0x30U);
    (void)SCCB_WriteReg(0x501FU, 0x00U);
    (void)SCCB_WriteReg(0x3002U, 0x1CU);
    (void)SCCB_WriteReg(0x3002U, 0x00U);
    (void)SCCB_WriteReg(0x3006U, 0xFFU);
    (void)SCCB_WriteReg(0x4713U, 0x03U);
    /* 量化档。数值越大压缩越狠、文件越小。 */
    (void)SCCB_WriteReg(0x4407U, 0x10U);
    OV5640_WriteU16(0x4602U, gear->width);
    OV5640_WriteU16(0x4604U, gear->height);
    (void)SCCB_WriteReg(0x460BU, 0x35U);
    (void)SCCB_WriteReg(0x460CU, 0x22U);
    (void)SCCB_WriteReg(0x3824U, 0x02U);
    (void)SCCB_WriteReg(0x4837U, 0x16U);
    /* 0x01：并行时钟不额外再除，内部行时钟再除 2。内部时钟快过并行口时 JPEG 会花。 */
    (void)SCCB_WriteReg(0x3108U, 0x01U);
  }
  else
  {
    /* LCD 扫的是 RGB565。0x6F 是这个屏用的字节序。输出宽度交给 VFIFO，避免多吐消隐。 */
    (void)SCCB_WriteReg(0x4300U, 0x6FU);
    (void)SCCB_WriteReg(0x501FU, 0x01U);
    (void)SCCB_WriteReg(0x4713U, 0x00U);
    OV5640_WriteU16(0x4602U, gear->width);
    OV5640_WriteU16(0x4604U, gear->height);
    (void)SCCB_WriteReg(0x3108U, 0x01U);
  }
  /* 快门手动，增益固定。上限收到当前帧内。 */
  OV5640_WriteU16(0x3A02U, exp_max);
  OV5640_WriteU16(0x3A14U, exp_max);
  (void)SCCB_WriteReg(0x3503U, 0x03U);
  (void)SCCB_WriteReg(0x350AU, 0x00U);
  (void)SCCB_WriteReg(0x350BU, cam_gain);
  (void)SCCB_WriteReg(0x3500U, (uint8_t)((exp_reg >> 16) & 0x0FU));
  (void)SCCB_WriteReg(0x3501U, (uint8_t)(exp_reg >> 8));
  (void)SCCB_WriteReg(0x3502U, (uint8_t)exp_reg);
  (void)SCCB_WriteReg(0x3035U, gear->pll);
  {
    uint8_t hi = 0U;
    uint8_t lo = 0U;

    if ((SCCB_ReadReg(0x3808U, &hi) == 0) && (SCCB_ReadReg(0x3809U, &lo) == 0))
    {
      ov5640_rb_w = (uint16_t)(((uint16_t)hi << 8) | lo);
    }
    if ((SCCB_ReadReg(0x380CU, &hi) == 0) && (SCCB_ReadReg(0x380DU, &lo) == 0))
    {
      ov5640_rb_hts = (uint16_t)(((uint16_t)hi << 8) | lo);
    }
    if (SCCB_ReadReg(0x3814U, &hi) == 0)
    {
      ov5640_rb_sub = hi;
    }
  }
}

/* 正点原子 RGB565 表。窗口仍是整幅抽样，输出尺寸后面再改成屏幕大小。 */
static const uint16_t ov5640_rgb565_lcd[] =
{
  0x4300, 0x6F, 0x501F, 0x01,
  0x3035, 0x41, 0x3036, 0x69,
  0x3C07, 0x07,
  0x3820, 0x40, 0x3821, 0x00,
  0x3814, 0x11, 0x3815, 0x11,
  0x3800, 0x00, 0x3801, 0x00, 0x3802, 0x00, 0x3803, 0x00,
  0x3804, 0x0A, 0x3805, 0x3F, 0x3806, 0x06, 0x3807, 0xA9,
  0x3808, 0x04, 0x3809, 0x00, 0x380A, 0x02, 0x380B, 0x58,
  0x380C, 0x05, 0x380D, 0xF8, 0x380E, 0x03, 0x380F, 0x84,
  0x3813, 0x04,
  0x3618, 0x00, 0x3612, 0x29, 0x3709, 0x52, 0x370C, 0x03,
  0x3A02, 0x02, 0x3A03, 0xE0, 0x3A14, 0x02, 0x3A15, 0xE0,
  0x4004, 0x02, 0x3002, 0x1C, 0x3006, 0xC3,
  0x4713, 0x03, 0x4407, 0x04, 0x460B, 0x37, 0x460C, 0x20,
  0x4837, 0x16, 0x3824, 0x04, 0x5001, 0xA3, 0x3503, 0x03,
};

static void OV5640_Group(uint8_t phase)
{
  (void)SCCB_WriteReg(0x3212U, phase);
}

/* 0：不写传感器。非 0：写进传感器。上电默认写入。 */
volatile uint8_t ov5640_win_write = 1U;
/* ShowOnLcd 先记下窗口，行场和输出尺寸一起写，避免先写成读不完的大窗口。 */
static uint8_t ov5640_win_hold;
/* 上电窗口：中心在画面原点，大小跟屏幕一致。 */
static uint16_t ov5640_win_x;
static uint16_t ov5640_win_y;
static uint16_t ov5640_win_w = 1024U;
static uint16_t ov5640_win_h = 600U;
static uint16_t ov5640_cap_w = 1024U;
static uint16_t ov5640_cap_h = 600U;
static int16_t ov5640_src_x;
static int16_t ov5640_src_y;
static volatile uint16_t ov5640_view_x;
static volatile uint16_t ov5640_view_y;
static volatile uint16_t ov5640_view_w = 1024U;
static volatile uint16_t ov5640_view_h = 600U;
/* 跟 CAM_SHUTTER_US、CAM_GAIN 一致。调试器改过之后以这里为准，开窗重写预览时不再退回常量。 */
static uint32_t ov5640_shutter_us = 1000U;
static uint16_t ov5640_gain = 0x00F0U;
static uint16_t ov5640_hts = 2308U;
static uint16_t ov5640_vts_min = 900U;
static uint8_t ov5640_native;
/* 小窗定时：默认值与原先写死的常数相同（升针模式不变）。梅花轮模式按实测时钟改写。 */
uint32_t ov5640_pclk_hz = 2625000000UL / (6U * 4U);
uint8_t  ov5640_native_fps = 50U;

void OV5640_SetWindowPos(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
  uint16_t x0;
  uint16_t y0;
  uint16_t x1;
  uint16_t y1;
  uint16_t y_max;

  if (width > OV5640_ARRAY_W)
  {
    width = (uint16_t)OV5640_ARRAY_W;
  }
  if (height > OV5640_ARRAY_H)
  {
    height = (uint16_t)OV5640_ARRAY_H;
  }
  if (width < 2U)
  {
    width = 2U;
  }
  if (height < 2U)
  {
    height = 2U;
  }
  width = (uint16_t)(width & (uint16_t)~1U);
  height = (uint16_t)(height & (uint16_t)~1U);
  if (x > (uint16_t)(OV5640_ARRAY_W - width))
  {
    x = (uint16_t)(OV5640_ARRAY_W - width);
  }
  y_max = (uint16_t)(OV5640_ARRAY_Y + OV5640_ARRAY_H - height);
  if (y < OV5640_ARRAY_Y)
  {
    y = (uint16_t)OV5640_ARRAY_Y;
  }
  if (y > y_max)
  {
    y = y_max;
  }
  x0 = (uint16_t)(x & (uint16_t)~1U);
  y0 = (uint16_t)(y & (uint16_t)~1U);
  x1 = (uint16_t)(x0 + width - 1U);
  y1 = (uint16_t)(y0 + height - 1U);
  ov5640_win_x = x0;
  ov5640_win_y = y0;
  ov5640_win_w = width;
  ov5640_win_h = height;
  if ((ov5640_win_write == 0U) || (ov5640_win_hold != 0U))
  {
    return;
  }

  OV5640_Group(0x03U);
  OV5640_WriteU16(0x3800U, x0);
  OV5640_WriteU16(0x3802U, y0);
  OV5640_WriteU16(0x3804U, x1);
  OV5640_WriteU16(0x3806U, y1);
  OV5640_Group(0x13U);
  OV5640_Group(0xA3U);
  OV5640_WriteU16(0x3800U, x0);
  OV5640_WriteU16(0x3802U, y0);
  OV5640_WriteU16(0x3804U, x1);
  OV5640_WriteU16(0x3806U, y1);
}

static void OV5640_WriteGainReg(void)
{
  uint16_t gain = ov5640_gain;

  if (gain > 0x03FFU)
  {
    gain = 0x03FFU;
  }
  (void)SCCB_WriteReg(0x3A18U, 0x03U);
  (void)SCCB_WriteReg(0x3A19U, 0xFFU);
  (void)SCCB_WriteReg(0x350AU, (uint8_t)((gain >> 8) & 0x03U));
  (void)SCCB_WriteReg(0x350BU, (uint8_t)gain);
}

void OV5640_SetExposure(uint32_t shutter_us)
{
  uint8_t hi = 0U;
  uint8_t lo = 0U;
  uint16_t hts = 2308U;
  uint16_t vts;
  uint32_t pclk_hz = 2625000000UL / (6U * 4U);
  uint32_t exp_reg;
  uint32_t exp_lines;
  uint32_t exp_max_lines;

  ov5640_shutter_us = shutter_us;
  if ((SCCB_ReadReg(0x380CU, &hi) == 0) && (SCCB_ReadReg(0x380DU, &lo) == 0))
  {
    hts = (uint16_t)(((uint16_t)hi << 8) | lo);
  }
  if (hts < 2U)
  {
    hts = 2308U;
  }
  exp_reg = (uint32_t)(((uint64_t)shutter_us * pclk_hz * 16ULL) / ((uint64_t)hts * 1000000ULL));
  if (exp_reg < 16U)
  {
    exp_reg = 16U;
  }
  exp_lines = (exp_reg + 15U) >> 4;
  if (exp_lines > (65535U - 4U))
  {
    exp_lines = 65535U - 4U;
    exp_reg = exp_lines << 4;
  }
  vts = ov5640_vts_min;
  if ((uint32_t)vts < (exp_lines + 4U))
  {
    vts = (uint16_t)(exp_lines + 4U);
  }
  if (vts < 8U)
  {
    vts = 8U;
  }
  exp_max_lines = (uint32_t)vts - 4U;
  /* 0x3503 整字节写 0x03。只置其中一位会清掉另一位，自动曝光或自动增益会把亮度拉回去。 */
  (void)SCCB_WriteReg(0x3503U, 0x03U);
  OV5640_WriteU16(0x380EU, vts);
  OV5640_WriteU16(0x3A02U, (uint16_t)exp_max_lines);
  OV5640_WriteU16(0x3A14U, (uint16_t)exp_max_lines);
  (void)SCCB_WriteReg(0x3500U, (uint8_t)((exp_reg >> 16) & 0x0FU));
  (void)SCCB_WriteReg(0x3501U, (uint8_t)(exp_reg >> 8));
  (void)SCCB_WriteReg(0x3502U, (uint8_t)exp_reg);
  OV5640_WriteGainReg();
  (void)SCCB_WriteReg(0x3503U, 0x03U);
  ov5640_rb_exp = exp_reg;
}

void OV5640_SetGain(uint16_t gain)
{
  if (gain > 0x03FFU)
  {
    gain = 0x03FFU;
  }
  ov5640_gain = gain;
  (void)SCCB_WriteReg(0x3503U, 0x03U);
  OV5640_WriteGainReg();
  (void)SCCB_WriteReg(0x3503U, 0x03U);
}

/* 50Hz 灯光一个明暗周期是 10ms。步长（行）= PCLK / HTS / 100，自动曝光按这个整数倍取快门。 */
static void OV5640_ApplyBand50(uint16_t hts, uint16_t vts)
{
  uint32_t pclk_hz = ov5640_pclk_hz;
  uint32_t line_hz;
  uint32_t step50;
  uint32_t step60;
  uint32_t max50;
  uint32_t max60;
  uint8_t reg;

  if (hts < 2U)
  {
    hts = 2U;
  }
  if (vts < 8U)
  {
    vts = 900U;
  }
  line_hz = (uint32_t)hts * 100U;
  step50 = (pclk_hz + (line_hz / 2U)) / line_hz;
  line_hz = (uint32_t)hts * 120U;
  step60 = (pclk_hz + (line_hz / 2U)) / line_hz;
  if (step50 < 1U)
  {
    step50 = 1U;
  }
  if (step60 < 1U)
  {
    step60 = 1U;
  }
  if ((uint32_t)vts < (step50 + 4U))
  {
    vts = (uint16_t)(step50 + 4U);
    OV5640_WriteU16(0x380EU, vts);
  }
  max50 = ((uint32_t)vts - 4U) / step50;
  max60 = ((uint32_t)vts - 4U) / step60;
  if (max50 < 1U)
  {
    max50 = 1U;
  }
  if (max60 < 1U)
  {
    max60 = 1U;
  }
  if (max50 > 255U)
  {
    max50 = 255U;
  }
  if (max60 > 255U)
  {
    max60 = 255U;
  }
  OV5640_WriteU16(0x3A08U, (uint16_t)step50);
  OV5640_WriteU16(0x3A0AU, (uint16_t)step60);
  (void)SCCB_WriteReg(0x3A0EU, (uint8_t)max50);
  (void)SCCB_WriteReg(0x3A0DU, (uint8_t)max60);
  OV5640_WriteU16(0x3A02U, (uint16_t)((uint32_t)vts - 4U));
  OV5640_WriteU16(0x3A14U, (uint16_t)((uint32_t)vts - 4U));
  (void)SCCB_WriteReg(0x3A18U, 0x03U);
  (void)SCCB_WriteReg(0x3A19U, 0xFFU);
  if (SCCB_ReadReg(0x3A00U, &reg) != 0)
  {
    reg = 0x78U;
  }
  /* bit5 打开 band，bit4 关掉自动判断，固定走下面的 50Hz。 */
  reg = (uint8_t)((reg | 0x20U) & 0xEFU);
  (void)SCCB_WriteReg(0x3A00U, reg);
  if (SCCB_ReadReg(0x3C01U, &reg) != 0)
  {
    reg = 0x34U;
  }
  reg |= 0x80U;
  (void)SCCB_WriteReg(0x3C01U, reg);
  if (SCCB_ReadReg(0x3C00U, &reg) != 0)
  {
    reg = 0x00U;
  }
  reg |= 0x04U;
  (void)SCCB_WriteReg(0x3C00U, reg);
  /* bit0、bit1 都清掉：自动曝光才按步长走，自动增益才接得住 10ms 这一档。 */
  (void)SCCB_WriteReg(0x3503U, 0x00U);
}

static void OV5640_FitOutput(void)
{
  uint16_t width = ov5640_win_w;
  uint16_t height = ov5640_win_h;
  uint32_t out_w;
  uint32_t out_h;
  uint16_t cap_w;
  uint16_t cap_h;
  uint16_t hts;
  int32_t x_off = 16;
  int32_t y_off = 4;
  int32_t x0;
  int32_t y0;
  int32_t x1;
  int32_t y1;
  int32_t x_max = (int32_t)OV5640_ARRAY_W - 1;
  int32_t y_max = (int32_t)OV5640_ARRAY_Y + (int32_t)OV5640_ARRAY_H - 1;
  uint8_t reg20 = 0x40U;
  uint8_t reg21 = 0x06U;
  uint8_t misc = 0U;

  /* 转速模式：1:1 原尺寸居中，不放大填屏。普通预览仍按比例贴边留黑。 */
  if (ov5640_native != 0U)
  {
    out_w = width;
    out_h = height;
    if (out_w > LCD_WIDTH)
    {
      out_w = LCD_WIDTH;
    }
    if (out_h > LCD_HEIGHT)
    {
      out_h = LCD_HEIGHT;
    }
  }
  else if ((uint32_t)width * LCD_HEIGHT >= (uint32_t)height * LCD_WIDTH)
  {
    out_w = LCD_WIDTH;
    out_h = ((uint32_t)height * LCD_WIDTH) / width;
  }
  else
  {
    out_h = LCD_HEIGHT;
    out_w = ((uint32_t)width * LCD_HEIGHT) / height;
  }
  out_w &= ~1U;
  out_h &= ~1U;
  if (out_w < 2U)
  {
    out_w = 2U;
  }
  if (out_h < 2U)
  {
    out_h = 2U;
  }
  ov5640_view_w = (uint16_t)out_w;
  ov5640_view_h = (uint16_t)out_h;
  ov5640_view_x = (uint16_t)(((LCD_WIDTH - out_w) / 2U) & ~1U);
  ov5640_view_y = (uint16_t)(((LCD_HEIGHT - out_h) / 2U) & ~1U);
  /* 传感器只能缩小。开窗比屏幕目标小时按原尺寸吐出，放大放到 MCU。 */
  if (((uint32_t)width < out_w) || ((uint32_t)height < out_h))
  {
    cap_w = width;
    cap_h = height;
  }
  else
  {
    cap_w = (uint16_t)out_w;
    cap_h = (uint16_t)out_h;
  }
  ov5640_cap_w = cap_w;
  ov5640_cap_h = cap_h;
  if (ov5640_win_write == 0U)
  {
    return;
  }

  /* 预缩放区域 = ISP 开窗减去两侧偏移。偏移留着，1:1 时缩放器才不会旁路。 */
  x0 = (int32_t)ov5640_win_x - x_off;
  y0 = (int32_t)ov5640_win_y - y_off;
  x1 = (int32_t)ov5640_win_x + (int32_t)width - 1 + x_off;
  y1 = (int32_t)ov5640_win_y + (int32_t)height - 1 + y_off;
  if (x0 < 0)
  {
    x_off += x0;
    x0 = 0;
  }
  if (y0 < 0)
  {
    y_off += y0;
    y0 = 0;
  }
  if (x1 > x_max)
  {
    x_off -= (x1 - x_max);
    x1 = x_max;
  }
  if (y1 > y_max)
  {
    y_off -= (y1 - y_max);
    y1 = y_max;
  }
  if (x_off < 0)
  {
    x_off = 0;
  }
  if (y_off < 0)
  {
    y_off = 0;
  }
  {
    uint32_t isp_w = (uint32_t)(x1 - x0 + 1);
    uint32_t isp_h = (uint32_t)(y1 - y0 + 1);
    uint32_t pre_w = isp_w;
    uint32_t pre_h = isp_h;

    if ((uint32_t)(x_off * 2) < isp_w)
    {
      pre_w = isp_w - (uint32_t)(x_off * 2);
    }
    if ((uint32_t)(y_off * 2) < isp_h)
    {
      pre_h = isp_h - (uint32_t)(y_off * 2);
    }
    uint32_t timing_w;
    uint32_t timing_h;
    uint32_t hneed;
    uint32_t vneed;
    uint16_t vts;
    uint8_t sub = 0x11U;
    int bin = 0;

    if ((pre_w > ((uint32_t)cap_w * 2U)) || (pre_h > ((uint32_t)cap_h * 2U)))
    {
      /* 超过两倍先 2x2 抽样，剩下的交给缩放器。抽样和 binning 必须一起开。 */
      bin = 1;
      sub = 0x31U;
    }
    (void)SCCB_WriteReg(0x3814U, sub);
    (void)SCCB_WriteReg(0x3815U, sub);
    if (SCCB_ReadReg(0x3820U, &reg20) != 0)
    {
      reg20 = 0x40U;
    }
    if (SCCB_ReadReg(0x3821U, &reg21) != 0)
    {
      reg21 = 0x06U;
    }
    if (bin != 0)
    {
      reg20 |= 0x01U;
      reg21 |= 0x01U;
    }
    else
    {
      reg20 &= 0xFEU;
      reg21 &= 0xFEU;
    }
    (void)SCCB_WriteReg(0x3820U, reg20);
    (void)SCCB_WriteReg(0x3821U, reg21);
    (void)SCCB_WriteReg(0x5001U, 0xA3U);
    if (SCCB_ReadReg(0x501DU, &misc) != 0)
    {
      misc = 0U;
    }
    (void)SCCB_WriteReg(0x501DU, (uint8_t)(misc & 0xBFU));
    /* 行周期要盖住正在读的窗口，不能只按缩小后的输出宽算。否则大窗口读不完。 */
    timing_w = (bin != 0) ? (isp_w / 2U) : isp_w;
    timing_h = (bin != 0) ? (isp_h / 2U) : isp_h;
    hneed = timing_w + 200U;
    if (((uint32_t)cap_w * 2U + 8U) > hneed)
    {
      hneed = (uint32_t)cap_w * 2U + 8U;
    }
    /* 预览和大窗必须盖住读出窗口。小窗也沿用同一行周期下限，
     * 帧率改用拉长 VTS 到约 50 fps；行时间骤降时 ISP 会停，不再出同步。 */
    if (hneed < 0x0790U)
    {
      hneed = 0x0790U;
    }
    hts = (uint16_t)(hneed + 252U);
    vneed = timing_h + 32U;
    if (((uint32_t)cap_h + 32U) > vneed)
    {
      vneed = (uint32_t)cap_h + 32U;
    }
    if (ov5640_native == 0U)
    {
      if (vneed < 0x0384U)
      {
        vneed = 0x0384U;
      }
    }
    vts = (uint16_t)vneed;
    /* 小窗如果只按读完窗口来算行场，帧率会到几百，行中断跟不上，DCMI 会停。
     * PCLK ≈ 109.375 MHz，把 HTS*VTS 拉到约 50 fps，既高于 10 齿/s 所需，又跑得稳。 */
    if (ov5640_native != 0U)
    {
      uint32_t pclk_hz = ov5640_pclk_hz;
      uint32_t target = pclk_hz / (uint32_t)((ov5640_native_fps != 0U) ? ov5640_native_fps : 50U);
      uint32_t prod = (uint32_t)hts * (uint32_t)vts;
      if (prod < target)
      {
        uint32_t v2 = (target + (uint32_t)hts - 1U) / (uint32_t)hts;
        if (v2 > 0xFFFFU)
        {
          v2 = 0xFFFFU;
        }
        if (v2 > (uint32_t)vts)
        {
          vts = (uint16_t)v2;
        }
      }
    }
    ov5640_hts = hts;
    ov5640_vts_min = vts;
    OV5640_Group(0x03U);
    OV5640_WriteU16(0x3800U, (uint16_t)x0);
    OV5640_WriteU16(0x3802U, (uint16_t)y0);
    OV5640_WriteU16(0x3804U, (uint16_t)x1);
    OV5640_WriteU16(0x3806U, (uint16_t)y1);
    OV5640_WriteU16(0x3808U, cap_w);
    OV5640_WriteU16(0x380AU, cap_h);
    OV5640_WriteU16(0x380CU, hts);
    OV5640_WriteU16(0x380EU, vts);
    OV5640_WriteU16(0x3810U, (uint16_t)x_off);
    OV5640_WriteU16(0x3812U, (uint16_t)y_off);
    OV5640_Group(0x13U);
    OV5640_Group(0xA3U);
    OV5640_WriteU16(0x3800U, (uint16_t)x0);
    OV5640_WriteU16(0x3802U, (uint16_t)y0);
    OV5640_WriteU16(0x3804U, (uint16_t)x1);
    OV5640_WriteU16(0x3806U, (uint16_t)y1);
    OV5640_WriteU16(0x3808U, cap_w);
    OV5640_WriteU16(0x380AU, cap_h);
    OV5640_WriteU16(0x380CU, hts);
    OV5640_WriteU16(0x380EU, vts);
    OV5640_WriteU16(0x3810U, (uint16_t)x_off);
    OV5640_WriteU16(0x3812U, (uint16_t)y_off);
    OV5640_ApplyBand50(hts, vts);
  }
}

void OV5640_ShowOnLcd(int16_t x, int16_t y, uint16_t width, uint16_t height)
{
  int32_t left;
  int32_t top;

  ov5640_src_x = x;
  ov5640_src_y = y;
  /* 画面中心是原点。开窗中心落在中心点再加 (x, y)。 */
  left = ((int32_t)OV5640_ARRAY_W / 2) + (int32_t)x - (int32_t)(width / 2U);
  top = (int32_t)OV5640_ARRAY_Y + ((int32_t)OV5640_ARRAY_H / 2) + (int32_t)y - (int32_t)(height / 2U);
  if (left < 0)
  {
    left = 0;
  }
  if (top < 0)
  {
    top = 0;
  }
  ov5640_win_hold = 1U;
  OV5640_SetWindowPos((uint16_t)left, (uint16_t)top, width, height);
  ov5640_win_hold = 0U;
  OV5640_FitOutput();
}

void OV5640_GetView(uint16_t *x, uint16_t *y, uint16_t *width, uint16_t *height)
{
  *x = ov5640_view_x;
  *y = ov5640_view_y;
  *width = ov5640_view_w;
  *height = ov5640_view_h;
}

void OV5640_GetCapture(uint16_t *width, uint16_t *height)
{
  *width = ov5640_cap_w;
  *height = ov5640_cap_h;
}

void OV5640_GetWindow(uint16_t *x, uint16_t *y, uint16_t *width, uint16_t *height)
{
  *x = ov5640_win_x;
  *y = ov5640_win_y;
  *width = ov5640_win_w;
  *height = ov5640_win_h;
}

void OV5640_SetNativeOutput(uint8_t on)
{
  ov5640_native = (on != 0U) ? 1U : 0U;
}

void OV5640_SetNativeWindow(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
  int32_t src_x;
  int32_t src_y;

  if (width > LCD_WIDTH)
  {
    width = LCD_WIDTH;
  }
  if (height > LCD_HEIGHT)
  {
    height = LCD_HEIGHT;
  }
  width = (uint16_t)(width & (uint16_t)~1U);
  height = (uint16_t)(height & (uint16_t)~1U);
  ov5640_native = 1U;
  src_x = (int32_t)x + (int32_t)(width / 2U) - ((int32_t)OV5640_ARRAY_W / 2);
  src_y = (int32_t)y + (int32_t)(height / 2U)
          - ((int32_t)OV5640_ARRAY_Y + ((int32_t)OV5640_ARRAY_H / 2));
  ov5640_src_x = (int16_t)src_x;
  ov5640_src_y = (int16_t)src_y;
  ov5640_win_hold = 1U;
  OV5640_SetWindowPos(x, y, width, height);
  ov5640_win_hold = 0U;
  OV5640_FitOutput();
}

void OV5640_SetStream(uint8_t on)
{
  /* 改窗口前先软件掉电，避免行场卡在半帧后不再出同步。 */
  (void)SCCB_WriteReg(0x3008U, (on != 0U) ? 0x02U : 0x42U);
}

void OV5640_SnapRegs(uint16_t *sys, uint16_t *out_w, uint16_t *hts, uint16_t *vts)
{
  uint8_t v = 0U;
  uint8_t hi = 0U;
  uint8_t lo = 0U;

  (void)SCCB_ReadReg(0x3008U, &v);
  if (sys != 0)
  {
    *sys = v;
  }
  (void)SCCB_ReadReg(0x3808U, &hi);
  (void)SCCB_ReadReg(0x3809U, &lo);
  if (out_w != 0)
  {
    *out_w = (uint16_t)(((uint16_t)hi << 8) | lo);
  }
  (void)SCCB_ReadReg(0x380CU, &hi);
  (void)SCCB_ReadReg(0x380DU, &lo);
  if (hts != 0)
  {
    *hts = (uint16_t)(((uint16_t)hi << 8) | lo);
  }
  (void)SCCB_ReadReg(0x380EU, &hi);
  (void)SCCB_ReadReg(0x380FU, &lo);
  if (vts != 0)
  {
    *vts = (uint16_t)(((uint16_t)hi << 8) | lo);
  }
}

static uint16_t OV5640_ReadU16(uint16_t reg)
{
  uint8_t hi = 0U;
  uint8_t lo = 0U;

  (void)SCCB_ReadReg(reg, &hi);
  (void)SCCB_ReadReg((uint16_t)(reg + 1U), &lo);
  return (uint16_t)(((uint16_t)hi << 8) | lo);
}

void OV5640_SnapCropRegs(uint16_t *out_h, uint16_t *x0, uint16_t *y0, uint16_t *x1, uint16_t *y1,
                         uint8_t *sub, uint8_t *r20, uint8_t *r21, uint8_t *dvp)
{
  uint8_t v = 0U;

  if (out_h != 0)
  {
    *out_h = OV5640_ReadU16(0x380AU);
  }
  if (x0 != 0)
  {
    *x0 = OV5640_ReadU16(0x3800U);
  }
  if (y0 != 0)
  {
    *y0 = OV5640_ReadU16(0x3802U);
  }
  if (x1 != 0)
  {
    *x1 = OV5640_ReadU16(0x3804U);
  }
  if (y1 != 0)
  {
    *y1 = OV5640_ReadU16(0x3806U);
  }
  if (sub != 0)
  {
    v = 0U;
    (void)SCCB_ReadReg(0x3814U, &v);
    *sub = v;
  }
  if (r20 != 0)
  {
    v = 0U;
    (void)SCCB_ReadReg(0x3820U, &v);
    *r20 = v;
  }
  if (r21 != 0)
  {
    v = 0U;
    (void)SCCB_ReadReg(0x3821U, &v);
    *r21 = v;
  }
  if (dvp != 0)
  {
    v = 0U;
    (void)SCCB_ReadReg(0x3017U, &v);
    *dvp = v;
  }
}




static void OV5640_RgbLcdSetup(void)
{
  static const uint8_t light_auto[7] = { 0x04U, 0x00U, 0x04U, 0x00U, 0x04U, 0x00U, 0x00U };
  static const uint8_t sat0[6] = { 0x1AU, 0x66U, 0x80U, 0x82U, 0x80U, 0x02U };
  uint8_t i;

  (void)OV5640_WriteList(ov5640_rgb565_lcd,
                         (uint32_t)(sizeof(ov5640_rgb565_lcd) / sizeof(ov5640_rgb565_lcd[0]) / 2U));

  /* 例程在 RGB 屏上把 0x3821 写成 0x06，画面左右才和实物一致。 */
  (void)SCCB_WriteReg(0x3820U, 0x40U);
  (void)SCCB_WriteReg(0x3821U, 0x06U);

  OV5640_Group(0x03U);
  for (i = 0U; i < 7U; i++)
  {
    (void)SCCB_WriteReg((uint16_t)(0x3400U + i), light_auto[i]);
  }
  OV5640_Group(0x13U);
  OV5640_Group(0xA3U);

  OV5640_Group(0x03U);
  (void)SCCB_WriteReg(0x5381U, 0x1CU);
  (void)SCCB_WriteReg(0x5382U, 0x5AU);
  (void)SCCB_WriteReg(0x5383U, 0x06U);
  for (i = 0U; i < 6U; i++)
  {
    (void)SCCB_WriteReg((uint16_t)(0x5384U + i), sat0[i]);
  }
  (void)SCCB_WriteReg(0x538BU, 0x98U);
  (void)SCCB_WriteReg(0x538AU, 0x01U);
  OV5640_Group(0x13U);
  OV5640_Group(0xA3U);

  /* 亮度 +2。0x5580 bit2 打开亮度，bit1 保持饱和度。快门不在这里改。 */
  (void)SCCB_WriteReg(0x5580U, 0x06U);
  OV5640_Group(0x03U);
  (void)SCCB_WriteReg(0x5587U, 0x10U);
  (void)SCCB_WriteReg(0x5588U, 0x01U);
  OV5640_Group(0x13U);
  OV5640_Group(0xA3U);

  OV5640_Group(0x03U);
  (void)SCCB_WriteReg(0x5585U, 0x00U);
  (void)SCCB_WriteReg(0x5586U, 0x20U);
  OV5640_Group(0x13U);
  OV5640_Group(0xA3U);

  (void)SCCB_WriteReg(0x5308U, 0x25U);
  (void)SCCB_WriteReg(0x5300U, 0x08U);
  (void)SCCB_WriteReg(0x5301U, 0x30U);
  (void)SCCB_WriteReg(0x5302U, 0x10U);
  (void)SCCB_WriteReg(0x5303U, 0x00U);
  (void)SCCB_WriteReg(0x5309U, 0x08U);
  (void)SCCB_WriteReg(0x530AU, 0x30U);
  (void)SCCB_WriteReg(0x530BU, 0x04U);
  (void)SCCB_WriteReg(0x530CU, 0x06U);
}

static void OV5640_RgbLcdFinish(void)
{
  /* 自动曝光的目标亮度。消纹步长在开窗写完行场之后再算。 */
  (void)SCCB_WriteReg(0x3A0FU, 0x60U);
  (void)SCCB_WriteReg(0x3A10U, 0x58U);
  (void)SCCB_WriteReg(0x3A1BU, 0x60U);
  (void)SCCB_WriteReg(0x3A1EU, 0x50U);
  (void)SCCB_WriteReg(0x3A11U, 0x80U);
  (void)SCCB_WriteReg(0x3A1FU, 0x40U);

  /* 组写会把亮度冲掉，最后再直接写一次。 */
  (void)SCCB_WriteReg(0x5580U, 0x06U);
  (void)SCCB_WriteReg(0x5587U, 0x10U);
  (void)SCCB_WriteReg(0x5588U, 0x01U);
  OV5640_ApplyBand50(ov5640_hts, ov5640_vts_min);

  {
    uint8_t hi = 0U;
    uint8_t lo = 0U;
    uint8_t frac = 0U;

    if ((SCCB_ReadReg(0x3808U, &hi) == 0) && (SCCB_ReadReg(0x3809U, &lo) == 0))
    {
      ov5640_rb_w = (uint16_t)(((uint16_t)hi << 8) | lo);
    }
    if ((SCCB_ReadReg(0x380CU, &hi) == 0) && (SCCB_ReadReg(0x380DU, &lo) == 0))
    {
      ov5640_rb_hts = (uint16_t)(((uint16_t)hi << 8) | lo);
    }
    if (SCCB_ReadReg(0x3814U, &hi) == 0)
    {
      ov5640_rb_sub = hi;
    }
    if ((SCCB_ReadReg(0x3500U, &hi) == 0) &&
        (SCCB_ReadReg(0x3501U, &lo) == 0) &&
        (SCCB_ReadReg(0x3502U, &frac) == 0))
    {
      ov5640_rb_exp = ((uint32_t)hi << 16) | ((uint32_t)lo << 8) | frac;
    }
    if (SCCB_ReadReg(0x5587U, &hi) == 0)
    {
      ov5640_rb_bright = hi;
    }
  }
}

void OV5640_PowerCycle(void)
{
  /* 硬件掉电再复位，让 ISP 和 DVP 从干净状态起步。 */
  Cam_Pwdn(1U);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_Delay(20);
  Cam_Pwdn(0U);
  HAL_Delay(5);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(20);
}

void OV5640_RgbLcdPreview(uint16_t width, uint16_t height)
{
  (void)width;
  (void)height;
  OV5640_RgbLcdSetup();
  /* 上电以画面中心为原点，再按同一比例缩进屏幕。 */
  OV5640_ShowOnLcd(ov5640_src_x, ov5640_src_y, ov5640_win_w, ov5640_win_h);
  OV5640_RgbLcdFinish();
}

static void OV5640_ReloadBase(void)
{
  /* 硬件复位后寄存器回到默认，必须把公共表和 DVP 引脚再写一遍，否则 0x3017 为 0，不会出同步。 */
  (void)SCCB_WriteReg(0x3103U, 0x11U);
  (void)SCCB_WriteReg(0x3008U, 0x82U);
  HAL_Delay(10);
  (void)OV5640_WriteList(ov5640_common,
                         (uint32_t)(sizeof(ov5640_common) / sizeof(ov5640_common[0]) / 2U));
  (void)OV5640_WriteList(ov5640_dvp,
                         (uint32_t)(sizeof(ov5640_dvp) / sizeof(ov5640_dvp[0]) / 2U));
  HAL_Delay(50);
  OV5640_FocusReset();
}

void OV5640_RgbLcdNativeWindow(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
  /* 复位后先恢复 DVP 和 ISP，再直接按小窗初始化。 */
  OV5640_ReloadBase();
  OV5640_RgbLcdSetup();
  OV5640_SetStream(0U);
  HAL_Delay(10);
  OV5640_SetNativeWindow(x, y, width, height);
  OV5640_SetStream(1U);
  OV5640_RgbLcdFinish();
}

int OV5640_SensorInit(void)
{
  uint8_t id_hi = 0U;
  uint8_t id_lo = 0U;
  uint8_t try_index;
  int id_ok = 0;

  OV5640_Pins();
  for (try_index = 0U; try_index < 3U; try_index++)
  {
    if (try_index != 0U)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
      HAL_Delay(20);
      Cam_Pwdn(0U);
      HAL_Delay(5);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
      HAL_Delay(20);
    }
    if ((SCCB_ReadReg(0x300AU, &id_hi) != 0) || (SCCB_ReadReg(0x300BU, &id_lo) != 0))
    {
      continue;
    }
    if ((((uint16_t)id_hi << 8) | id_lo) == 0x5640U)
    {
      id_ok = 1;
      break;
    }
  }
  if (id_ok == 0)
  {
    return -1;
  }
  if (SCCB_WriteReg(0x3103U, 0x11U) != 0)
  {
    return -1;
  }
  if (SCCB_WriteReg(0x3008U, 0x82U) != 0)
  {
    return -1;
  }
  HAL_Delay(10);
  if (OV5640_WriteList(ov5640_common, (uint32_t)(sizeof(ov5640_common) / sizeof(ov5640_common[0]) / 2U)) != 0)
  {
    return -1;
  }
  if (OV5640_WriteList(ov5640_dvp, (uint32_t)(sizeof(ov5640_dvp) / sizeof(ov5640_dvp[0]) / 2U)) != 0)
  {
    return -1;
  }
  /* 预览寄存器由 OV5640_RgbLcdPreview 写。对焦固件在预览启动后再由状态机下载。 */
  HAL_Delay(50);
  OV5640_FocusReset();
  return 0;
}

/* 预览启动后置位 focus_start；状态机下载对焦固件并单次对焦。也可用 J-Link 再置位一次。 */
uint8_t focus_start;
uint8_t focus_state;
uint16_t focus_index;
uint8_t focus_ready;
uint8_t focus_phase;
uint8_t focus_status;
static uint32_t focus_tick;
static uint32_t focus_poll;
static uint8_t focus_retries;
static uint32_t focus_saved_delay;

#define FOCUS_ST_IDLE    0U
#define FOCUS_ST_RESET   1U
#define FOCUS_ST_WRITE   2U
#define FOCUS_ST_BOOT    3U
#define FOCUS_ST_WAITFW  4U
#define FOCUS_ST_TRIG    5U
#define FOCUS_ST_WAITAF  6U
#define FOCUS_ST_DONE    7U
#define FOCUS_ST_FAIL    8U
#define FOCUS_CHUNK      16U

static int OV5640_WriteRetry(uint16_t reg, uint8_t value)
{
  uint8_t n;

  for (n = 0U; n < 3U; n++)
  {
    if (SCCB_WriteReg(reg, value) == 0)
    {
      return 0;
    }
  }
  return -1;
}

static void OV5640_FocusDelayRestore(void)
{
  if (focus_saved_delay != 0U)
  {
    sccb_delay_n = focus_saved_delay;
    focus_saved_delay = 0U;
  }
}

void OV5640_FocusReset(void)
{
  OV5640_FocusDelayRestore();
  focus_start = 0U;
  focus_state = FOCUS_ST_IDLE;
  focus_index = 0U;
  focus_ready = 0U;
  focus_phase = 0U;
  focus_status = 0U;
  focus_retries = 0U;
}

void OV5640_FocusLoad(void)
{
  focus_start = 1U;
}

void OV5640_FocusSweep(void)
{
  uint32_t n = (uint32_t)sizeof(OV5640_AF_Config);
  uint32_t chunk;
  uint32_t i;
  uint8_t state = 0xFFU;

  if (focus_start != 0U)
  {
    focus_start = 0U;
    OV5640_FocusDelayRestore();
    focus_ready = 0U;
    focus_phase = 3U;
    focus_status = 0U;
    focus_index = 0U;
    focus_retries = 0U;
    focus_state = FOCUS_ST_RESET;
  }

  switch (focus_state)
  {
    case FOCUS_ST_RESET:
      if (OV5640_WriteRetry(0x3000U, 0x20U) != 0)
      {
        focus_state = FOCUS_ST_FAIL;
        focus_phase = 1U;
        break;
      }
      focus_tick = HAL_GetTick();
      focus_state = FOCUS_ST_WRITE;
      break;

    case FOCUS_ST_WRITE:
      if ((HAL_GetTick() - focus_tick) < 5U)
      {
        break;
      }
      if (focus_saved_delay == 0U)
      {
        focus_saved_delay = sccb_delay_n;
        sccb_delay_n = 400U;
      }
      chunk = n - (uint32_t)focus_index;
      if (chunk > FOCUS_CHUNK)
      {
        chunk = FOCUS_CHUNK;
      }
      for (i = 0U; i < chunk; i++)
      {
        if (OV5640_WriteRetry((uint16_t)(0x8000U + focus_index), OV5640_AF_Config[focus_index]) != 0)
        {
          OV5640_FocusDelayRestore();
          focus_state = FOCUS_ST_FAIL;
          focus_phase = 1U;
          return;
        }
        focus_index++;
      }
      if ((uint32_t)focus_index >= n)
      {
        OV5640_FocusDelayRestore();
        focus_state = FOCUS_ST_BOOT;
      }
      break;

    case FOCUS_ST_BOOT:
      (void)SCCB_WriteReg(0x3022U, 0x00U);
      (void)SCCB_WriteReg(0x3023U, 0x00U);
      (void)SCCB_WriteReg(0x3024U, 0x00U);
      (void)SCCB_WriteReg(0x3025U, 0x00U);
      (void)SCCB_WriteReg(0x3026U, 0x00U);
      (void)SCCB_WriteReg(0x3027U, 0x00U);
      (void)SCCB_WriteReg(0x3028U, 0x00U);
      (void)SCCB_WriteReg(0x3029U, 0x7FU);
      if (OV5640_WriteRetry(0x3000U, 0x00U) != 0)
      {
        focus_state = FOCUS_ST_FAIL;
        focus_phase = 1U;
        break;
      }
      focus_tick = HAL_GetTick();
      focus_state = FOCUS_ST_WAITFW;
      break;

    case FOCUS_ST_WAITFW:
      {
        uint32_t now = HAL_GetTick();
        if ((now - focus_tick) > 5000U)
        {
          focus_state = FOCUS_ST_FAIL;
          focus_phase = 1U;
          break;
        }
        if (SCCB_ReadReg(0x3029U, &state) == 0)
        {
          focus_status = state;
          if (state == 0x70U)
          {
            focus_ready = 1U;
            focus_state = FOCUS_ST_TRIG;
          }
        }
      }
      break;

    case FOCUS_ST_TRIG:
      (void)SCCB_WriteReg(0x3022U, 0x03U);
      focus_retries = 0U;
      focus_tick = HAL_GetTick();
      focus_poll = focus_tick;
      focus_state = FOCUS_ST_WAITAF;
      break;

    case FOCUS_ST_WAITAF:
      if ((HAL_GetTick() - focus_poll) < 50U)
      {
        break;
      }
      focus_poll = HAL_GetTick();
      if (SCCB_ReadReg(0x3029U, &state) == 0)
      {
        focus_status = state;
      }
      if (state == 0x10U)
      {
        focus_phase = 2U;
        focus_state = FOCUS_ST_DONE;
        break;
      }
      if ((state == 0x20U) || ((HAL_GetTick() - focus_tick) > 2000U))
      {
        if (focus_retries < 4U)
        {
          focus_retries++;
          (void)SCCB_WriteReg(0x3022U, 0x03U);
          focus_tick = HAL_GetTick();
          focus_poll = focus_tick;
        }
        else
        {
          focus_phase = 1U;
          focus_state = FOCUS_ST_FAIL;
        }
      }
      break;

    default:
      break;
  }
}

uint8_t OV5640_FocusState(void)
{
  if (focus_ready == 0U)
  {
    return 0U;
  }
  return focus_phase;
}
