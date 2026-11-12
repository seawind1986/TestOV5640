#include "jpeg_enc.h"

#include <string.h>

static const int16_t cos_q14[8][8] = {
  { 16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384 },
  { 16069, 13623, 9102, 3196, -3196, -9102, -13623, -16069 },
  { 15137, 6270, -6270, -15137, -15137, -6270, 6270, 15137 },
  { 13623, -3196, -16069, -9102, 9102, 16069, 3196, -13623 },
  { 11585, -11585, -11585, 11585, 11585, -11585, -11585, 11585 },
  { 9102, -16069, 3196, 13623, -13623, -3196, 16069, -9102 },
  { 6270, -15137, 15137, -6270, -6270, 15137, -15137, 6270 },
  { 3196, -9102, 13623, -16069, 16069, -13623, 9102, -3196 },
};

static const uint8_t std_luma[64] = {
  16, 11, 10, 16, 24, 40, 51, 61,
  12, 12, 14, 19, 26, 58, 60, 55,
  14, 13, 16, 24, 40, 57, 69, 56,
  14, 17, 22, 29, 51, 87, 80, 62,
  18, 22, 37, 56, 68, 109, 103, 77,
  24, 35, 55, 64, 81, 104, 113, 92,
  49, 64, 78, 87, 103, 121, 120, 101,
  72, 92, 95, 98, 112, 100, 103, 99
};

static const uint8_t zigzag[64] = {
  0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
  12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
  35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
  58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63
};

static const uint8_t dc_bits[16] = { 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
static const uint8_t dc_val[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
static const uint8_t ac_bits[16] = { 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d };
static const uint8_t ac_val[162] = {
  0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
  0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
  0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
  0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
  0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
  0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
  0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
  0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
  0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
  0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
  0xf9, 0xfa
};

typedef struct
{
  uint16_t code;
  uint8_t len;
} HuffCode;

static HuffCode dc_code[12];
static HuffCode ac_code[256];
static uint8_t quant[64];
static uint8_t tables_ready;

static uint8_t *out_p;
static uint8_t *out_end;
static uint32_t bit_buf;
static uint8_t bit_n;
static uint8_t overflow;

static void tables_init(void)
{
  uint32_t i;
  uint32_t k;
  uint32_t code;
  uint32_t scale;

  if (tables_ready != 0U)
  {
    return;
  }

  code = 0U;
  k = 0U;
  for (i = 0U; i < 16U; i++)
  {
    uint32_t j;
    for (j = 0U; j < dc_bits[i]; j++)
    {
      dc_code[dc_val[k]].code = (uint16_t)code;
      dc_code[dc_val[k]].len = (uint8_t)(i + 1U);
      code++;
      k++;
    }
    code <<= 1;
  }

  memset(ac_code, 0, sizeof(ac_code));
  code = 0U;
  k = 0U;
  for (i = 0U; i < 16U; i++)
  {
    uint32_t j;
    for (j = 0U; j < ac_bits[i]; j++)
    {
      ac_code[ac_val[k]].code = (uint16_t)code;
      ac_code[ac_val[k]].len = (uint8_t)(i + 1U);
      code++;
      k++;
    }
    code <<= 1;
  }

  /* 质量约 90，二值笔画尽量少被量化抹掉。 */
  scale = 20U;
  for (i = 0U; i < 64U; i++)
  {
    uint32_t q = ((uint32_t)std_luma[i] * scale + 50U) / 100U;
    quant[i] = (q == 0U) ? 1U : (uint8_t)q;
  }
  tables_ready = 1U;
}

static void put_byte(uint8_t value)
{
  if (out_p >= out_end)
  {
    overflow = 1U;
    return;
  }
  *out_p++ = value;
}

static void put_bytes(const uint8_t *data, uint32_t len)
{
  uint32_t i;
  for (i = 0U; i < len; i++)
  {
    put_byte(data[i]);
  }
}

static void put_u16(uint16_t value)
{
  put_byte((uint8_t)(value >> 8));
  put_byte((uint8_t)value);
}

static void put_bits(uint32_t code, uint8_t len)
{
  bit_buf = (bit_buf << len) | (code & ((1UL << len) - 1UL));
  bit_n = (uint8_t)(bit_n + len);
  while (bit_n >= 8U)
  {
    uint8_t byte;
    bit_n = (uint8_t)(bit_n - 8U);
    byte = (uint8_t)((bit_buf >> bit_n) & 0xFFU);
    put_byte(byte);
    if (byte == 0xFFU)
    {
      put_byte(0x00U);
    }
  }
}

static void put_huff(const HuffCode *huff)
{
  put_bits(huff->code, huff->len);
}

static void flush_bits(void)
{
  if (bit_n > 0U)
  {
    put_bits(0x7FU, (uint8_t)(8U - bit_n));
  }
  bit_buf = 0U;
  bit_n = 0U;
}

static int magnitude(int value)
{
  int n = 0;
  int abs_v = (value < 0) ? -value : value;
  while ((abs_v >> n) != 0 && n < 15)
  {
    n++;
  }
  return n;
}

static void encode_coeff(int value, const HuffCode *huff)
{
  int n = magnitude(value);
  int mag = value;
  put_huff(huff);
  if (n > 0)
  {
    if (mag < 0)
    {
      mag += (1 << n) - 1;
    }
    put_bits((uint32_t)mag, (uint8_t)n);
  }
}

static void fdct(const int16_t in[8][8], int16_t out[8][8])
{
  int32_t tmp[8][8];
  int u;
  int v;
  int x;
  int y;

  for (y = 0; y < 8; y++)
  {
    for (u = 0; u < 8; u++)
    {
      int32_t sum = 0;
      for (x = 0; x < 8; x++)
      {
        sum += (int32_t)in[y][x] * (int32_t)cos_q14[u][x];
      }
      sum = (sum + 8192) >> 14;
      if (u == 0)
      {
        sum = (sum * 11585 + 8192) >> 14;
      }
      tmp[y][u] = sum;
    }
  }

  for (v = 0; v < 8; v++)
  {
    for (u = 0; u < 8; u++)
    {
      int32_t sum = 0;
      for (y = 0; y < 8; y++)
      {
        sum += tmp[y][u] * (int32_t)cos_q14[v][y];
      }
      sum = (sum + 8192) >> 14;
      if (v == 0)
      {
        sum = (sum * 11585 + 8192) >> 14;
      }
      sum >>= 2;
      if (sum > 32767)
      {
        sum = 32767;
      }
      if (sum < -32768)
      {
        sum = -32768;
      }
      out[v][u] = (int16_t)sum;
    }
  }
}

static int clamp8(int value)
{
  if (value < 0)
  {
    return 0;
  }
  if (value > 255)
  {
    return 255;
  }
  return value;
}

static void pix_ycbcr(uint16_t pix, int *y, int *cb, int *cr)
{
  int r = (int)((pix >> 11) & 0x1FU) << 3;
  int g = (int)((pix >> 5) & 0x3FU) << 2;
  int b = (int)(pix & 0x1FU) << 3;

  *y = (77 * r + 150 * g + 29 * b) >> 8;
  *cb = clamp8((((-43 * r) - (85 * g) + (128 * b)) >> 8) + 128);
  *cr = clamp8((((128 * r) - (107 * g) - (21 * b)) >> 8) + 128);
}

static void write_headers(uint16_t width, uint16_t height)
{
  static const uint8_t dht_dc[] = {
    0xFF, 0xC4, 0x00, 0x1F, 0x00,
    0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
  };
  uint8_t dqt[69];
  uint8_t sof[19];
  uint32_t i;

  put_u16(0xFFD8);
  put_bytes((const uint8_t *)"\xFF\xE0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00", 18);

  dqt[0] = 0xFF;
  dqt[1] = 0xDB;
  dqt[2] = 0x00;
  dqt[3] = 0x43;
  dqt[4] = 0x00;
  for (i = 0U; i < 64U; i++)
  {
    dqt[5U + i] = quant[zigzag[i]];
  }
  put_bytes(dqt, 69);

  /* Y 按 2x2 抽样，Cb/Cr 各一块。三个分量共用同一套量化表和霍夫曼表。 */
  sof[0] = 0xFF;
  sof[1] = 0xC0;
  sof[2] = 0x00;
  sof[3] = 17;
  sof[4] = 8;
  sof[5] = (uint8_t)(height >> 8);
  sof[6] = (uint8_t)height;
  sof[7] = (uint8_t)(width >> 8);
  sof[8] = (uint8_t)width;
  sof[9] = 3;
  sof[10] = 1;
  sof[11] = 0x22;
  sof[12] = 0;
  sof[13] = 2;
  sof[14] = 0x11;
  sof[15] = 0;
  sof[16] = 3;
  sof[17] = 0x11;
  sof[18] = 0;
  put_bytes(sof, 19);

  put_bytes(dht_dc, sizeof(dht_dc));
  put_byte(0xFF);
  put_byte(0xC4);
  put_u16(19U + (uint16_t)sizeof(ac_val));
  put_byte(0x10);
  put_bytes(ac_bits, 16);
  put_bytes(ac_val, sizeof(ac_val));

  put_bytes((const uint8_t *)"\xFF\xDA\x00\x0C\x03\x01\x00\x02\x00\x03\x00\x00\x3F\x00", 14);
}

static const uint16_t *enc_src;
static uint16_t enc_w;
static uint16_t enc_h;
static uint16_t enc_mx;
static uint16_t enc_my;
static uint8_t enc_part;
static int enc_dc_y;
static int enc_dc_cb;
static int enc_dc_cr;
static uint8_t *enc_out;
static uint8_t enc_on;
static uint8_t mcu_y[16 * 16];
static uint8_t mcu_cb[8 * 8];
static uint8_t mcu_cr[8 * 8];

static void load_mcu(void)
{
  uint16_t acc_cb[8 * 8];
  uint16_t acc_cr[8 * 8];
  int y;
  int x;

  memset(acc_cb, 0, sizeof(acc_cb));
  memset(acc_cr, 0, sizeof(acc_cr));
  for (y = 0; y < 16; y++)
  {
    const uint16_t *row = enc_src + (((uint32_t)enc_my + (uint32_t)y) * enc_w) + enc_mx;

    for (x = 0; x < 16; x++)
    {
      int yy;
      int cb;
      int cr;
      int cx = x >> 1;
      int cy = y >> 1;

      pix_ycbcr(row[x], &yy, &cb, &cr);
      mcu_y[(y * 16) + x] = (uint8_t)yy;
      acc_cb[(cy * 8) + cx] = (uint16_t)(acc_cb[(cy * 8) + cx] + (uint16_t)cb);
      acc_cr[(cy * 8) + cx] = (uint16_t)(acc_cr[(cy * 8) + cx] + (uint16_t)cr);
    }
  }
  for (x = 0; x < (8 * 8); x++)
  {
    mcu_cb[x] = (uint8_t)((acc_cb[x] + 2U) >> 2);
    mcu_cr[x] = (uint8_t)((acc_cr[x] + 2U) >> 2);
  }
}

static void fill_plane(const uint8_t *plane, int stride, int ox, int oy, int16_t samples[8][8])
{
  int y;
  int x;

  for (y = 0; y < 8; y++)
  {
    const uint8_t *row = plane + ((oy + y) * stride) + ox;

    for (x = 0; x < 8; x++)
    {
      samples[y][x] = (int16_t)row[x] - 128;
    }
  }
}

static int encode_one_block(void)
{
  static const uint8_t y_ox[4] = { 0U, 8U, 0U, 8U };
  static const uint8_t y_oy[4] = { 0U, 0U, 8U, 8U };
  int16_t samples[8][8];
  int16_t freq[8][8];
  int zz[64];
  int run = 0;
  int k;
  int *dc;

  if (enc_part == 0U)
  {
    load_mcu();
  }
  if (enc_part < 4U)
  {
    fill_plane(mcu_y, 16, (int)y_ox[enc_part], (int)y_oy[enc_part], samples);
    dc = &enc_dc_y;
  }
  else if (enc_part == 4U)
  {
    fill_plane(mcu_cb, 8, 0, 0, samples);
    dc = &enc_dc_cb;
  }
  else
  {
    fill_plane(mcu_cr, 8, 0, 0, samples);
    dc = &enc_dc_cr;
  }

  fdct(samples, freq);
  for (k = 0; k < 64; k++)
  {
    int idx = zigzag[k];
    int v = freq[idx / 8][idx % 8];
    int q = quant[idx];
    if (v >= 0)
    {
      zz[k] = (v + q / 2) / q;
    }
    else
    {
      zz[k] = -(((-v) + q / 2) / q);
    }
  }

  encode_coeff(zz[0] - *dc, &dc_code[magnitude(zz[0] - *dc)]);
  *dc = zz[0];
  for (k = 1; k < 64; k++)
  {
    if (zz[k] == 0)
    {
      run++;
      continue;
    }
    while (run > 15)
    {
      put_huff(&ac_code[0xF0]);
      run -= 16;
    }
    encode_coeff(zz[k], &ac_code[(run << 4) | magnitude(zz[k])]);
    run = 0;
  }
  if (run > 0)
  {
    put_huff(&ac_code[0x00]);
  }
  if (overflow != 0U)
  {
    return -1;
  }
  enc_part = (uint8_t)(enc_part + 1U);
  if (enc_part >= 6U)
  {
    enc_part = 0U;
    enc_mx = (uint16_t)(enc_mx + 16U);
    if (enc_mx >= enc_w)
    {
      enc_mx = 0U;
      enc_my = (uint16_t)(enc_my + 16U);
    }
  }
  return 0;
}

int JPEG_EncodeBegin(const uint16_t *src, uint16_t width, uint16_t height,
                      uint8_t *out, uint32_t out_max)
{
  if ((src == 0) || (out == 0) || ((width & 15U) != 0U) || ((height & 15U) != 0U) || (out_max < 64U))
  {
    return 0;
  }

  tables_init();
  out_p = out;
  out_end = out + out_max;
  bit_buf = 0U;
  bit_n = 0U;
  overflow = 0U;
  enc_src = src;
  enc_out = out;
  enc_w = width;
  enc_h = height;
  enc_mx = 0U;
  enc_my = 0U;
  enc_part = 0U;
  enc_dc_y = 0;
  enc_dc_cb = 0;
  enc_dc_cr = 0;
  enc_on = 1U;
  write_headers(width, height);
  if (overflow != 0U)
  {
    enc_on = 0U;
    return 0;
  }
  return 1;
}

int JPEG_EncodeSlice(uint16_t max_blocks)
{
  if (enc_on == 0U)
  {
    return -1;
  }
  while ((max_blocks > 0U) && (enc_my < enc_h))
  {
    if (encode_one_block() != 0)
    {
      enc_on = 0U;
      return -1;
    }
    max_blocks--;
  }
  if (enc_my < enc_h)
  {
    return 0;
  }
  flush_bits();
  put_u16(0xFFD9);
  enc_on = 0U;
  if (overflow != 0U)
  {
    return -1;
  }
  return 1;
}

uint32_t JPEG_EncodeLength(void)
{
  return (uint32_t)(out_p - enc_out);
}

uint32_t JPEG_EncodeRgb565(const uint16_t *src, uint16_t width, uint16_t height,
                           uint8_t *out, uint32_t out_max)
{
  if (JPEG_EncodeBegin(src, width, height, out, out_max) == 0)
  {
    return 0U;
  }
  for (;;)
  {
    int step = JPEG_EncodeSlice(1U);
    if (step < 0)
    {
      return 0U;
    }
    if (step > 0)
    {
      return JPEG_EncodeLength();
    }
  }
}
