#ifndef JPEG_ENC_H
#define JPEG_ENC_H

#include <stdint.h>

/* 把 RGB565 画面编成 4:2:0 彩色 JPEG。宽和高都要是 16 的倍数。返回字节数，缓冲不够时返回 0。 */
uint32_t JPEG_EncodeRgb565(const uint16_t *src, uint16_t width, uint16_t height,
                           uint8_t *out, uint32_t out_max);
/* 1 开始成功。Slice 最多编 max_blocks 个 8x8 块。返回 1 表示整帧结束，0 表示还没完，-1 表示失败。 */
int JPEG_EncodeBegin(const uint16_t *src, uint16_t width, uint16_t height,
                      uint8_t *out, uint32_t out_max);
int JPEG_EncodeSlice(uint16_t max_blocks);
uint32_t JPEG_EncodeLength(void);

#endif
