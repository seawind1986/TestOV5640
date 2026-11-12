#ifndef METER_LOCATE_H
#define METER_LOCATE_H

#include <stdint.h>

/* 圆 A：指针圆盘最大内切圆（轴心）。
 * 圆 B：尖角里被两边夹住、且与圆盘分开的内切圆，取离 A 最远的一个。
 * 方向：两圆心连线。 */

#define METER_MAX_TARGETS  8U
#define METER_MAX_POINTERS 4U

typedef struct
{
  uint16_t cx;
  uint16_t cy;
  uint16_t r;
  uint16_t bx;
  uint16_t by;
  uint16_t br;
  uint16_t tx;
  uint16_t ty;
  uint8_t locked;
  uint8_t have_ang;
  int32_t ang_milli;
} meter_pointer_t;

int MeterLocate_Find(const uint16_t *rgb, uint16_t w, uint16_t h,
                     uint32_t work, uint32_t work_bytes,
                     meter_pointer_t *out, uint8_t max_out);

int MeterLocate_Track(const uint16_t *rgb, uint16_t w, uint16_t h,
                      uint32_t work, uint32_t work_bytes,
                      meter_pointer_t *ptrs, uint8_t n);

void MeterLocate_DrawPointers(volatile uint16_t *fb, uint16_t w, uint16_t h,
                              const meter_pointer_t *ptrs, uint8_t n);

#endif
