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

/* 可调识别参数，供截屏工具「参数表」经邮箱读写。下标与工具目录一致，不要改顺序。 */
extern volatile int g_search_mul;
extern volatile int g_search_min;
extern volatile int g_search_max;
extern volatile int g_b_rmin_div;
extern volatile int g_b_rmin_min;
extern volatile int g_b_rmin_max;
extern volatile int g_tip_gap;
extern volatile int g_red_min;
extern volatile int g_red_over_g;
extern volatile int g_red_over_b;
extern volatile int g_red_g_max;
extern volatile int g_red_b_max;
extern volatile int g_min_area;
extern volatile int g_pale_min;
extern volatile int g_gold_max;
extern volatile int g_track_near_min;
extern volatile int g_ang_alpha_num;
extern volatile int g_ang_jump_milli;
extern volatile int g_ang_jump_div;
extern volatile int g_dial_rmul_min;
extern volatile int g_dial_rmul_max;
extern volatile int g_dial_rmin;
extern volatile int g_dial_rmax;
extern volatile int g_dial_ink_luma;
extern volatile int g_dial_ink_chroma;
extern volatile int g_tick_n;
extern volatile int g_tick_n_left;
extern volatile int g_tick_len_major;
extern volatile int g_tick_len_minor;
extern volatile uint16_t g_color_ab;
extern volatile uint16_t g_color_search;
extern volatile uint16_t g_color_dial;
extern volatile uint16_t g_color_dial_est;
extern volatile uint16_t g_color_text;
extern volatile uint16_t g_color_text_bg;

#endif
