#ifndef SPEED_TRACK_H
#define SPEED_TRACK_H

#include <stdint.h>
#include "dial_read.h"

/* 升针（×0.0001）和梅花轮的转速跟踪。
 * 与四指针读体积模式分开：只借用其定位结果，不改动 dial_read 源文件。 */

typedef struct
{
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint16_t h;
} speed_rect_t;

typedef struct
{
  uint8_t  found;
  uint8_t  n_teeth;
  int16_t  cx;
  int16_t  cy;
  int16_t  r;
  int16_t  hub_r;
  int16_t  tip_x;
  int16_t  tip_y;
  int32_t  ang_mdeg;
  int32_t  phase_mdeg;
} speed_obj_t;

typedef struct
{
  uint8_t      loc_ok;
  uint8_t      track_ok;
  uint8_t      have_speed;
  uint8_t      have_prev;
  speed_obj_t  needle;
  speed_obj_t  star;
  speed_rect_t crop;
  uint32_t     stamp_us;
  uint32_t     dt_us;
  int32_t      flow_lh_e2;    /* L/h * 100 */
  int32_t      tps_e2;        /* 齿/s * 100 */
  int32_t      mlp_e3;        /* ml/p * 1000 */
  int32_t      flow_filt_e2;
  int32_t      tps_filt_e2;
  int32_t      last_ang;      /* previous raw needle angle, mdeg */
  int32_t      last_phase;    /* previous raw tooth phase, mdeg */
} speed_state_t;

void SpeedTrack_Reset(speed_state_t *st);

/* 从四指针报告里取出 place 3（升针），并把坐标映回实时画面。成功返回 0。 */
int SpeedTrack_TakeNeedle(const dial_report_t *rep, speed_state_t *st);

/* 在升针附近找梅花轮（不走表盘算法）。成功返回 0。 */
int SpeedTrack_FindStar(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                        uint8_t *work, uint32_t work_bytes, speed_state_t *st);

/* 按升针和梅花轮算出裁窗（含边距，偶数对齐）。 */
void SpeedTrack_FinishLocate(speed_state_t *st, uint16_t frame_w, uint16_t frame_h);

/* 把定位坐标从全幅平移到裁窗坐标系。 */
void SpeedTrack_Shift(speed_state_t *st, int16_t dx, int16_t dy);

/* 用带时间戳的一帧更新转速。速度只用 stamp_us，不用 RTC。 */
int SpeedTrack_Tick(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                    uint32_t stamp_us, speed_state_t *st);

/* 在 LCD 上画圆、针线和左上角读数。ox,oy 是裁窗在屏幕上的原点。 */
void SpeedTrack_Paint(uint16_t *fb, uint16_t fb_w, uint16_t fb_h,
                      int16_t ox, int16_t oy, const speed_state_t *st,
                      const dial_time_t *t);

#endif
