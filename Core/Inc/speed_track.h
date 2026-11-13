#ifndef SPEED_TRACK_H
#define SPEED_TRACK_H

#include <stdint.h>
#include "dial_read.h"

/* 升针（×0.0001）转速跟踪。不认刻度盘、不跟梅花轮，不改动 dial_read 源文件。
 * 一圈 1 L（一格 0.1 L），流速 L/h = 10 * deg/s，按一段时间窗口计算。 */

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
  int32_t      last_ang;      /* previous leading-edge angle, mdeg */
  int32_t      last_phase;    /* unused (star wheel paused) */
  uint32_t     win_t0;        /* flow window start, TIM2 us */
  int32_t      win_accum;     /* unwrapped mdeg since win_t0 */
  uint32_t     mid_t;         /* mid-window snapshot time */
  int32_t      mid_accum;
  int32_t      dir_sign;      /* +1 angle increases, -1 decreases, 0 unknown */
  int32_t      hub_locked;    /* 针尖走够约 0.3 L 后拟合出圆心 */
} speed_state_t;

void SpeedTrack_Reset(speed_state_t *st);

/* 在实时画面上找升针（不走表盘算法）。成功返回 0。 */
int SpeedTrack_FindNeedle(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                          uint8_t *work, uint32_t work_bytes, speed_state_t *st);

/* 保留：旧接口仍编译，速度模式不再调用。 */
int SpeedTrack_TakeNeedle(const dial_report_t *rep, speed_state_t *st);
int SpeedTrack_FindStar(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                        uint8_t *work, uint32_t work_bytes, speed_state_t *st);

/* 按升针算出贴合裁窗（小边距，偶数对齐）。 */
void SpeedTrack_FinishLocate(speed_state_t *st, uint16_t frame_w, uint16_t frame_h);

/* 把定位坐标从全幅平移到裁窗坐标系。 */
void SpeedTrack_Shift(speed_state_t *st, int16_t dx, int16_t dy);

/* 在当前裁窗里重新找升针。若针太靠边，写出新裁窗在本图中的原点与尺寸并返回 1，
 * 需要主程序再裁一次；已经居中返回 0；没找到返回 -1。 */
int SpeedTrack_RefineCrop(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                          uint8_t *work, uint32_t work_bytes, speed_state_t *st,
                          int16_t *ox, int16_t *oy, uint16_t *nw, uint16_t *nh);

/* 用带时间戳的一帧更新转速。速度只用 stamp_us，不用 RTC。 */
int SpeedTrack_Tick(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                    uint32_t stamp_us, speed_state_t *st);

/* 在 LCD 上画黑色轨迹圆、白色前沿点和左上角读数。ox,oy 是裁窗在屏幕上的原点。 */
void SpeedTrack_Paint(uint16_t *fb, uint16_t fb_w, uint16_t fb_h,
                      int16_t ox, int16_t oy, const speed_state_t *st,
                      const dial_time_t *t);

#endif
