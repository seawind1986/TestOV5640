#ifndef STAR_TRACK_H
#define STAR_TRACK_H

#include <stdint.h>
#include "dial_read.h"

/* 梅花轮齿频模式（第三个功能）。与四针读数、升针测速互不影响。
 * 全幅先找梅花轮（暗色、带等分缺口的圆盘），裁小窗后在行中断里按圆环取亮度，
 * 每帧用 N 次谐波相位求齿相位，按 DCMI 帧完成时刻（TIM2 微秒）展开，算 齿/s。 */

#define STAR_BINS      72U     /* 圆环角度分 72 格，每格 5° */
#define STAR_TAB_MAX   9000U   /* 圆环采样点上限 */
#define STAR_ROW_MAX   600U
#define STAR_RING      64U     /* 行中断 -> 主循环 的每帧相位队列 */
#define STAR_HIST      400U    /* 约 8 s @50 fps 的 (时刻, 累计齿) 历史 */
#define STAR_MEAS_HIST 10U

typedef struct
{
  uint8_t  found;      /* 最近一次定位成功 */
  uint8_t  n_teeth;    /* 缺口（齿）数 */
  uint8_t  armed;      /* 行中断采样表已建好 */
  uint8_t  ncand;      /* 最近一次定位的候选数 */
  int16_t  cx;         /* 当前摄像头画面坐标（全幅=LCD 坐标；裁窗后=小窗坐标） */
  int16_t  cy;
  int16_t  r_out;      /* 齿顶半径 */
  int16_t  r_in;       /* 缺口底半径 */
  int16_t  ann_r0;     /* 采样圆环内外半径 */
  int16_t  ann_r1;
  int16_t  ann_pk;     /* 径向扫描 N 次谐波最强的半径 */
  int16_t  full_cx;    /* 全幅定位结果 */
  int16_t  full_cy;
  int16_t  full_r;
  uint16_t score_e3;   /* N 次谐波归一化幅度 ×1000 */
  uint16_t trans;      /* 采样圆上暗->亮跳变次数 */
  uint16_t thr;        /* 暗阈值 */
  uint16_t med;        /* 画面亮度中位数 */
  uint16_t tab_n;      /* 圆环采样点数 */
} star_info_t;

extern star_info_t g_star;
#define STAR_RAD_MAX 200U
extern uint16_t g_star_rad_amp[STAR_RAD_MAX];
extern uint16_t g_star_rad_n;
extern volatile uint32_t g_star_isr_frames;   /* 行中断统计到的完整帧 */
extern volatile uint32_t g_star_isr_bad;      /* 行数不对、未建表的帧 */
extern volatile uint16_t g_star_isr_lines;    /* 最近一帧行数 */
extern double   g_star_accum;                 /* 展开后的累计齿数（有符号） */
extern double   g_star_wheel_deg;             /* 展开后的轮转角（°，有符号） */
extern int32_t  g_star_ang_mdeg;              /* 白线当前角 0~360° ×1000 */
extern int32_t  g_star_dps_e3;                /* 约 2 s 窗口 °/s ×1000 */
extern uint32_t g_star_lap_us;                /* 最近一整圈用时 */
extern uint8_t  g_star_mw_laps;
extern uint8_t  g_star_mw_target;
extern uint32_t g_star_meas_dt_us[STAR_MEAS_HIST];
extern int32_t  g_star_meas_ang_mdeg[STAR_MEAS_HIST];
extern int32_t  g_star_meas_dps_e3[STAR_MEAS_HIST];
extern uint8_t  g_star_meas_laps[STAR_MEAS_HIST];
extern int32_t  g_star_tps_e3;                /* 最近约 2 s 齿/s ×1000（绝对值） */
extern int32_t  g_star_fps_e1;                /* 实测采样帧率 ×10 */
extern uint32_t g_star_frame_dt_us;           /* 液晶相邻两帧时间差 */
extern uint16_t g_star_seq;
extern uint8_t  g_star_meas_n;
extern uint32_t g_star_gap_n;                 /* 帧间隔过大、重新起算的次数 */
extern uint32_t g_star_lowq_n;                /* 谐波幅度过低被丢弃的帧数 */

void StarTrack_Reset(void);
void StarTrack_NewTest(void); /* 用户重新开始：含序号/历史；停采集后调用 */

/* 在 rgb（stride 像素一行）里找梅花轮。ds=降采样倍数（全幅 2，小窗 1）。
 * skip_x1/skip_y1：左上角 HUD 区域不参与。成功返回 0，结果在 g_star（rgb 坐标）。 */
int StarTrack_Locate(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h, uint8_t ds,
                     uint8_t *work, uint32_t work_bytes, uint16_t skip_x1, uint16_t skip_y1);

/* 按 g_star 当前圆心/半径建行中断采样表并开始测量。cap_w/cap_h 为摄像头输出尺寸。 */
int StarTrack_Arm(uint16_t cap_w, uint16_t cap_h);
void StarTrack_Disarm(void);

/* 行中断里每收完一行调用；src 是刚收完的行缓冲。 */
void StarTrack_IsrLine(const uint32_t *src);
/* 帧完成中断里调用。 */
void StarTrack_IsrFrame(uint32_t stamp_us, uint16_t lines_expect);

/* 主循环：消化每帧相位，更新累计齿数、齿/s、列表。返回最近 1 s 内有效帧数是否不足（1=丢失）。 */
int StarTrack_Tick(uint32_t now_us);

/* 左上角 HUD 和实时画面上的圆环/相位标记。ox,oy 是摄像头画面在屏幕上的原点。 */
void StarTrack_Paint(uint16_t *fb, uint16_t fb_w, uint16_t fb_h, int16_t ox, int16_t oy,
                     uint32_t stamp_us, const dial_time_t *t, uint8_t phase);

/* HUD 在全幅找轮时占的区域（右下角坐标），定位时跳过。 */
void StarTrack_HudRect(uint16_t *x1, uint16_t *y1);

#endif
