/*
 * dial_read.h  --  water-meter pointer-dial reader (STM32H743 port of dial_read.py)
 *
 * Integer / fixed-point only: add, sub, mul, shifts, a few small divisions,
 * lookup tables (cos per degree, atan 0..1), integer sqrt by iteration.
 * No float, no malloc: the caller hands in one work area (SDRAM).
 *
 * Pipeline (same order as the PC verification):
 *   1 seam restore (frame wrapped horizontally, auto-detected column)
 *   2 perspective rectification (fixed 3x3 H, inverse map + bilinear)
 *   3 red pointer blobs, hub = max chamfer distance (reject r<15 / clipped)
 *   4 rim circle from one real arc segment + adaptive tracing (Kasa fit)
 *   5 tick-length profile, 36 deg phase fit, 20 slots, completion, snap
 *   6 pointer direction from the whole-pointer axis (2nd moments)
 *   7 reading interpolated between the printed main ticks
 *   8 dial count / scale validity  -> SUSPECT
 *   9 digit place from a calibration table (never by left/right)
 *  10 boundary rules  -> carry / borrow / SUSPECT
 *  11 combined reading 0.xxxxxx
 *  12 overlay: timestamp + reading (red on a solid white panel). On the live
 *     frame: green recognised rim and a blue line from centre to tip.
 *     Analysis copy also keeps per-dial digit at 6 o'clock and green ticks.
 *     SUSPECT: raw (restored+rectified) frame with timestamp only, saved via hook.
 */
#ifndef DIAL_READ_H
#define DIAL_READ_H

#include <stdint.h>

/* ---------------- configuration ---------------- */
#ifndef DIAL_COUNT_CFG
#define DIAL_COUNT_CFG      4U      /* configured number of pointer dials           */
#endif
#define DIAL_MAX_FOUND      8U      /* pointer candidates examined                   */
#define DIAL_MAX_TICK      20U      /* 10 majors, or 20 if x0.0001 has halves        */
#define DIAL_MAX_BLOBS      16U

/* seam restore: -1 = auto detect, 0 = off, >0 = fixed column (260 on the 17:29 frame) */
#ifndef DIAL_SEAM_COL_CFG
#define DIAL_SEAM_COL_CFG   (-1)
#endif
/* 0 = rotate each row (what the PC verification did)
 * 1 = linear stream shift (left block belongs to the PREVIOUS row; matches a DMA/line phase slip)
 * 2 = auto: pick the mode with the better wrap continuity */
#ifndef DIAL_SEAM_MODE_CFG
#define DIAL_SEAM_MODE_CFG  0
#endif
/* 1 = apply the fitted rectification H, 0 = identity (correction is < 5 px) */
#ifndef DIAL_RECTIFY_ON
#define DIAL_RECTIFY_ON     1
#endif

/* ---------------- types ---------------- */
typedef struct
{
  uint16_t year;
  uint8_t  mon, day, hour, min, sec;
  uint16_t msec;                 /* 0..999, from RTC subseconds          */
} dial_time_t;

typedef struct
{
  int16_t x, y;      /* dial centre in the rectified frame (px) */
  int8_t  place;     /* 0 = highest digit (x0.1), 1 = x0.01, ...  */
} dial_cal_t;

typedef struct
{
  int32_t cx_q8, cy_q8, r_q8;     /* rim circle, 1/256 px                 */
  int32_t tip_x_q8, tip_y_q8;     /* far end of pointer axis              */
  int32_t pang_mdeg;              /* pointer angle, milli-degree, image   */
  int32_t value_e4;               /* dial value * 10000 (0..99999)        */
  int16_t hub_x, hub_y;           /* hub (max inscribed) position         */
  uint8_t hub_r;                  /* hub radius px                        */
  uint8_t n_vis;                  /* printed ticks seen (of 20)           */
  uint8_t arc_cov;                /* traced rim coverage, 10-deg sectors  */
  int8_t  place;                  /* from calibration, -1 = no match      */
  uint8_t digit;                  /* integer digit after boundary rules   */
} dial_info_t;

/* suspect reasons */
#define DIAL_SUS_COUNT   0x01U   /* valid dials != configured            */
#define DIAL_SUS_SCALE   0x02U   /* pointer found, scale not recognized  */
#define DIAL_SUS_CAL     0x04U   /* dial not in order calibration        */
#define DIAL_SUS_AMBIG   0x08U   /* boundary rule: next dial in 3..7     */

#define DIAL_OK          0
#define DIAL_SUSPECT     1
#define DIAL_ERR_WORK   (-1)
#define DIAL_ERR_ARG    (-2)

typedef struct
{
  int32_t tick_mdeg[DIAL_MAX_TICK]; /* recognised tick angles, milli-degree */
  uint8_t keep[DIAL_MAX_TICK];      /* 1 = draw this tick                  */
  uint8_t n_tick;
  uint8_t allow_half;
  uint8_t major_base;
} dial_ovl_t;

typedef struct
{
  int      status;               /* DIAL_OK / DIAL_SUSPECT / error         */
  uint32_t suspect_flags;
  uint8_t  ambig_places;         /* bit i: place i was ambiguous           */
  int16_t  seam_col;             /* seam column used (0 = none)            */
  uint8_t  seam_mode;
  int16_t  level_gain_q8;         /* lighting normalisation used (256 = 1.0) */
  int16_t  level_off;
  uint8_t  n_dials;              /* valid dials                            */
  uint8_t  n_scale_bad;
  dial_info_t dial[DIAL_MAX_FOUND];
  uint32_t reading_e6;           /* 714889 -> 0.714889 m3, shown as 714.889 L */
  dial_time_t ts;
  char     line1[56];            /* "2026-11-11 11:28:50 [007]    5234 ms  10" */
  char     line2[24];            /* "714.889 L"                             */
  char     line3[16];            /* "5234 ms" interval, empty if first     */
  uint32_t interval_ms;          /* this stamp minus previous, 0 if first  */
  char     ts_name[20];          /* "20261110_172939"                      */
  uint8_t  track_mode;           /* 0 = full keyframe, 1 = fast track      */
  uint16_t track_fb;             /* 0=fast ok; else fallback code after ms */
  dial_ovl_t ovl[DIAL_MAX_FOUND]; /* live tick overlay; not used for reading */
} dial_report_t;

/* work area needed for a w x h RGB565 frame (about 4.6 MB for 1024x600) */
uint32_t DialRead_WorkBytes(uint16_t w, uint16_t h);

/* raw: captured RGB565 frame. out: result frame (may be == raw).
 * work: 8-byte aligned scratch (SDRAM), at least DialRead_WorkBytes(). */
int DialRead_Run(const uint16_t *raw, uint16_t *out, uint16_t w, uint16_t h,
                 uint8_t *work, uint32_t work_bytes, dial_report_t *rep);

/* digit-place calibration (positions -> place). Default table is built in. */
void DialRead_SetOrderCal(const dial_cal_t *cal, uint8_t n);
const dial_cal_t *DialRead_GetOrderCal(uint8_t *n);

/* Drop the cached 4-dial geometry so the next Run is a full keyframe. */
void DialRead_ResetTrack(void);

/* J-Link diagnostics: read g_dial_dbg (magic 0x44424731 = 'DBG1').
 * Reason codes: 0 ok, 1 area<800, 2 chamfer window too big, 3 hub dt<45
 * (red mark), 4 hub clipped at frame edge, 5 arc/R fail, 6 pointer tip
 * fail, 7 too few ticks / centre off-frame, 8 no blob for this place.
 * tip_sub (only with reason 6, or 0 if tip used farthest fallback):
 * 1 few red pixels, 2 hub r too small, 3 too few mid-arc rings,
 * 4 axis zero, 5 red run shorter than 2*hub, 6 farthest-pixel fail.
 * last_* keeps the most recent frame that missed a dial (bit0=x0.1). */
#define DIAL_DBG_MAGIC   0x44424731U
#define DIAL_DBG_NCAND   16U
#define DIAL_FAIL_OK         0U
#define DIAL_FAIL_AREA       1U
#define DIAL_FAIL_WIN        2U
#define DIAL_FAIL_HUB_DT     3U
#define DIAL_FAIL_HUB_CLIP   4U
#define DIAL_FAIL_ARC        5U
#define DIAL_FAIL_TIP        6U
#define DIAL_FAIL_VIS        7U
#define DIAL_FAIL_NO_BLOB    8U

typedef struct
{
  int16_t  cx, cy;       /* blob centroid, px                          */
  int16_t  hx, hy;       /* hub, or -1 if not computed                 */
  uint16_t area;
  uint16_t dtm;          /* chamfer * 3, 0 if not computed             */
  int8_t   place;        /* 0=x0.1 .. 3=x0.0001, -1 unknown            */
  uint8_t  reason;       /* DIAL_FAIL_* ; 0 = accepted                 */
  uint8_t  tip_sub;      /* pointer_tip detail, see above              */
  uint8_t  n_vis;
} dial_dbg_cand_t;       /* 16 bytes                                   */

typedef struct
{
  uint32_t magic;            /* 0  0x44424731                          */
  uint32_t seq;              /* 4  increments each DialRead_Run        */
  uint32_t fail_seq;         /* 8  seq of last incomplete frame        */
  uint32_t suspect_flags;    /* 12 this frame                          */
  int16_t  seam_col;         /* 16                                     */
  int16_t  last_seam_col;    /* 18                                     */
  uint16_t track_fb;         /* 20 HUD status code                     */
  uint8_t  seam_mode;        /* 22                                     */
  uint8_t  seam_reused;      /* 23 1 = reused last good seam           */
  uint8_t  n_blobs;          /* 24                                     */
  uint8_t  n_cand;           /* 25 entries in cand[] this frame        */
  uint8_t  n_dials;          /* 26 accepted                            */
  uint8_t  n_scale_bad;      /* 27                                     */
  uint8_t  status;           /* 28 DIAL_OK / DIAL_SUSPECT              */
  uint8_t  place_mask;       /* 29 bit i set if place i accepted       */
  uint8_t  miss_place;       /* 30 bits of missing places              */
  uint8_t  cnt_area;         /* 31 this-frame skip counts              */
  uint8_t  cnt_win;          /* 32                                     */
  uint8_t  cnt_hub_dt;       /* 33                                     */
  uint8_t  cnt_hub_clip;     /* 34                                     */
  uint8_t  cnt_arc;          /* 35                                     */
  uint8_t  cnt_tip;          /* 36                                     */
  uint8_t  cnt_vis;          /* 37                                     */
  uint8_t  last_n_dials;     /* 38 snapshot when miss_place != 0       */
  uint8_t  last_miss_place;  /* 39                                     */
  uint8_t  last_n_cand;      /* 40                                     */
  uint8_t  last_seam_reused; /* 41                                     */
  uint8_t  _pad0, _pad1;     /* 42 keep 16-bit alignment               */
  uint8_t  last_reason[4];   /* 44 reason for place 0..3 last miss     */
  uint8_t  last_tip_sub[4];  /* 48                                     */
  int16_t  last_cx[4];       /* 52                                     */
  int16_t  last_cy[4];       /* 60                                     */
  int16_t  last_hx[4];       /* 68                                     */
  int16_t  last_hy[4];       /* 76                                     */
  dial_dbg_cand_t cand[DIAL_DBG_NCAND]; /* 84, 16*16 = 256; total 340  */
} dial_dbg_t;

extern volatile dial_dbg_t g_dial_dbg;

/* speed-based auto ordering (water flowing): lower digit turns ~10x faster.
 * Feed one report per frame (status may be SUSPECT for lack of calibration),
 * then Finish() writes the table.  Returns 0 ok, <0 error. */
void DialOrder_Reset(void);
int  DialOrder_Feed(const dial_report_t *rep, uint32_t t_ms);
int  DialOrder_Finish(dial_cal_t *cal_out, uint8_t *n_out);

/* Paint timestamp, reading, rims, ticks (extended past the rim) and pointer lines. */
void DialRead_PaintHud(uint16_t *fb, uint16_t w, uint16_t h, const dial_report_t *rep);

/* hooks: weak defaults in dial_read.c, override in the board port */
void dial_yield(void);
void dial_get_time(dial_time_t *t);
int  dial_save_suspect_frame(const uint16_t *frame, uint16_t w, uint16_t h, const char *name);

#endif /* DIAL_READ_H */
