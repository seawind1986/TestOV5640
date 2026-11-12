/*
 * dial_read.c  --  water-meter pointer-dial reader, STM32H743 port of the
 * PC verification (dial_read.py + rectify.py + dial_mask_0001.py).
 *
 * Integer only. Angles in milli-degrees (mdeg), coordinates in 1/256 px (q8).
 * All large buffers live in the caller's work area (SDRAM).
 */
#include "dial_read.h"
#include "dial_font.h"
#include <string.h>
#include <limits.h>
#ifdef DIAL_HOST_DEBUG
#include <stdio.h>
#endif

#if defined(__GNUC__) || defined(__clang__) || defined(__ARMCC_VERSION)
#define DIAL_WEAK __attribute__((weak))
#else
#define DIAL_WEAK
#endif

extern const int32_t dial_cos_q16[361];
extern const int32_t dial_atan_mdeg[257];

/* ---------------- parameters (same numbers as dial_read.py) ---------------- */
#define RED_RMIN        95
#define RED_DG          16
#define RED_DB          16
#define RED_GMAX        200
#define RED_BMAX        190
#define BLOB_MIN_AREA   400U
#define CAND_MIN_AREA   800U
#define HUB_WIN         120     /* hub test window half size               */
#define HUB_MIN_DT      45      /* hub radius 15 px * 3 (3-4 chamfer)      */
#define RAD_PAD         170     /* dial crop half size                     */
#define ARC_T_STRONG    120
#define ARC_WHITE       185
#define ARC_RMIN        15
#define ARC_RMAX        125
#define ARC_SEED_MIN    40
#define ARC_SEED_RMS2_X100 144  /* rms <= 1.2 px                            */
#define ARC_NEAR        35
#define ARC_CONTRAST    45
#define ARC_WHITE_GAP   25
#define ARC_MISS        2
#define DARK_L          (ARC_WHITE - 30)   /* 155: dark / near-dark         */
#define N_TICK          20
#define STEP_MDEG       18000
#define SNAP_MAX_MDEG   7000
#define ISOLATE_MDEG    2500
#define MIN_VIS_TICKS   5
#define MAX_ARC_PTS     1100
#define R_MIN_Q8        (30 * 256)
#define R_MAX_Q8        (160 * 256)
#define MAX_LABELS      65535U
#define DT_INF          60000U

#define TXT_R 230
#define TXT_G 30
#define TXT_B 30

/* inverse rectification (rectified -> captured), from rectify.py fit on the
 * 17:29:39 frame.  h11..h23 in Q20, h31/h32 in Q32, h33 = 1. */
#define HI11  1071152
#define HI12  5069
#define HI13  (-4317748)
#define HI21  5544
#define HI22  1071955
#define HI23  (-5061496)
#define HI31  75697
#define HI32  86945

/* lighting normalisation for the analysis copy (display copy stays raw):
 * 0 = off (exactly the PC verification), 1 = gain so the 90th luma percentile
 * maps to the reference frame's white, 2 = two-point (10th and 90th percentile). */
#ifndef DIAL_AUTO_LEVEL
#define DIAL_AUTO_LEVEL     0
#endif
#define LEVEL_REF_LO        119     /* 10th percentile luma of the 17:29:39 frame */
#define LEVEL_REF_HI        227     /* 90th percentile                            */
static uint8_t s_lvl[256];
static int32_t s_gain_q8 = 256, s_off = 0;

/* default digit-place calibration (rectified frame, px) */
static dial_cal_t s_cal[DIAL_MAX_FOUND] = {
  {748, 132, 0}, {688, 345, 1}, {501, 456, 2}, {286, 416, 3}
};
static uint8_t s_ncal = 4U;
static uint16_t s_read_seq;   /* 000..999, increments once per recognition */


/* lighting thresholds, filled from the current white face (not a fixed table) */
static int s_arc_t_strong = ARC_T_STRONG;
static int s_arc_white    = ARC_WHITE;
static int s_arc_contrast = ARC_CONTRAST;
static int s_arc_white_gap = ARC_WHITE_GAP;
static int s_dark_l       = (ARC_WHITE - 30);
static int s_contrast_floor = 20;
static int s_face_w = 220, s_face_d = 100;
static const uint16_t *s_rgb;   /* rectified RGB565, for chroma / face estimate */
static int8_t s_blob_place[DIAL_MAX_BLOBS];
static void px8(uint16_t p, int *r, int *g, int *b);
static uint16_t s_w, s_h;
static int32_t s_x0, s_y0, s_x1, s_y1;

static void apply_face_thresholds(int W, int D)
{
  int span = W - D;
  int arc_white, arc_strong, t;
  if (span < 40) { span = 40; }
  t = span * 20 / 100; if (t < 16) { t = 16; }
  arc_white = W - t;
  t = span * 40 / 100; if (t < 30) { t = 30; }
  arc_strong = W - t;
  if (arc_strong > arc_white - 15) { arc_strong = arc_white - 15; }
  s_arc_white = arc_white;
  s_arc_t_strong = arc_strong;
  t = span * 30 / 100; if (t < 18) { t = 18; }
  s_arc_contrast = t;
  t = span * 18 / 100; if (t < 12) { t = 12; }
  s_arc_white_gap = t;
  s_dark_l = (s_arc_white + s_arc_t_strong) / 2;
  t = span / 6; if (t < 14) { t = 14; }
  s_contrast_floor = t;
  s_face_w = W; s_face_d = D;
}

static void estimate_face_ink(int cx, int cy, int *pW, int *pD)
{
  uint32_t hist[256], n = 0U, acc, i;
  int x, y, W = -1, D = -1, minc;
  for (i = 0U; i < 256U; i++) { hist[i] = 0U; }
  if (s_rgb == 0) { *pW = s_face_w; *pD = s_face_d; return; }
  for (y = (int)s_y0; y < (int)s_y1; y++)
  {
    for (x = (int)s_x0; x < (int)s_x1; x++)
    {
      int dx = x - cx, dy = y - cy, d2 = dx * dx + dy * dy;
      int r, g, b, ch, mn, L;
      if ((d2 < 18 * 18) || (d2 > 80 * 80)) { continue; }
      px8(s_rgb[(uint32_t)y * s_w + (uint32_t)x], &r, &g, &b);
      ch = r; if (g > ch) { ch = g; } if (b > ch) { ch = b; }
      mn = r; if (g < mn) { mn = g; } if (b < mn) { mn = b; }
      ch -= mn;
      if (ch > 55) { continue; }
      L = (r + 2 * g + b) >> 2;
      hist[L]++; n++;
    }
  }
  minc = (int)(n / 200U); if (minc < 50) { minc = 50; }
  for (i = 1U; i < 255U; i++)
  {
    if ((hist[i] >= (uint32_t)minc) && (hist[i] >= hist[i - 1U]) && (hist[i] >= hist[i + 1U]))
    {
      if ((int)i > W) { W = (int)i; }
    }
  }
  if (W < 0)
  {
    acc = 0U;
    for (i = 0U; i < 256U; i++) { acc += hist[i]; if (acc * 10U >= 8U * n) { W = (int)i; break; } }
    if (W < 0) { W = 220; }
  }
  for (i = 1U; i < 255U; i++)
  {
    if ((hist[i] >= (uint32_t)minc) && (hist[i] >= hist[i - 1U]) && (hist[i] >= hist[i + 1U])
        && ((int)i <= W - 40))
    {
      if ((D < 0) || ((int)i < D)) { D = (int)i; }
    }
  }
  if (W < 80) { W = 80; }
  if (W > 250) { W = 250; }
  if ((D < 20) || (D > W - 30))
  {
    acc = 0U; D = 20;
    for (i = 0U; i < 256U; i++) { acc += hist[i]; if (acc * 100U >= 15U * n) { D = (int)i; break; } }
    if (D > W - 30) { D = W - 30; }
    if (D < 20) { D = 20; }
  }
  *pW = W; *pD = D;
}

/* ---------------- frame / work state ---------------- */
/* s_w, s_h, s_x0..s_y1 declared with the face-threshold state */
static const uint8_t *s_L;      /* luma (r+2g+b)>>2           */
static const uint8_t *s_red;    /* red mask                   */
static uint16_t *s_lab;         /* blob labels (root id)      */
static uint16_t *s_dt;          /* chamfer distance scratch   */

/* RGB565 -> 8 bit exactly as the LCD dump tool: floor(v*255/31), floor(v*255/63) */
static uint8_t s_r5to8[32];
static uint8_t s_g6to8[64];

/* ---------------- small math ---------------- */
static int32_t iabs32(int32_t v) { return (v < 0) ? -v : v; }
static int64_t iabs64(int64_t v) { return (v < 0) ? -v : v; }

static uint32_t isqrt64(uint64_t v)
{
  uint64_t res = 0U;
  uint64_t bit = (uint64_t)1U << 62;
  while (bit > v) { bit >>= 2; }
  while (bit != 0U)
  {
    if (v >= res + bit) { v -= res + bit; res = (res >> 1) + bit; }
    else { res >>= 1; }
    bit >>= 2;
  }
  return (uint32_t)res;
}

static int32_t wrap_mdeg(int32_t a)
{
  a %= 360000;
  if (a < 0) { a += 360000; }
  return a;
}

/* (a-b+180)%360-180 */
static int32_t angdiff_mdeg(int32_t a, int32_t b)
{
  return wrap_mdeg(a - b + 180000) - 180000;
}

static int32_t cos_deg(int32_t d) { d %= 360; if (d < 0) { d += 360; } return dial_cos_q16[d]; }
static int32_t sin_deg(int32_t d) { return cos_deg(d + 270); }

/* cos of an arbitrary angle: per-degree table + linear interpolation (Q16) */
static int32_t cos_mdeg(int32_t a)
{
  int32_t i, f;
  a = wrap_mdeg(a);
  i = a / 1000;
  f = a - i * 1000;
  return dial_cos_q16[i] + (int32_t)(((int64_t)(dial_cos_q16[i + 1] - dial_cos_q16[i]) * f) / 1000);
}
static int32_t sin_mdeg(int32_t a) { return cos_mdeg(a + 270000); }

/* atan2 -> [0, 360000) mdeg, image convention (y down) like numpy */
static int32_t atan2_mdeg(int64_t y, int64_t x)
{
  uint64_t ax = (uint64_t)iabs64(x), ay = (uint64_t)iabs64(y);
  uint64_t mx;
  uint32_t t, idx, fr;
  int32_t a;
  if ((ax == 0U) && (ay == 0U)) { return 0; }
  mx = (ax > ay) ? ax : ay;
  while (mx >= 32768U) { ax >>= 1; ay >>= 1; mx >>= 1; }
  if (ax >= ay)
  {
    t = (uint32_t)((ay << 16) / ax);
  }
  else
  {
    t = (uint32_t)((ax << 16) / ay);
  }
  idx = t >> 8;
  fr = t & 255U;
  if (idx >= 256U) { a = dial_atan_mdeg[256]; }
  else { a = dial_atan_mdeg[idx] + (((dial_atan_mdeg[idx + 1] - dial_atan_mdeg[idx]) * (int32_t)fr) >> 8); }
  if (ax < ay) { a = 90000 - a; }
  if (x < 0) { a = 180000 - a; }
  if (y < 0) { a = 360000 - a; }
  if (a >= 360000) { a -= 360000; }
  return a;
}

/* num * 2^e / den, keeping inside int64 */
static int64_t muldiv_p2(int64_t num, int64_t den, int e)
{
  if (den == 0) { return 0; }
  while ((e > 0) && (iabs64(num) < ((int64_t)1 << 61))) { num *= 2; e--; }
  while (e > 0) { den /= 2; e--; if (den == 0) { return 0; } }
  while ((e < 0) && (iabs64(den) < ((int64_t)1 << 61))) { den *= 2; e++; }
  while (e < 0) { num /= 2; e++; }
  return num / den;
}

static int bits_needed(int64_t v)
{
  int n = 0;
  uint64_t u = (uint64_t)iabs64(v);
  while (u != 0U) { u >>= 1; n++; }
  return n;
}

/* ---------------- samplers ---------------- */
static int round_q8(int32_t v) { return (v + 128) >> 8; }

/* luma at a q8 position inside the current crop, -1 = outside (None) */
static int L_at(int32_t x_q8, int32_t y_q8)
{
  int32_t xi = round_q8(x_q8), yi = round_q8(y_q8);
  if ((xi < s_x0) || (yi < s_y0) || (xi >= s_x1) || (yi >= s_y1)) { return -1; }
  return s_L[(uint32_t)yi * s_w + (uint32_t)xi];
}

static int red_at(int32_t x_q8, int32_t y_q8)
{
  int32_t xi = round_q8(x_q8), yi = round_q8(y_q8);
  if ((xi < s_x0) || (yi < s_y0) || (xi >= s_x1) || (yi >= s_y1)) { return 0; }
  return s_red[(uint32_t)yi * s_w + (uint32_t)xi];
}

/* point on a ray: c + r*dir, r in q8, dir in Q16 */
static int32_t ray_q8(int32_t c_q8, int32_t r_q8, int32_t d_q16)
{
  return c_q8 + (int32_t)(((int64_t)r_q8 * d_q16 + 32768) >> 16);
}

/* ---------------- Kasa algebraic circle fit ---------------- */
typedef struct { int32_t x, y; } pt_t;

static int kasa_fit(const pt_t *p, int n, int32_t *cx, int32_t *cy, int32_t *r)
{
  int64_t sx = 0, sy = 0, suu = 0, svv = 0, suv = 0, suuu = 0, svvv = 0, suvv = 0, suuv = 0;
  int64_t b1, b2, num_u, num_v, det, r2;
  int32_t mx, my, uc, vc;
  int i, sa, sb, m;
  if (n < 3) { return -1; }
  for (i = 0; i < n; i++) { sx += p[i].x; sy += p[i].y; }
  mx = (int32_t)(sx / n);
  my = (int32_t)(sy / n);
  for (i = 0; i < n; i++)
  {
    int64_t u = p[i].x - mx, v = p[i].y - my;
    int64_t uu = u * u, vv = v * v;
    suu += uu; svv += vv; suv += u * v;
    suuu += uu * u; svvv += vv * v; suvv += u * vv; suuv += uu * v;
  }
  b1 = suuu + suvv;
  b2 = svvv + suuv;
  m = bits_needed(suu); if (bits_needed(svv) > m) { m = bits_needed(svv); } if (bits_needed(suv) > m) { m = bits_needed(suv); }
  sa = (m > 28) ? (m - 28) : 0;
  m = bits_needed(b1); if (bits_needed(b2) > m) { m = bits_needed(b2); }
  sb = (m > 31) ? (m - 31) : 0;
  {
    int64_t a11 = suu >> sa, a22 = svv >> sa, a12 = suv >> sa;
    int64_t c1 = b1 >> sb, c2 = b2 >> sb;
    det = 2 * (a11 * a22 - a12 * a12);
    if (det <= 0) { return -1; }
    num_u = a22 * c1 - a12 * c2;
    num_v = a11 * c2 - a12 * c1;
    uc = (int32_t)muldiv_p2(num_u, det, sb - sa);
    vc = (int32_t)muldiv_p2(num_v, det, sb - sa);
  }
  r2 = (int64_t)uc * uc + (int64_t)vc * vc + (suu + svv) / n;
  if (r2 <= 16 * 65536) { return -1; }
  *cx = mx + uc;
  *cy = my + vc;
  *r = (int32_t)isqrt64((uint64_t)r2);
  return 0;
}

/* sum of squared radial residuals (q16) */
static int64_t circle_ss(const pt_t *p, int n, int32_t cx, int32_t cy, int32_t r)
{
  int64_t ss = 0;
  int i;
  for (i = 0; i < n; i++)
  {
    int64_t dx = p[i].x - cx, dy = p[i].y - cy;
    int32_t d = (int32_t)isqrt64((uint64_t)(dx * dx + dy * dy)) - r;
    ss += (int64_t)d * d;
  }
  return ss;
}

/* ---------------- chamfer 3-4 distance on a window ---------------- */
/* mask(x,y) != 0 -> foreground. returns argmax (first in raster order) */
typedef int (*mask_fn)(int32_t x, int32_t y, uint32_t arg);

static uint16_t chamfer(int32_t x0, int32_t y0, int32_t w, int32_t h, mask_fn fn, uint32_t arg,
                        int32_t *ax, int32_t *ay)
{
  int32_t x, y;
  uint16_t best = 0U;
  *ax = 0; *ay = 0;
  for (y = 0; y < h; y++)
  {
    dial_yield();
    for (x = 0; x < w; x++)
    {
      uint32_t v;
      uint16_t *d = &s_dt[y * w + x];
      if (!fn(x0 + x, y0 + y, arg)) { *d = 0U; continue; }
      v = DT_INF;
      if (x) { uint32_t t = d[-1] + 3U; if (t < v) { v = t; } }
      if (y) { uint32_t t = d[-w] + 3U; if (t < v) { v = t; } }
      if (x && y) { uint32_t t = d[-w - 1] + 4U; if (t < v) { v = t; } }
      if ((x < w - 1) && y) { uint32_t t = d[-w + 1] + 4U; if (t < v) { v = t; } }
      *d = (uint16_t)v;
    }
  }
  for (y = h - 1; y >= 0; y--)
  {
    dial_yield();
    for (x = w - 1; x >= 0; x--)
    {
      uint32_t v;
      uint16_t *d = &s_dt[y * w + x];
      if (*d == 0U) { continue; }
      v = *d;
      if (x < w - 1) { uint32_t t = d[1] + 3U; if (t < v) { v = t; } }
      if (y < h - 1) { uint32_t t = d[w] + 3U; if (t < v) { v = t; } }
      if ((x < w - 1) && (y < h - 1)) { uint32_t t = d[w + 1] + 4U; if (t < v) { v = t; } }
      if (x && (y < h - 1)) { uint32_t t = d[w - 1] + 4U; if (t < v) { v = t; } }
      *d = (uint16_t)v;
    }
  }
  for (y = 0; y < h; y++)
  {
    dial_yield();
    for (x = 0; x < w; x++)
    {
      if (s_dt[y * w + x] > best) { best = s_dt[y * w + x]; *ax = x0 + x; *ay = y0 + y; }
    }
  }
  return best;
}

static int mask_red(int32_t x, int32_t y, uint32_t arg)
{
  (void)arg;
  return s_red[(uint32_t)y * s_w + (uint32_t)x];
}
static int mask_lab(int32_t x, int32_t y, uint32_t arg)
{
  return s_lab[(uint32_t)y * s_w + (uint32_t)x] == (uint16_t)arg;
}

/* ---------------- step 1: seam restore ---------------- */
static void px8(uint16_t c, int *r, int *g, int *b)
{
  *r = s_r5to8[c >> 11];
  *g = s_g6to8[(c >> 5) & 63U];
  *b = s_r5to8[c & 31U];
}
static uint32_t pxdiff(uint16_t a, uint16_t b)
{
  int r1, g1, b1, r2, g2, b2;
  px8(a, &r1, &g1, &b1);
  px8(b, &r2, &g2, &b2);
  return (uint32_t)(iabs32(r1 - r2) + iabs32(g1 - g2) + iabs32(b1 - b2));
}

/* column with the largest mean adjacent-column difference; verified by the
 * wrap continuity W-1 -> 0.  Returns seam column (0 = none) and mode. */
static int seam_detect(const uint16_t *raw, uint16_t w, uint16_t h, uint8_t *mode, uint32_t *colsum)
{
  uint32_t x, y, best = 0U, bestx = 0U;
  uint64_t tot = 0U, wrap_row = 0U, wrap_lin = 0U;
  for (x = 0U; x < w; x++) { colsum[x] = 0U; }
  for (y = 0U; y < h; y++)
  {
    const uint16_t *row = raw + y * w;
    dial_yield();
    for (x = 1U; x < w; x++) { colsum[x] += pxdiff(row[x], row[x - 1U]); }
    wrap_row += pxdiff(row[w - 1U], row[0]);
    if (y + 1U < h) { wrap_lin += pxdiff(row[w - 1U], row[w]); }
  }
  for (x = 1U; x < w; x++)
  {
    tot += colsum[x];
    if (colsum[x] > best) { best = colsum[x]; bestx = x; }
  }
  if (DIAL_SEAM_MODE_CFG == 2) { *mode = (wrap_lin * h < wrap_row * (h - 1U)) ? 1U : 0U; }
  else { *mode = (uint8_t)DIAL_SEAM_MODE_CFG; }
  /* real seam: >= 4x the mean column difference and >= 3x the wrap difference */
  if (((uint64_t)best * (w - 1U) < 4U * tot) || ((uint64_t)best < 3U * wrap_row)) { return 0; }
  return (int)bestx;
}

/* ---------------- step 2: rectification (+ seam remap) ---------------- */
static uint16_t pack565(int r, int g, int b)
{
  /* inverse of the 565->888 table above */
  int r5 = (r * 31 + 127) / 255, g6 = (g * 63 + 127) / 255, b5 = (b * 31 + 127) / 255;
  return (uint16_t)((r5 << 11) | (g6 << 5) | b5);
}

static int is_red_px(int r, int g, int b)
{
  return (r >= RED_RMIN) && (r > g + RED_DG) && (r > b + RED_DB) && (g < RED_GMAX) && (b < RED_BMAX);
}

static void rectify_roi(const uint16_t *src, uint16_t *out, uint8_t *L, uint8_t *red,
                       uint16_t w, uint16_t h, int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
  int32_t x, y;
  if (x0 < 0) { x0 = 0; }
  if (y0 < 0) { y0 = 0; }
  if (x1 > (int32_t)w) { x1 = (int32_t)w; }
  if (y1 > (int32_t)h) { y1 = (int32_t)h; }
  if ((x1 <= x0) || (y1 <= y0)) { return; }
  for (y = y0; y < y1; y++)
  {
    dial_yield();
#if DIAL_RECTIFY_ON
    int64_t X = (int64_t)HI11 * x0 + (int64_t)HI12 * y + HI13;
    int64_t Y = (int64_t)HI21 * x0 + (int64_t)HI22 * y + HI23;
    int64_t D = (int64_t)HI31 * x0 + (int64_t)HI32 * y + ((int64_t)1 << 32);
#endif
    for (x = x0; x < x1; x++)
    {
      uint32_t o = (uint32_t)y * w + (uint32_t)x;
      int r, g, b;
#if DIAL_RECTIFY_ON
      /* 1/D by series (D = 1 - e, |e| < 0.05): 1 + e + e^2 + e^3 (Q30) */
      int64_t e30 = (((int64_t)1 << 32) - D) >> 2;
      int64_t e2 = (e30 * e30) >> 30;
      int64_t inv = ((int64_t)1 << 30) + e30 + e2 + ((e2 * e30) >> 30);
      int32_t sx = (int32_t)((X * inv) >> 42);   /* q8 */
      int32_t sy = (int32_t)((Y * inv) >> 42);
      int32_t xi = sx >> 8, yi = sy >> 8, fx = sx & 255, fy = sy & 255;
      X += HI11; Y += HI21; D += HI31;
      if ((xi < 0) || (yi < 0) || (xi >= (int32_t)w - 1) || (yi >= (int32_t)h - 1))
      {
        r = g = b = 0;
      }
      else
      {
        const uint16_t *p = src + (uint32_t)yi * w + (uint32_t)xi;
        int r00, g00, b00, r01, g01, b01, r10, g10, b10, r11, g11, b11;
        px8(p[0], &r00, &g00, &b00); px8(p[1], &r01, &g01, &b01);
        px8(p[w], &r10, &g10, &b10); px8(p[w + 1], &r11, &g11, &b11);
        r = (((r00 * (256 - fx) + r01 * fx) * (256 - fy)) + ((r10 * (256 - fx) + r11 * fx) * fy)) >> 16;
        g = (((g00 * (256 - fx) + g01 * fx) * (256 - fy)) + ((g10 * (256 - fx) + g11 * fx) * fy)) >> 16;
        b = (((b00 * (256 - fx) + b01 * fx) * (256 - fy)) + ((b10 * (256 - fx) + b11 * fx) * fy)) >> 16;
      }
#else
      px8(src[o], &r, &g, &b);
#endif
      out[o] = pack565(r, g, b);
      r = s_lvl[r]; g = s_lvl[g]; b = s_lvl[b];
      L[o] = (uint8_t)((r + 2 * g + b) >> 2);
      red[o] = (uint8_t)is_red_px(r, g, b);
    }
  }
}

static void rectify(const uint16_t *src, uint16_t *out, uint8_t *L, uint8_t *red, uint16_t w, uint16_t h)
{
  rectify_roi(src, out, L, red, w, h, 0, 0, (int32_t)w, (int32_t)h);
}

/* luma percentiles of the frame -> level LUT. Near-unity corrections are
 * ignored so a well-lit frame is processed exactly like the PC verification. */
static void level_setup(const uint16_t *src, uint32_t npx, uint32_t *hist)
{
  uint32_t i, acc = 0U, p10 = 0U, p90 = 0U;
  int32_t g = 256, off = 0;
  for (i = 0U; i < 256U; i++) { hist[i] = 0U; }
  if (DIAL_AUTO_LEVEL != 0)
  {
    for (i = 0U; i < npx; i += 4U)
    {
      int r, gg, b;
      if ((i & 1023U) == 0U) { dial_yield(); }
      px8(src[i], &r, &gg, &b);
      hist[(r + 2 * gg + b) >> 2]++;
    }
    for (i = 0U; i < 256U; i++)
    {
      acc += hist[i];
      if ((p10 == 0U) && (acc * 10U >= npx / 4U)) { p10 = i; }
      if ((p90 == 0U) && (acc * 10U >= 9U * (npx / 4U))) { p90 = i; }
    }
    if (DIAL_AUTO_LEVEL == 1)
    {
      if (p90 > 0U) { g = (int32_t)((LEVEL_REF_HI * 256U + p90 / 2U) / p90); }
    }
    else if (p90 > p10 + 20U)
    {
      g = (int32_t)(((LEVEL_REF_HI - LEVEL_REF_LO) * 256 + (int32_t)(p90 - p10) / 2) / (int32_t)(p90 - p10));
      off = LEVEL_REF_LO * 256 - g * (int32_t)p10;     /* q8 */
    }
    if (g < 128) { g = 128; }
    if (g > 512) { g = 512; }
    if ((iabs32(g - 256) <= 8) && (iabs32(off) <= 4 * 256)) { g = 256; off = 0; }
  }
  s_gain_q8 = g; s_off = off;
  for (i = 0U; i < 256U; i++)
  {
    int32_t v = ((int32_t)i * g + off + 128) >> 8;
    s_lvl[i] = (uint8_t)((v < 0) ? 0 : ((v > 255) ? 255 : v));
  }
}

/* ---------------- step 3: red blobs (4-connected union-find) ---------------- */
typedef struct { uint32_t area; int64_t sx, sy; uint16_t id; uint32_t first; } blob_t;

static uint16_t uf_find(uint16_t *par, uint16_t a)
{
  while (par[a] != a) { par[a] = par[par[a]]; a = par[a]; }
  return a;
}

static int find_blobs(uint16_t *par, uint32_t *area, blob_t *bl, int maxb)
{
  uint32_t x, y, n = 0U;
  int nb = 0, i, j;
  for (y = 0U; y < s_h; y++)
  {
    dial_yield();
    for (x = 0U; x < s_w; x++)
    {
      uint32_t o = y * s_w + x;
      uint16_t up, lf;
      if (!s_red[o]) { s_lab[o] = 0U; continue; }
      up = y ? s_lab[o - s_w] : 0U;
      lf = x ? s_lab[o - 1U] : 0U;
      if (up && lf)
      {
        uint16_t a = uf_find(par, up), b = uf_find(par, lf);
        s_lab[o] = up;
        if (a != b) { par[b] = a; }
      }
      else if (up) { s_lab[o] = up; }
      else if (lf) { s_lab[o] = lf; }
      else if (n + 1U < MAX_LABELS) { n++; par[n] = (uint16_t)n; s_lab[o] = (uint16_t)n; }
      else { s_lab[o] = 0U; }
    }
  }
  for (i = 0; i <= (int)n; i++) { area[i] = 0U; }
  /* resolve to roots; collect stats per root in first-appearance order */
  for (y = 0U; y < s_h; y++)
  {
    dial_yield();
    for (x = 0U; x < s_w; x++)
    {
      uint32_t o = y * s_w + x;
      uint16_t r;
      if (!s_lab[o]) { continue; }
      r = uf_find(par, s_lab[o]);
      s_lab[o] = r;
      area[r]++;
    }
  }
  for (y = 0U; (y < s_h) && (nb < maxb); y++)
  {
    dial_yield();
    for (x = 0U; x < s_w; x++)
    {
      uint32_t o = y * s_w + x;
      uint16_t r = s_lab[o];
      if (!r || (area[r] < BLOB_MIN_AREA)) { continue; }
      for (i = 0; i < nb; i++) { if (bl[i].id == r) { break; } }
      if (i == nb)
      {
        if (nb >= maxb) { continue; }
        bl[nb].id = r; bl[nb].area = 0U; bl[nb].sx = 0; bl[nb].sy = 0; bl[nb].first = o; nb++;
      }
    }
  }
  for (y = 0U; y < s_h; y++)
  {
    dial_yield();
    for (x = 0U; x < s_w; x++)
    {
      uint16_t r = s_lab[y * s_w + x];
      if (!r || (area[r] < BLOB_MIN_AREA)) { continue; }
      for (i = 0; i < nb; i++)
      {
        if (bl[i].id == r) { bl[i].area++; bl[i].sx += (int64_t)x; bl[i].sy += (int64_t)y; break; }
      }
    }
  }
  /* stable sort by area, descending */
  for (i = 1; i < nb; i++)
  {
    blob_t t = bl[i];
    j = i - 1;
    while ((j >= 0) && (bl[j].area < t.area)) { bl[j + 1] = bl[j]; j--; }
    bl[j + 1] = t;
  }
  return nb;
}

/* ---------------- step 4: arc-seed circle ---------------- */
static pt_t s_pts[MAX_ARC_PTS];
static pt_t s_seed[360];
static int16_t s_rr[360];

/* adaptive trace along the circle from a_start (deg), step +1/-1 */
static int arc_trace(int32_t cx, int32_t cy, int32_t R, int a_start, int step, pt_t *outp, int maxp)
{
  int vals[5] = { s_arc_t_strong, s_arc_t_strong, s_arc_t_strong, s_arc_t_strong, s_arc_t_strong };
  int vi = 0, vsum = 5 * s_arc_t_strong;
  int32_t r_prev_q8 = R;
  int miss = 0, n = 0, a = a_start, it;
  for (it = 0; it < 360; it++)
  {
    int32_t c, s, base, r, rmin = 0;
    int vmin = 1000, vout = -1, d, ok = 1;
    a = ((a + step) % 360 + 360) % 360;
    c = cos_deg(a); s = sin_deg(a);
    base = round_q8(r_prev_q8);
    for (r = base - 3; r <= base + 3; r++)
    {
      int v = L_at(ray_q8(cx, r * 256, c), ray_q8(cy, r * 256, s));
      if (v < 0) { ok = 0; break; }
      if (v < vmin) { vmin = v; rmin = r; }
    }
    if (!ok) { break; }
    for (d = 3; d < 8; d++)
    {
      int t = L_at(ray_q8(cx, (rmin + d) * 256, c), ray_q8(cy, (rmin + d) * 256, s));
      if (t > vout) { vout = t; }
    }
    if (vout < 0) { vout = vmin; }
    if (red_at(ray_q8(cx, rmin * 256, c), ray_q8(cy, rmin * 256, s))) { break; }
    if ((vout - vmin < s_arc_white_gap) && (vmin > s_arc_white - 20)) { break; }
    if ((5 * vmin <= vsum + 5 * ARC_NEAR) && (vout - vmin >= s_arc_contrast))
    {
      if (n < maxp)
      {
        outp[n].x = ray_q8(cx, rmin * 256, c);
        outp[n].y = ray_q8(cy, rmin * 256, s);
        n++;
      }
      vsum += vmin - vals[vi]; vals[vi] = vmin; vi = (vi + 1) % 5;
      r_prev_q8 = rmin * 256;
      miss = 0;
    }
    else
    {
      miss++;
      if (miss > ARC_MISS) { break; }
    }
  }
  return n;
}

static int arc_circle(int32_t hx, int32_t hy, int32_t *pcx, int32_t *pcy, int32_t *pR, int *pcov)
{
  int a, i, run_s = 0, run_n = 0, best_s = 0, best_n = 0, nseed, rnd, npts;
  int32_t cx, cy, R;
  for (a = 0; a < 360; a++)
  {
    int32_t c = cos_deg(a), s = sin_deg(a), r;
    int best = -1;
    s_rr[a] = -1;
    for (r = ARC_RMIN; r < ARC_RMAX; r++)
    {
      int v = L_at(ray_q8(hx, r * 256, c), ray_q8(hy, r * 256, s));
      if (v < 0) { break; }
      if (v <= s_arc_t_strong)
      {
        int d, mx = -1;
        for (d = 2; d < 9; d++)
        {
          int t = L_at(ray_q8(hx, (r + d) * 256, c), ray_q8(hy, (r + d) * 256, s));
          if (t > mx) { mx = t; }
        }
        if (mx >= s_arc_white) { best = r; }
      }
    }
    if (best >= 0)
    {
      int vb = 1000, rb = best;
      for (r = best - 2; r < best + 2; r++)
      {
        int v = L_at(ray_q8(hx, r * 256, c), ray_q8(hy, r * 256, s));
        if ((v >= 0) && (v < vb)) { vb = v; rb = r; }
      }
      s_rr[a] = (int16_t)rb;
    }
  }
  /* longest circular run with |dr| <= 2 */
  for (i = 0; i < 720; i++)
  {
    int r = s_rr[i % 360];
    int prev = (run_n > 0) ? s_rr[(run_s + run_n - 1) % 360] : 0;
    if ((r >= 0) && ((run_n == 0) || (iabs32(r - prev) <= 2)))
    {
      if (run_n == 0) { run_s = i; }
      run_n++;
    }
    else
    {
      if (run_n > best_n) { best_n = run_n; best_s = run_s; }
      if (r >= 0) { run_s = i; run_n = 1; } else { run_n = 0; }
    }
    if (run_n >= 360) { break; }
  }
  if (run_n > best_n) { best_n = run_n; best_s = run_s; }
  if (best_n > 360) { best_n = 360; }
  if (best_n < ARC_SEED_MIN) { return -1; }
  nseed = best_n;
  for (i = 0; i < nseed; i++)
  {
    int aa = (best_s + i) % 360;
    s_seed[i].x = ray_q8(hx, s_rr[aa] * 256, cos_deg(aa));
    s_seed[i].y = ray_q8(hy, s_rr[aa] * 256, sin_deg(aa));
  }
  if (kasa_fit(s_seed, nseed, &cx, &cy, &R) != 0) { return -1; }
  if (circle_ss(s_seed, nseed, cx, cy, R) * 100 > (int64_t)ARC_SEED_RMS2_X100 * nseed * 65536) { return -1; }
  {
    int32_t seed_cx = cx, seed_cy = cy, seed_R = R;
    int a_end = (atan2_mdeg(s_seed[nseed - 1].y - cy, s_seed[nseed - 1].x - cx) + 500) / 1000;
    int a_beg = (atan2_mdeg(s_seed[0].y - cy, s_seed[0].x - cx) + 500) / 1000;
    memcpy(s_pts, s_seed, (size_t)nseed * sizeof(pt_t));
    npts = nseed;
    for (rnd = 0; rnd < 3; rnd++)
    {
      int nf, nbw, nn;
      int32_t ncx, ncy, nR;
      nn = nseed;
      memcpy(s_pts, s_seed, (size_t)nseed * sizeof(pt_t));
      nf = arc_trace(cx, cy, R, a_end, +1, &s_pts[nn], MAX_ARC_PTS - nn);
      nn += nf;
      nbw = arc_trace(cx, cy, R, a_beg, -1, &s_pts[nn], MAX_ARC_PTS - nn);
      nn += nbw;
      if (kasa_fit(s_pts, nn, &ncx, &ncy, &nR) != 0) { break; }
      cx = ncx; cy = ncy; R = nR; npts = nn;
    }
    /* If tracing shrinks the circle (short / hidden rim), keep the seed. */
    if (R < seed_R - 2 * 256)
    {
      cx = seed_cx; cy = seed_cy; R = seed_R;
      memcpy(s_pts, s_seed, (size_t)nseed * sizeof(pt_t));
      npts = nseed;
    }
  }
  {
    uint8_t occ[36];
    int cov = 0;
    memset(occ, 0, sizeof(occ));
    for (i = 0; i < npts; i++) { occ[atan2_mdeg(s_pts[i].y - cy, s_pts[i].x - cx) / 10000] = 1U; }
    for (i = 0; i < 36; i++) { cov += occ[i]; }
    *pcov = cov;
  }
  *pcx = cx; *pcy = cy; *pR = R;
  return 0;
}

/* ---------------- step 5: ticks ---------------- */
static void tick_len_profile(int32_t cx, int32_t cy, int32_t R, int16_t *hist)
{
  int a;
  int rmax_in = (int)(((int64_t)R * 24 + 12800) / 25600);   /* round(R*0.24) */
  for (a = 0; a < 360; a++)
  {
    int32_t c = cos_deg(a), s = sin_deg(a);
    int d, vr = 1000, dr = 0, ok = 1, k, n = 0, gap = 0, thr;
    int32_t r0;
    hist[a] = 0;
    for (d = -2; d <= 2; d++)
    {
      int v = L_at(ray_q8(cx, R + d * 256, c), ray_q8(cy, R + d * 256, s));
      if (v < 0) { ok = 0; break; }
      if (v < vr) { vr = v; dr = d; }
    }
    if (!ok) { continue; }
    thr = vr + ARC_NEAR;
    r0 = R + dr * 256;
    for (k = 3; k < rmax_in; k++)
    {
      int32_t px = ray_q8(cx, r0 - k * 256, c), py = ray_q8(cy, r0 - k * 256, s);
      int v = L_at(px, py);
      if (v < 0) { break; }
      if (red_at(px, py)) { n = -1; break; }
      if ((v <= thr) && (v <= s_dark_l)) { n = k - 2; gap = 0; }
      else { gap++; if (gap > 1) { break; } }
    }
    hist[a] = (int16_t)((n > 0) ? n : 0);
  }
}

typedef struct { int32_t a; int32_t h; } peak_t;   /* a in mdeg */

static void sort_by_h_stable(peak_t *p, int n)
{
  int i, j;
  for (i = 1; i < n; i++)
  {
    peak_t t = p[i];
    j = i - 1;
    while ((j >= 0) && (p[j].h < t.h)) { p[j + 1] = p[j]; j--; }
    p[j + 1] = t;
  }
}
static void sort_by_a(peak_t *p, int n)
{
  int i, j;
  for (i = 1; i < n; i++)
  {
    peak_t t = p[i];
    j = i - 1;
    while ((j >= 0) && (p[j].a > t.a)) { p[j + 1] = p[j]; j--; }
    p[j + 1] = t;
  }
}

static int H360(const int16_t *h, int i) { return h[((i % 360) + 360) % 360]; }

static int find_peaks(const int16_t *hist, peak_t *out, int maxn, int min_sep)
{
  peak_t c[360];
  int nc = 0, i, j, n = 0;
  const int sep = min_sep, minh = 3;
  for (i = 0; i < 360; i++)
  {
    int v = hist[i], d, ok = 1;
    if ((v < minh) || (v < H360(hist, i - 1)) || (v < H360(hist, i + 1))) { continue; }
    for (d = -2; d <= 2; d++) { if (H360(hist, i + d) > v) { ok = 0; } }
    if (ok) { c[nc].a = i; c[nc].h = v; nc++; }
  }
  sort_by_h_stable(c, nc);
  for (i = 0; (i < nc) && (n < maxn); i++)
  {
    int ok = 1;
    for (j = 0; j < n; j++)
    {
      int d = iabs32(c[i].a - out[j].a);
      if (360 - d < d) { d = 360 - d; }
      if (d < sep) { ok = 0; break; }
    }
    if (ok) { out[n++] = c[i]; }
  }
  sort_by_a(out, n);
  for (i = 0; i < n; i++) { out[i].a *= 1000; }
  return n;
}

#if 0
/* Drop short peaks (< 0.75 of the upper-half median length).  Used on
 * 10-tick dials so a mid-sector shadow / paper-tip mark is not a tick. */
static int drop_short_peaks(peak_t *pk, int n)
{
  int h[64], i, j, ntop, mid;
  int32_t lmaj, thr;
  if (n <= 0) { return n; }
  if (n > 64) { n = 64; }
  for (i = 0; i < n; i++) { h[i] = pk[i].h; }
  for (i = 1; i < n; i++)
  {
    int t = h[i];
    j = i - 1;
    while ((j >= 0) && (h[j] > t)) { h[j + 1] = h[j]; j--; }
    h[j + 1] = t;
  }
  ntop = n / 2; if (ntop < 1) { ntop = 1; }
  mid = n - ntop + (ntop - 1) / 2;
  lmaj = h[mid];
  if ((ntop % 2) == 0) { lmaj = (h[mid] + h[mid + 1]) / 2; }
  thr = (lmaj * 3 + 2) / 4;            /* 0.75 * lmaj, rounded */
  j = 0;
  for (i = 0; i < n; i++) { if (pk[i].h >= thr) { pk[j++] = pk[i]; } }
  return j;
}

#endif

/* fit a 36 deg grid to the strongest peaks; returns offset of the slot grid */
static void phase_offset(const peak_t *pk, int n, int *off, int *major_base, int allow_half)
{
  peak_t top[40];
  int nt, o, i, best_o = 0;
  int64_t best_s = -1;
  if (n <= 0) { *off = 0; *major_base = 0; return; }
  nt = (n > 40) ? 40 : n;
  memcpy(top, pk, (size_t)nt * sizeof(peak_t));
  sort_by_h_stable(top, nt);
  if (nt > 10) { nt = 10; }
  for (o = 0; o < 36; o++)
  {
    int64_t acc = 0;
    for (i = 0; i < nt; i++)
    {
      int a = top[i].a / 1000;
      int d = iabs32(angdiff_mdeg(a * 1000, o * 1000)) / 1000;
      /* nearest 36-grid line: |ang_diff| folded into 0..18 */
      d = ((a - o) % 36 + 36) % 36;
      if (d > 18) { d = 36 - d; }
      if (d < 18) { acc += (int64_t)(18 - d) * top[i].h; }
    }
    if (acc > best_s) { best_s = acc; best_o = o; }
  }
  if (!allow_half) { *off = best_o % 36; *major_base = 0; }
  else if (best_o < 18) { *off = best_o; *major_base = 0; }
  else { *off = best_o - 18; *major_base = 1; }
}

static int32_t refine_peak(const int16_t *hist, int32_t a0_mdeg)
{
  int a0 = a0_mdeg / 1000, d, k = 0, peak = -1, cnt = 0;
  int64_t sw = 0, sh = 0;
  int hh[9];
  for (d = -4; d <= 4; d++)
  {
    hh[d + 4] = H360(hist, a0 + d);
    if (hh[d + 4] > peak) { peak = hh[d + 4]; k = d + 4; }
  }
  if (peak <= 0) { return a0 * 1000; }
  for (d = 0; d < 9; d++)
  {
    if (10 * hh[d] >= 7 * peak) { cnt++; sw += (int64_t)(a0 + d - 4) * 1000 * hh[d]; sh += hh[d]; }
  }
  if (cnt <= 1) { return wrap_mdeg((a0 + k - 4) * 1000); }
  return wrap_mdeg((int32_t)((sw + sh / 2) / sh));
}

/* ---------------- tick authenticity features (port of dial_read.py) ------ */
#define TICK_GAP_MAJOR      1
#define TICK_GAP_HALF       3
#define PAIR_ANG_MDEG       8000
#define PAIR_CENTER_NUM     3       /* 0.03 = 3/100 */
#define PAIR_CENTER_DEN     100
#define PTR_NEAR_MDEG       15000
#define PTR_RIM_MIN_INK     4
#define CONTRAST_FLOOR      20
#define CONTRAST_FRAC_NUM   40
#define CONTRAST_FRAC_DEN   100
#define INK_BRIGHT_ADD      20
#define CLOSE_MDEG_MAJOR    18000
#define CLOSE_MDEG_HALF     12000
#define SLOT_DIP_MIN        12
#define MAX_FEATS           64

typedef struct
{
  int32_t a, h;             /* a in mdeg */
  int len, n_red, mean_ink, mean_face, contrast;
  int32_t xin, yin;         /* inner tip, q8 */
  int has_xy;
  int slot, side, slot_dip, slot_w, bridged_gap;
  int32_t dptr;             /* mdeg; 999999 if unknown */
  int rb_gap;
} feat_t;

static int median_i(int *a, int n)
{
  int i, j, t;
  if (n <= 0) { return 0; }
  for (i = 1; i < n; i++)
  {
    t = a[i];
    j = i - 1;
    while ((j >= 0) && (a[j] > t)) { a[j + 1] = a[j]; j--; }
    a[j + 1] = t;
  }
  if (n & 1) { return a[n / 2]; }
  return (a[n / 2 - 1] + a[n / 2]) / 2;
}

static int ray_mean_luma(int32_t cx, int32_t cy, int32_t R, int ang)
{
  int32_t c = cos_deg(ang), s = sin_deg(ang);
  int r0 = (int)(((int64_t)R * 84 + 12800) / 25600);
  int r1 = (int)(((int64_t)R * 97 + 12800) / 25600);
  int rr, sum = 0, n = 0, v;
  int32_t px, py;
  for (rr = r0; rr < r1; rr++)
  {
    px = ray_q8(cx, rr * 256, c);
    py = ray_q8(cy, rr * 256, s);
    if (red_at(px, py)) { continue; }
    v = L_at(px, py);
    if (v < 0) { continue; }
    sum += v;
    n++;
  }
  return n ? (sum / n) : 255;
}

/* Peek the rim (min luma in R-2..R+2) at angle a_deg. Returns 0, or -1 if off-frame. */
static int rim_thr(int32_t cx, int32_t cy, int32_t R, int a_deg, int *pthr, int32_t *pr0, int32_t *pc, int32_t *ps)
{
  int32_t c = cos_deg(a_deg), s = sin_deg(a_deg);
  int d, vr = 1000, dr = 0, v;
  *pc = c; *ps = s;
  for (d = -2; d <= 2; d++)
  {
    v = L_at(ray_q8(cx, R + d * 256, c), ray_q8(cy, R + d * 256, s));
    if (v < 0) { return -1; }
    if (v < vr) { vr = v; dr = d; }
  }
  *pthr = vr + ARC_NEAR;
  *pr0 = R + dr * 256;
  return 0;
}

static int tick_ink_features(int32_t cx, int32_t cy, int32_t R, int32_t a_mdeg, int gap_max, feat_t *ft)
{
  int a = ((a_mdeg + 500) / 1000) % 360;
  int rmax_in = (int)(((int64_t)R * 24 + 12800) / 25600);
  int32_t c, s, r0, px, py;
  int thr, k, v, isr, gap = 0, started = 0, n_dark = 0, n_red = 0, bridged = 0;
  int lumas[80], nl = 0, face[48], nf = 0;
  int da, sides[16], ns = 0, slot, side, half, slot_w, sum;
  if (a < 0) { a += 360; }
  memset(ft, 0, sizeof(*ft));
  if (rim_thr(cx, cy, R, a, &thr, &r0, &c, &s) != 0) { return -1; }
  for (k = 3; k < rmax_in; k++)
  {
    px = ray_q8(cx, r0 - k * 256, c);
    py = ray_q8(cy, r0 - k * 256, s);
    v = L_at(px, py);
    if (v < 0) { break; }
    isr = red_at(px, py);
    if (isr) { n_red++; }
    if ((v <= thr) && (v <= s_dark_l))
    {
      if (started && (gap > 1) && !isr) { if (gap > bridged) { bridged = gap; } }
      started = 1;
      if (!isr)
      {
        if (nl < 80) { lumas[nl++] = v; }
        n_dark++;
        ft->xin = px; ft->yin = py; ft->has_xy = 1;
      }
      gap = 0;
    }
    else
    {
      gap++;
      if ((gap > gap_max) && started) { break; }
    }
  }
  for (da = 0; da < 4; da++)
  {
    static const int off[4] = { 10, -10, 12, -12 };
    int32_t cc = cos_deg(a + off[da]), ss = sin_deg(a + off[da]);
    for (k = 6; k < 16; k++)
    {
      v = L_at(ray_q8(cx, r0 - k * 256, cc), ray_q8(cy, r0 - k * 256, ss));
      if ((v >= 0) && (nf < 48)) { face[nf++] = v; }
    }
  }
  sum = 0;
  for (k = 0; k < nl; k++) { sum += lumas[k]; }
  ft->len = n_dark;
  ft->n_red = n_red;
  ft->mean_ink = nl ? (sum / nl) : 255;
  ft->mean_face = nf ? median_i(face, nf) : 200;
  ft->contrast = ft->mean_face - ft->mean_ink;
  ft->bridged_gap = bridged;
  slot = (ray_mean_luma(cx, cy, R, a - 1) + ray_mean_luma(cx, cy, R, a) + ray_mean_luma(cx, cy, R, a + 1)) / 3;
  for (da = -12; da <= -5; da++) { sides[ns++] = ray_mean_luma(cx, cy, R, a + da); }
  for (da = 5; da <= 12; da++) { sides[ns++] = ray_mean_luma(cx, cy, R, a + da); }
  side = ns ? median_i(sides, ns) : 255;
  ft->slot = slot;
  ft->side = side;
  ft->slot_dip = side - slot;
  half = (slot + side) / 2;
  slot_w = 16;
  for (da = 1; da < 16; da++)
  {
    if ((ray_mean_luma(cx, cy, R, a + da) >= half) && (ray_mean_luma(cx, cy, R, a - da) >= half))
    {
      slot_w = da;
      break;
    }
  }
  ft->slot_w = slot_w;
  return 0;
}

/* Outer-band only (k = 0..8 inside the rim). 0 = touch, 99 = no red. */
static int red_black_gap(int32_t cx, int32_t cy, int32_t R, int32_t a_mdeg)
{
  int a = ((a_mdeg + 500) / 1000) % 360;
  int32_t c, s, r0, px, py;
  int thr, k, v, isr, isb, min_gap = 99;
  int seen_b = 0, seen_r = 0, w_after_b = 0, w_after_r = 0;
  if (a < 0) { a += 360; }
  if (rim_thr(cx, cy, R, a, &thr, &r0, &c, &s) != 0) { return 99; }
  for (k = 0; k < 9; k++)
  {
    px = ray_q8(cx, r0 - k * 256, c);
    py = ray_q8(cy, r0 - k * 256, s);
    v = L_at(px, py);
    if (v < 0) { break; }
    isr = red_at(px, py);
    isb = (!isr) && (v <= thr) && (v <= s_dark_l);
    if (isr)
    {
      if (seen_b && (w_after_b < min_gap)) { min_gap = w_after_b; }
      seen_r = 1; w_after_r = 0;
    }
    else if (isb)
    {
      if (seen_r && (w_after_r < min_gap)) { min_gap = w_after_r; }
      seen_b = 1; w_after_b = 0;
    }
    else
    {
      if (seen_b) { w_after_b++; }
      if (seen_r) { w_after_r++; }
    }
  }
  return min_gap;
}

/* Perp. distance from center to the line through ink tips, compared to 0.03 R.
 * Returns 1 if dist/R <= PAIR_CENTER_FRAC. */
static int line_center_ok(int32_t x1, int32_t y1, int32_t x2, int32_t y2,
                          int32_t cx, int32_t cy, int32_t R, int32_t *pfrac_q16)
{
  int64_t dx = (int64_t)x2 - x1, dy = (int64_t)y2 - y1;
  uint64_t L2, L;
  int64_t num, dist;
  L2 = (uint64_t)(dx * dx + dy * dy);
  if (L2 < (uint64_t)256U * 256U) { return 0; }          /* shorter than 1 px */
  L = isqrt64(L2);
  num = iabs64(((int64_t)cx - x1) * dy - ((int64_t)cy - y1) * dx);
  dist = (int64_t)(num / (int64_t)L);                     /* q8 */
  if (pfrac_q16) { *pfrac_q16 = (int32_t)((dist * 65536) / (R > 0 ? R : 1)); }
  return (dist * PAIR_CENTER_DEN <= (int64_t)R * PAIR_CENTER_NUM);
}

static int peak_pair_ok(const feat_t *f, const feat_t *oth, int n, int32_t cx, int32_t cy, int32_t R)
{
  int i;
  for (i = 0; i < n; i++)
  {
    if (&oth[i] == f) { continue; }
    if (iabs32(angdiff_mdeg(f->a + 180000, oth[i].a)) > PAIR_ANG_MDEG) { continue; }
    if (!f->has_xy || !oth[i].has_xy) { continue; }
    if (line_center_ok(f->xin, f->yin, oth[i].xin, oth[i].yin, cx, cy, R, 0)) { return 1; }
  }
  return 0;
}

static int peak_strength(const feat_t *f, const feat_t *oth, int n, int32_t cx, int32_t cy, int32_t R)
{
  int tw = 10 - f->slot_w;
  if (tw < 0) { tw = 0; }
  return f->slot_dip * 2 + f->contrast + f->len * 4 + tw * 5
       + (peak_pair_ok(f, oth, n, cx, cy, R) ? 50 : 0);
}

static int resolve_close_peaks(feat_t *kept, int n, int32_t cx, int32_t cy, int32_t R, int allow_half)
{
  int close = allow_half ? CLOSE_MDEG_HALF : CLOSE_MDEG_MAJOR;
  for (;;)
  {
    int i, j, bi = -1, bj = -1;
    int32_t bda = close;
    for (i = 0; i < n; i++)
    {
      for (j = i + 1; j < n; j++)
      {
        int32_t da = iabs32(angdiff_mdeg(kept[i].a, kept[j].a));
        if (da < bda) { bda = da; bi = i; bj = j; }
      }
    }
    if (bi < 0) { break; }
    {
      int si = peak_strength(&kept[bi], kept, n, cx, cy, R);
      int sj = peak_strength(&kept[bj], kept, n, cx, cy, R);
      int wi, wj, drop;
      const feat_t *win, *lose;
      if ((si > sj) ||
          ((si == sj) && (kept[bi].slot_dip > kept[bj].slot_dip)) ||
          ((si == sj) && (kept[bi].slot_dip == kept[bj].slot_dip) && (kept[bi].slot_w < kept[bj].slot_w)) ||
          ((si == sj) && (kept[bi].slot_dip == kept[bj].slot_dip) && (kept[bi].slot_w == kept[bj].slot_w) && (kept[bi].h >= kept[bj].h)))
      {
        wi = bi; wj = bj;
      }
      else { wi = bj; wj = bi; }
      win = &kept[wi]; lose = &kept[wj];
#ifdef DIAL_HOST_DEBUG
      printf("  close %d/%d (%.1f): keep %d (格子 dip %d vs %d, slot %d/%d mid %d/%d, width %d/%d, ink %d/%d, ctr %d/%d, pair %d/%d)\n",
             lose->a / 1000, win->a / 1000, bda / 1000.0, win->a / 1000,
             win->slot_dip, lose->slot_dip, win->slot, lose->slot, win->side, lose->side,
             win->slot_w, lose->slot_w, win->len, lose->len, win->contrast, lose->contrast,
             peak_pair_ok(win, kept, n, cx, cy, R), peak_pair_ok(lose, kept, n, cx, cy, R));
#endif
      (void)win; (void)lose;
      drop = wj;
      for (i = drop; i < n - 1; i++) { kept[i] = kept[i + 1]; }
      n--;
    }
  }
  return n;
}

static int filter_tick_peaks(peak_t *pk, int n, int32_t cx, int32_t cy, int32_t R,
                             int32_t pang_mdeg, int have_pang, int allow_half)
{
  static feat_t feats[MAX_FEATS], stage1[MAX_FEATS], kept[MAX_FEATS];
  int nf = 0, i, j, n1, nk, gmax, med_ink, med_ctr;
  static int hs[MAX_FEATS], inks[MAX_FEATS], ctrs[MAX_FEATS];
  int lmaj, longs_n, ntop;
  static peak_t paired_pk[MAX_FEATS];
  static int used[MAX_FEATS], is_paired[MAX_FEATS];
  int n_paired, off_g, mb_g;
  int32_t step;
  if (n <= 0) { return 0; }
  if (n > MAX_FEATS) { n = MAX_FEATS; }
  gmax = allow_half ? TICK_GAP_HALF : TICK_GAP_MAJOR;
  for (i = 0; i < n; i++)
  {
    if (tick_ink_features(cx, cy, R, pk[i].a, gmax, &feats[nf]) != 0) { continue; }
    feats[nf].a = pk[i].a;
    feats[nf].h = pk[i].h;
    feats[nf].dptr = have_pang ? iabs32(angdiff_mdeg(pk[i].a, pang_mdeg)) : 999999;
    feats[nf].rb_gap = red_black_gap(cx, cy, R, pk[i].a);
    nf++;
  }
  if (nf <= 0) { return 0; }
#ifdef DIAL_HOST_DEBUG
  if (allow_half)
  {
    int any = 0;
    for (i = 0; i < nf; i++) { if (feats[i].bridged_gap > 1) { any = 1; break; } }
    if (any)
    {
      printf("  tadpole gap-merge (close breaks, max %dpx):", TICK_GAP_HALF);
      for (i = 0; i < nf; i++)
      {
        if (feats[i].bridged_gap > 1)
        {
          printf(" %d deg gap=%dpx len=%d", feats[i].a / 1000, feats[i].bridged_gap, feats[i].len);
        }
      }
      printf("\n");
    }
  }
#endif
  for (i = 0; i < nf; i++) { hs[i] = (int)feats[i].h; }
  /* insertion-sort a copy for the upper-half median */
  {
    int tmp[MAX_FEATS];
    for (i = 0; i < nf; i++) { tmp[i] = hs[i]; }
    (void)median_i(tmp, nf);                         /* sorts tmp ascending */
    ntop = nf / 2; if (ntop < 1) { ntop = 1; }
    lmaj = median_i(tmp + (nf - ntop), ntop);
  }
  longs_n = 0;
  for (i = 0; i < nf; i++)
  {
    if (feats[i].h * 4 >= lmaj * 3)
    {
      inks[longs_n] = feats[i].mean_ink;
      ctrs[longs_n] = feats[i].contrast;
      longs_n++;
    }
  }
  if (longs_n <= 0)
  {
    for (i = 0; i < nf; i++) { inks[i] = feats[i].mean_ink; ctrs[i] = feats[i].contrast; }
    longs_n = nf;
  }
  med_ink = median_i(inks, longs_n);
  med_ctr = median_i(ctrs, longs_n);

  n1 = resolve_close_peaks(feats, nf, cx, cy, R, allow_half);
  /* contrast / no-slot */
  {
    int w = 0;
    for (i = 0; i < n1; i++)
    {
      int drop = 0;
      if (feats[i].contrast < s_contrast_floor) { drop = 1; }
      else if ((feats[i].contrast * CONTRAST_FRAC_DEN < CONTRAST_FRAC_NUM * med_ctr)
               && (feats[i].mean_ink > med_ink + INK_BRIGHT_ADD)) { drop = 1; }
      if (!allow_half && (feats[i].slot_dip < SLOT_DIP_MIN) && (feats[i].slot_w >= 8)) { drop = 1; }
#ifdef DIAL_HOST_DEBUG
      if (drop)
      {
        printf("  drop a=%d h=%d len=%d ink=%d ctr=%d dip=%d w=%d rb=%d dptr=%.1f: contrast/no-slot\n",
               feats[i].a / 1000, (int)feats[i].h, feats[i].len, feats[i].mean_ink,
               feats[i].contrast, feats[i].slot_dip, feats[i].slot_w, feats[i].rb_gap,
               feats[i].dptr / 1000.0);
      }
#endif
      if (!drop) { if (w != i) { feats[w] = feats[i]; } w++; }
    }
    n1 = w;
  }
  for (i = 0; i < n1; i++) { stage1[i] = feats[i]; }

  /* 0-5 pairing through the center using measured ink tips */
  {
    typedef struct { int32_t da, dist; int nh, i, j; } edge_t;
    static edge_t edges[MAX_FEATS * 4];
    int ne = 0;
    memset(used, 0, sizeof(used));
    memset(is_paired, 0, sizeof(is_paired));
    for (i = 0; i < n1; i++)
    {
      for (j = i + 1; j < n1; j++)
      {
        int32_t da = iabs32(angdiff_mdeg(stage1[i].a + 180000, stage1[j].a));
        int32_t frac_q16 = 0;
        if (da > PAIR_ANG_MDEG) { continue; }
        if (!stage1[i].has_xy || !stage1[j].has_xy) { continue; }
        if (!line_center_ok(stage1[i].xin, stage1[i].yin, stage1[j].xin, stage1[j].yin,
                            cx, cy, R, &frac_q16))
        {
#ifdef DIAL_HOST_DEBUG
          printf("  pair %d-%d dAng=%.1f dist/R=%.4f: not through center\n",
                 stage1[i].a / 1000, stage1[j].a / 1000, da / 1000.0, frac_q16 / 65536.0);
#endif
          continue;
        }
        edges[ne].da = da;
        edges[ne].dist = frac_q16;
        edges[ne].nh = -((int)stage1[i].h + (int)stage1[j].h);
        edges[ne].i = i; edges[ne].j = j;
        ne++;
        if (ne >= (int)(sizeof(edges) / sizeof(edges[0]))) { break; }
      }
    }
    /* sort edges by (da, dist, nh) */
    for (i = 1; i < ne; i++)
    {
      edge_t t = edges[i];
      j = i - 1;
      while ((j >= 0) && ((edges[j].da > t.da) ||
             ((edges[j].da == t.da) && (edges[j].dist > t.dist)) ||
             ((edges[j].da == t.da) && (edges[j].dist == t.dist) && (edges[j].nh > t.nh))))
      {
        edges[j + 1] = edges[j]; j--;
      }
      edges[j + 1] = t;
    }
    n_paired = 0;
    for (i = 0; i < ne; i++)
    {
      int ia = edges[i].i, ja = edges[i].j;
      if (used[ia] || used[ja]) { continue; }
      used[ia] = used[ja] = 1;
      is_paired[ia] = is_paired[ja] = 1;
      paired_pk[n_paired].a = stage1[ia].a; paired_pk[n_paired].h = 10; n_paired++;
      paired_pk[n_paired].a = stage1[ja].a; paired_pk[n_paired].h = 10; n_paired++;
#ifdef DIAL_HOST_DEBUG
      printf("  pair %d-%d dAng=%.1f dist/R=%.4f: ok\n",
             stage1[ia].a / 1000, stage1[ja].a / 1000, edges[i].da / 1000.0, edges[i].dist / 65536.0);
#endif
    }
  }
  off_g = 0; mb_g = 0;
  if (n_paired > 0) { phase_offset(paired_pk, n_paired, &off_g, &mb_g, allow_half); }
  step = allow_half ? STEP_MDEG : 36000;

  nk = 0;
  for (i = 0; i < n1; i++)
  {
    feat_t *f = &stage1[i];
    int ink_ok = (f->len >= PTR_RIM_MIN_INK) && (f->contrast >= s_contrast_floor);
    int aligned = 0, near, rb0, keep = 0;
    if (is_paired[i]) { aligned = 1; }
    else if (n_paired > 0)
    {
      /* nearest grid line of the paired phase */
      int32_t g, best = 0x7fffffff;
      int k, nslot = allow_half ? 20 : 10;
      for (k = 0; k < nslot; k++)
      {
        int32_t d = iabs32(angdiff_mdeg(f->a, wrap_mdeg(off_g * 1000 + k * step)));
        if (d < best) { best = d; g = off_g * 1000 + k * step; }
      }
      (void)g;
      aligned = (best <= SNAP_MAX_MDEG);
    }
    near = (f->dptr <= PTR_NEAR_MDEG);
    rb0 = (f->rb_gap == 0);
    if (near && ink_ok && aligned)
    {
#ifdef DIAL_HOST_DEBUG
      printf("  keep a=%d ptr-cover (dptr=%.1f len=%d ctr=%d rb=%d aligned=1)\n",
             f->a / 1000, f->dptr / 1000.0, f->len, f->contrast, f->rb_gap);
#endif
      keep = 1;
    }
    else if (rb0 && !(ink_ok && aligned))
    {
#ifdef DIAL_HOST_DEBUG
      printf("  drop a=%d h=%d len=%d dptr=%.1f rb=%d: red-black shadow\n",
             f->a / 1000, (int)f->h, f->len, f->dptr / 1000.0, f->rb_gap);
#endif
    }
    else if (near && !aligned)
    {
#ifdef DIAL_HOST_DEBUG
      printf("  drop a=%d h=%d len=%d dptr=%.1f: ptr-offset shadow\n",
             f->a / 1000, (int)f->h, f->len, f->dptr / 1000.0);
#endif
    }
    else if (near && !ink_ok)
    {
#ifdef DIAL_HOST_DEBUG
      printf("  drop a=%d h=%d len=%d dptr=%.1f: ptr-cover weak ink\n",
             f->a / 1000, (int)f->h, f->len, f->dptr / 1000.0);
#endif
    }
    else if (is_paired[i] || allow_half)
    {
      keep = 1;
    }
    else if (aligned)
    {
#ifdef DIAL_HOST_DEBUG
      printf("  keep a=%d unpaired but on major grid (partner hidden?)\n", f->a / 1000);
#endif
      keep = 1;
    }
#ifdef DIAL_HOST_DEBUG
    else
    {
      printf("  drop a=%d h=%d: no 0-5 style pair\n", f->a / 1000, (int)f->h);
    }
#endif
    if (keep) { kept[nk++] = *f; }
  }
  for (i = 0; i < nk; i++) { pk[i].a = kept[i].a; pk[i].h = kept[i].h; }
  return nk;
}

/* True if a 补缺 candidate is real ink (or a pointer-covered printed tick). */
static int completion_ok(int32_t cx, int32_t cy, int32_t R, int32_t a_mdeg, int allow_half)
{
  feat_t ft;
  int gmax = allow_half ? TICK_GAP_HALF : TICK_GAP_MAJOR;
  int ink_ok, rb;
  if (tick_ink_features(cx, cy, R, a_mdeg, gmax, &ft) != 0) { return 0; }
  ink_ok = (ft.len >= PTR_RIM_MIN_INK) && (ft.contrast >= s_contrast_floor);
  if (!ink_ok) { return 0; }
  rb = red_black_gap(cx, cy, R, a_mdeg);
  if ((rb == 0) && !ink_ok) { return 0; }
  return 1;
}


typedef struct
{
  int32_t tick[N_TICK];
  uint8_t vis[N_TICK];
  uint8_t keep[N_TICK];
  int major_base;
  int n_vis;
  int n_tick;        /* 20 if half ticks allowed, else 10                  */
  int allow_half;    /* 1 only on calibration place 3 (x0.0001)            */
} ticks_t;

/* allow_half: only x0.0001 has half ticks (20 slots / 18 deg).
 * Other dials: 10 majors on a 36 deg grid; mid-sector peaks (~18 deg away)
 * fail SNAP_MAX (7 deg) and are discarded as noise. */
static void build_ticks(const int16_t *hist, const peak_t *pk, int n, int off, int major_base, int allow_half, ticks_t *T)
{
  peak_t pr[64], kept[64];
  int jj[64];
  int32_t dd[64];
  int32_t grid[N_TICK];
  int32_t asg[N_TICK];
  uint8_t used[N_TICK];
  int i, k, j, nk = 0;
  int n_tick = allow_half ? 20 : 10;
  int32_t step = allow_half ? STEP_MDEG : 36000;
  int half;
  memset(T, 0, sizeof(*T));
  T->n_tick = n_tick;
  T->allow_half = allow_half;
  T->major_base = allow_half ? major_base : 0;
  if (n > 64) { n = 64; }
  for (k = 0; k < n_tick; k++) { grid[k] = wrap_mdeg(off * 1000 + k * step); asg[k] = -1; used[k] = 0U; }
  for (i = 0; i < n; i++) { pr[i].a = refine_peak(hist, pk[i].a); pr[i].h = pk[i].h; }
  /* isolate_drop against this dial's grid */
  for (i = 0; i < n; i++)
  {
    int32_t bd = 0x7fffffff;
    jj[i] = 0;
    for (k = 0; k < n_tick; k++)
    {
      int32_t d = angdiff_mdeg(pr[i].a, grid[k]);
      if (iabs32(d) < bd) { bd = iabs32(d); jj[i] = k; dd[i] = d; }
    }
  }
  half = n_tick / 2;
  for (i = 0; i < n; i++)
  {
    int share = 0;
    if (iabs32(dd[i]) <= ISOLATE_MDEG) { kept[nk++] = pr[i]; continue; }
    for (j = 0; j < n; j++)
    {
      int sd;
      if (j == i) { continue; }
      sd = (((jj[j] - jj[i] + half) % n_tick) + n_tick) % n_tick - half;
      if ((iabs32(sd) <= 1) && (iabs32(dd[j] - dd[i]) < 1500) && (iabs32(dd[j]) > (ISOLATE_MDEG * 6) / 10)) { share = 1; break; }
    }
    if (share) { kept[nk++] = pr[i]; }
  }
  sort_by_h_stable(kept, nk);
  for (i = 0; i < nk; i++)
  {
    int32_t bd = 0x7fffffff;
    int bk = 0;
    for (k = 0; k < n_tick; k++)
    {
      int32_t d = used[k] ? 0x7fffffff : iabs32(angdiff_mdeg(kept[i].a, grid[k]));
      if (d < bd) { bd = d; bk = k; }
    }
    if (bd <= SNAP_MAX_MDEG) { asg[bk] = kept[i].a; used[bk] = 1U; }
  }
  T->n_vis = 0;
  for (k = 0; k < n_tick; k++)
  {
    int pk_ = -1, nx = -1, d;
    if (asg[k] >= 0) { T->tick[k] = asg[k]; T->vis[k] = 1U; T->n_vis++; continue; }
    T->vis[k] = 0U;
    for (d = 1; d < n_tick; d++) { if (asg[(k - d + n_tick) % n_tick] >= 0) { pk_ = (k - d + n_tick) % n_tick; break; } }
    for (d = 1; d < n_tick; d++) { if (asg[(k + d) % n_tick] >= 0) { nx = (k + d) % n_tick; break; } }
    if ((pk_ >= 0) && (nx >= 0) && (pk_ != nx))
    {
      int span = (nx - pk_ + n_tick) % n_tick, t = (k - pk_ + n_tick) % n_tick;
      int32_t fwd = wrap_mdeg(asg[nx] - asg[pk_]);
      T->tick[k] = wrap_mdeg(asg[pk_] + (int32_t)(((int64_t)fwd * t + span / 2) / span));
    }
    else
    {
      T->tick[k] = grid[k];
    }
  }
}

/* count dark pixels along a tick line (for snapping completed ticks) */
static void dark_count(int32_t cx, int32_t cy, int32_t R, int32_t a_mdeg, int len_pct, int *pn, int *ptot)
{
  int32_t c = cos_mdeg(a_mdeg), s = sin_mdeg(a_mdeg);
  int32_t r, r0 = (int32_t)(((int64_t)R * (100 - len_pct)) / 25600), r1 = (R >> 8) - 2;
  int n = 0, tot = 0;
  for (r = r0; r < r1; r++)
  {
    int32_t px = ray_q8(cx, r * 256, c), py = ray_q8(cy, r * 256, s);
    int v = L_at(px, py);
    if (v < 0) { continue; }
    if (red_at(px, py)) { continue; }   /* pointer is not printed tick ink */
    tot++;
    if (v <= s_dark_l) { n++; }
  }
  *pn = n; *ptot = tot;
}

static void snap_completed(int32_t cx, int32_t cy, int32_t R, ticks_t *T)
{
  int k, i, n_tick = T->n_tick ? T->n_tick : N_TICK;
  for (k = 0; k < n_tick; k++)
  {
    int bn = -1, bi = 0, found = 0;
    int32_t ba = 0;
    int is_maj = (!T->allow_half || ((k % 2) == T->major_base));
    int len = is_maj ? 16 : 10;
    T->keep[k] = 1U;
    if (T->vis[k]) { continue; }
    for (i = -8; i <= 8; i++)
    {
      int n, tot;
      int32_t a = T->tick[k] + 500 * i;
      dark_count(cx, cy, R, a, len, &n, &tot);
      if (tot && (10 * n >= 6 * tot))
      {
        if (!found || (n > bn) || ((n == bn) && (-iabs32(i) > -iabs32(bi))))
        {
          found = 1; bn = n; bi = i; ba = a;
        }
      }
    }
    if (!found) { T->keep[k] = 0U; }
    else if (!completion_ok(cx, cy, R, wrap_mdeg(ba), T->allow_half)) { T->keep[k] = 0U; }
    else { T->tick[k] = wrap_mdeg(ba); }
  }
}

/* ---------------- step 6: pointer axis ---------------- */

/* Longest circular run of set degrees in hit[360]. */
static void longest_arc360(const uint8_t *hit, int *start, int *width)
{
  int i, run = 0, best = 0, best_s = 0, s = 0, on = 0;
  for (i = 0; i < 720; i++)
  {
    if (hit[i % 360] != 0U)
    {
      if (on == 0) { s = i % 360; run = 0; on = 1; }
      run++;
      if (run > best) { best = run; best_s = s; }
    }
    else { on = 0; run = 0; }
  }
  if (best > 360) { best = 360; }
  *start = best_s;
  *width = best;
}

/* Tip along the pointer centerline (outer, thinner half of the blade).
 * Black rim / tick ink is not walked: only this pointer's red label.
 * A hooked shadow at the rim is ignored because the angle comes from
 * the mid-arc of many rings, not from a single farthest pixel. */

/* Walk this label along angle ang (mdeg) from r0..r1 (px from ax,ay). */
static int walk_red_ray(uint16_t id, int32_t ax, int32_t ay, int32_t ang,
                        int32_t r0, int32_t r1, int32_t *last_r, int32_t *last_x, int32_t *last_y)
{
  int32_t rr, lr = -1, lx = 0, ly = 0;
  for (rr = r0; rr <= r1; rr++)
  {
    int32_t x = ax + (int32_t)(((int64_t)rr * cos_mdeg(ang)) / 65536);
    int32_t y = ay + (int32_t)(((int64_t)rr * sin_mdeg(ang)) / 65536);
    if ((x < s_x0) || (y < s_y0) || (x >= s_x1) || (y >= s_y1)) { break; }
    if (s_lab[(uint32_t)y * s_w + (uint32_t)x] == id)
    {
      lr = rr; lx = x; ly = y;
    }
    else if ((lr >= 0) && (rr - lr > 4)) { break; }
  }
  *last_r = lr;
  *last_x = lx;
  *last_y = ly;
  return (lr >= 0) ? 0 : -1;
}

static uint8_t s_tip_why;           /* pointer_tip / blade detail for g_dial_dbg */
volatile dial_dbg_t g_dial_dbg;

static int blade_midline_tip(uint16_t id, int32_t ax, int32_t ay, int32_t ar,
                             int32_t *tx, int32_t *ty)
{
  int32_t rr, r0, r1, nmid = 0;
  int32_t mid_a[160], mid_w[160];
  int32_t i, tail0, sc = 0, ss = 0, ang, last_r = -1, last_x = 0, last_y = 0;
  if (ar < 6) { s_tip_why = 2U; return -1; }
  r0 = ar + 6;
  r1 = (ar * 18) / 5 + 40;   /* 3.6 * ar + 40 */
  for (rr = r0; (rr <= r1) && (nmid < 160); rr++)
  {
    uint8_t hit[360];
    int a, nh = 0, border = 0, start = 0, width = 0;
    for (a = 0; a < 360; a++) { hit[a] = 0U; }
    for (a = 0; a < 360; a++)
    {
      int32_t x = ax + (int32_t)(((int64_t)rr * cos_deg(a)) / 65536);
      int32_t y = ay + (int32_t)(((int64_t)rr * sin_deg(a)) / 65536);
      if ((x < s_x0) || (y < s_y0) || (x >= s_x1) || (y >= s_y1)) { continue; }
      if (s_lab[(uint32_t)y * s_w + (uint32_t)x] != id) { continue; }
      if ((x <= 1) || (y <= 1) || (x >= (int32_t)s_w - 2) || (y >= (int32_t)s_h - 2))
      {
        border = 1; break;
      }
      hit[a] = 1U; nh++;
    }
    if (border) { continue; }
    if (nh < 3)
    {
      if ((nmid > 0) && (rr - (r0 + nmid) > 8) && (nmid >= 4)) { break; }
      continue;
    }
    longest_arc360(hit, &start, &width);
    if ((width < 3) || (width > 160)) { continue; }
    mid_a[nmid] = (start * 1000 + (width * 1000) / 2) % 360000;
    mid_w[nmid] = width;
    nmid++;
  }
  if (nmid < 4) { s_tip_why = 3U; return -1; }
  tail0 = nmid / 3;
  for (i = tail0; i < nmid; i++)
  {
    sc += cos_mdeg(mid_a[i]) / 256;   /* Q8 accum, enough for ~100 rings */
    ss += sin_mdeg(mid_a[i]) / 256;
  }
  if ((sc == 0) && (ss == 0)) { s_tip_why = 4U; return -1; }
  ang = atan2_mdeg((int64_t)ss, (int64_t)sc);
  r0 = ar + 2;
  r1 = ar * 4 + 60;
  /* Try both directions: a wrong mid-arc can point at the hub's short side. */
  {
    int32_t rA = -1, xA = 0, yA = 0, rB = -1, xB = 0, yB = 0;
    int32_t best_a = ang, d, rD, xD, yD;
    (void)walk_red_ray(id, ax, ay, ang, r0, r1, &rA, &xA, &yA);
    (void)walk_red_ray(id, ax, ay, ang + 180000, r0, r1, &rB, &xB, &yB);
    if ((rB > rA) && (rB >= 0))
    {
      ang = ang + 180000;
      last_r = rB; last_x = xB; last_y = yB;
    }
    else
    {
      last_r = rA; last_x = xA; last_y = yA;
    }
    if ((last_r < 0) || (last_r < ar * 2)) { s_tip_why = 5U; return -1; }
    /* Snap to the ray with the longest red run, +/- 8 deg. */
    best_a = ang;
    for (d = -8; d <= 8; d++)
    {
      if (d == 0) { continue; }
      if (walk_red_ray(id, ax, ay, ang + d * 1000, r0, r1, &rD, &xD, &yD) != 0) { continue; }
      if (rD > last_r)
      {
        last_r = rD; last_x = xD; last_y = yD; best_a = ang + d * 1000;
      }
    }
    (void)best_a;
  }
  *tx = last_x * 256;
  *ty = last_y * 256;
  return 0;
}

static int pointer_tip(uint16_t id, int32_t *tx, int32_t *ty)
{
  int32_t w = s_x1 - s_x0, h = s_y1 - s_y0, ax, ay, x, y;
  uint16_t dtm;
  int64_t n = 0, sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, cnt = 0;
  int64_t lim;
  s_tip_why = 0U;
  for (y = s_y0; y < s_y1; y++)
  {
    dial_yield();
    for (x = s_x0; x < s_x1; x++) { if (s_lab[(uint32_t)y * s_w + (uint32_t)x] == id) { cnt++; } }
  }
  if (cnt < 40) { s_tip_why = 1U; return -1; }
  dtm = chamfer(s_x0, s_y0, w, h, mask_lab, id, &ax, &ay);
  lim = (int64_t)(dtm + 9U) * (dtm + 9U);      /* (ar+3)^2 * 9 */
  for (y = s_y0; y < s_y1; y++)
  {
    dial_yield();
    for (x = s_x0; x < s_x1; x++)
    {
      int64_t dx = x - ax, dy = y - ay;
      if (s_lab[(uint32_t)y * s_w + (uint32_t)x] != id) { continue; }
      if (9 * (dx * dx + dy * dy) <= lim) { continue; }
      n++; sx += x; sy += y; sxx += (int64_t)x * x; syy += (int64_t)y * y; sxy += (int64_t)x * y;
    }
  }
  if (0)   /* mass-axis projection pulled the tip off the blade; use the sharp end */
  {
    int64_t a = n * sxx - sx * sx, b = n * syy - sy * sy, c = n * sxy - sx * sy;
    int64_t p, q, s, v1x, v1y, v2x, v2y, vx, vy, mx, my, tmax = INT64_MIN, nv2;
    int m = bits_needed(a), sh;
    if (bits_needed(b) > m) { m = bits_needed(b); }
    if (bits_needed(c) > m) { m = bits_needed(c); }
    sh = (m > 28) ? (m - 28) : 0;
    a >>= sh; b >>= sh; c >>= sh;
    p = a - b; q = 2 * c;
    s = (int64_t)isqrt64((uint64_t)(p * p + q * q));
    v1x = p + s; v1y = q; v2x = q; v2y = s - p;
    if (iabs64(v1x) + iabs64(v1y) >= iabs64(v2x) + iabs64(v2y)) { vx = v1x; vy = v1y; } else { vx = v2x; vy = v2y; }
    if ((vx == 0) && (vy == 0)) { vx = 1; }
    while ((iabs64(vx) > 16384) || (iabs64(vy) > 16384)) { vx /= 2; vy /= 2; }
    mx = (sx * 256) / n; my = (sy * 256) / n;
    if (((mx - (int64_t)ax * 256) * vx + (my - (int64_t)ay * 256) * vy) < 0) { vx = -vx; vy = -vy; }
    for (y = s_y0; y < s_y1; y++)
    {
      dial_yield();
      for (x = s_x0; x < s_x1; x++)
      {
        int64_t dx = x - ax, dy = y - ay, pr;
        if (s_lab[(uint32_t)y * s_w + (uint32_t)x] != id) { continue; }
        if (9 * (dx * dx + dy * dy) <= lim) { continue; }
        pr = ((int64_t)x * 256 - mx) * vx + ((int64_t)y * 256 - my) * vy;
        if (pr > tmax) { tmax = pr; }
      }
    }
    nv2 = vx * vx + vy * vy;
    *tx = (int32_t)(mx + (vx * tmax) / nv2);
    *ty = (int32_t)(my + (vy * tmax) / nv2);
    return 0;
  }
  /* Prefer the blade centerline (ignores a hooked black-rim shadow). */
  {
    int32_t ar = (int32_t)(dtm / 3U);
    if (ar < 8) { ar = 8; }
    if (blade_midline_tip(id, ax, ay, ar, tx, ty) == 0) { return 0; }
  }
  /* Fallback: farthest pointer pixel from the hub. */
  {
    int64_t bd = -1;
    for (y = s_y0; y < s_y1; y++)
    {
      dial_yield();
      for (x = s_x0; x < s_x1; x++)
      {
        int64_t dx = x - ax, dy = y - ay;
        if (s_lab[(uint32_t)y * s_w + (uint32_t)x] != id) { continue; }
        if (dx * dx + dy * dy > bd) { bd = dx * dx + dy * dy; *tx = x * 256; *ty = y * 256; }
      }
    }
    if (bd >= 0) { return 0; }
    if (s_tip_why == 0U) { s_tip_why = 6U; }
    return -1;
  }
}

/* ---------------- step 7: reading between printed main ticks ---------------- */
static int32_t read_value(int32_t pang, const int32_t *maj)
{
  int k, bk = 0;
  int32_t bd = 0x7fffffff;
  for (k = 0; k < 10; k++)
  {
    int32_t a0 = maj[k], a1 = maj[(k + 1) % 10];
    int32_t span = wrap_mdeg(a1 - a0), t = wrap_mdeg(pang - a0);
    if (span <= 0) { continue; }
    if (t <= span) { return k * 10000 + (int32_t)(((int64_t)t * 10000 + span / 2) / span); }
  }
  for (k = 0; k < 10; k++)
  {
    int32_t d = iabs32(angdiff_mdeg(pang, maj[k]));
    if (d < bd) { bd = d; bk = k; }
  }
  return bk * 10000;
}

/* Rebuild major 5 from the 4/6 bisector and the 0+180 axis when the
 * printed 5 is pointer-covered and its peak has drifted. */
static void anchor_majors_4_6_0(int32_t *maj)
{
  int32_t a0 = maj[0], a4 = maj[4], a5 = maj[5], a6 = maj[6];
  int32_t span46 = wrap_mdeg(a6 - a4);
  int32_t bis, opp0, recon;
  if ((span46 < 50000) || (span46 > 100000)) { return; }
  bis = wrap_mdeg(a4 + span46 / 2);
  opp0 = wrap_mdeg(a0 + 180000);
  if (iabs32(angdiff_mdeg(bis, opp0)) > 8000) { return; }
  recon = wrap_mdeg(bis + angdiff_mdeg(opp0, bis) / 2);
  if (iabs32(angdiff_mdeg(a5, recon)) > 1500) { maj[5] = recon; }
}

/* ---------------- text ---------------- */
static const dial_glyph_t *glyph_of(uint8_t ch)
{
  uint32_t i;
  for (i = 0U; i < DIAL_FONT_NGLYPH; i++) { if (dial_font_glyphs[i].code == ch) { return &dial_font_glyphs[i]; } }
  return 0;
}

static void blend(uint16_t *px, int r, int g, int b, int a)
{
  int r0, g0, b0;
  px8(*px, &r0, &g0, &b0);
  r0 = (r * a + r0 * (15 - a) + 7) / 15;
  g0 = (g * a + g0 * (15 - a) + 7) / 15;
  b0 = (b * a + b0 * (15 - a) + 7) / 15;
  *px = pack565(r0, g0, b0);
}

static void draw_str(uint16_t *fb, int x, int y, const char *s, int r, int g, int b)
{
  int32_t pen = x * 64;
  for (; *s; s++)
  {
    const dial_glyph_t *gl = glyph_of((uint8_t)*s);
    int gx, gy, i, j;
    if (!gl) { continue; }
    gx = (int)((pen + 32) >> 6) + gl->xoff;
    gy = y + gl->yoff;
    for (j = 0; j < gl->h; j++)
    {
      for (i = 0; i < gl->w; i++)
      {
        int idx = j * gl->w + i, a = (gl->bits[idx >> 1] >> ((idx & 1) * 4)) & 15;
        int px = gx + i, py = gy + j;
        if (!a || (px < 0) || (py < 0) || (px >= s_w) || (py >= s_h)) { continue; }
        blend(&fb[(uint32_t)py * s_w + (uint32_t)px], r, g, b, a);
      }
    }
    pen += gl->adv64;
  }
}

/* red text with black outline (8 neighbours, 2 px). Used for dial digits. */
static void draw_text(uint16_t *fb, int x, int y, const char *s)
{
  static const int8_t o[8][2] = { {-2, 0}, {2, 0}, {0, -2}, {0, 2}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1} };
  int i;
  for (i = 0; i < 8; i++) { draw_str(fb, x + o[i][0], y + o[i][1], s, 0, 0, 0); }
  draw_str(fb, x, y, s, TXT_R, TXT_G, TXT_B);
}

static void fill_rect(uint16_t *fb, int x0, int y0, int x1, int y1, uint16_t color)
{
  int x, y;
  if (x0 < 0) { x0 = 0; }
  if (y0 < 0) { y0 = 0; }
  if (x1 > (int)s_w) { x1 = (int)s_w; }
  if (y1 > (int)s_h) { y1 = (int)s_h; }
  for (y = y0; y < y1; y++)
  {
    uint16_t *row = fb + ((uint32_t)y * (uint32_t)s_w);
    for (x = x0; x < x1; x++) { row[x] = color; }
  }
}

/* Tight ink bbox of a string drawn at (x, y). Space-only glyphs are skipped. */
static void text_ink_box(const char *s, int x, int y, int *x0, int *y0, int *x1, int *y1)
{
  int32_t pen = x * 64;
  int left = 0x7fff, top = 0x7fff, right = -0x7fff, bot = -0x7fff;

  for (; *s != 0; s++)
  {
    const dial_glyph_t *gl = glyph_of((uint8_t)*s);
    int gx, gy;
    if ((gl == 0) || (gl->h == 0) || (gl->w == 0))
    {
      if (gl != 0) { pen += gl->adv64; }
      continue;
    }
    gx = (int)((pen + 32) >> 6) + gl->xoff;
    gy = y + gl->yoff;
    if (gx < left) { left = gx; }
    if (gy < top) { top = gy; }
    if ((gx + gl->w) > right) { right = gx + gl->w; }
    if ((gy + gl->h) > bot) { bot = gy + gl->h; }
    pen += gl->adv64;
  }
  if (left == 0x7fff)
  {
    left = x;
    top = y;
    right = x;
    bot = y + DIAL_FONT_LINE_H;
  }
  *x0 = left;
  *y0 = top;
  *x1 = right;
  *y1 = bot;
}


/* Map a point from the rectified / unwrapped analysis frame back onto the
 * live captured framebuffer (inverse rectify, then re-apply the seam). */
static void inv_rectify_q8(int32_t x_q8, int32_t y_q8, int32_t *sx_q8, int32_t *sy_q8)
{
#if DIAL_RECTIFY_ON
  int64_t X = ((int64_t)HI11 * x_q8 + (int64_t)HI12 * y_q8 + 128) >> 8;
  int64_t Y = ((int64_t)HI21 * x_q8 + (int64_t)HI22 * y_q8 + 128) >> 8;
  int64_t D = ((int64_t)HI31 * x_q8 + (int64_t)HI32 * y_q8 + 128) >> 8;
  int64_t e30, e2, inv;
  X += HI13;
  Y += HI23;
  D += ((int64_t)1 << 32);
  e30 = (((int64_t)1 << 32) - D) >> 2;
  e2 = (e30 * e30) >> 30;
  inv = ((int64_t)1 << 30) + e30 + e2 + ((e2 * e30) >> 30);
  *sx_q8 = (int32_t)((X * inv) >> 42);
  *sy_q8 = (int32_t)((Y * inv) >> 42);
#else
  *sx_q8 = x_q8;
  *sy_q8 = y_q8;
#endif
}

static void live_plot(uint16_t *fb, int x, int y, uint16_t col,
                     int w, int h, int seam, uint8_t mode)
{
  int32_t ww = (int32_t)w, hh = (int32_t)h, xx, yy;
  if ((ww <= 0) || (hh <= 0)) { return; }
  if (seam > 0)
  {
    if (mode == 1U)
    {
      int32_t n = ww * hh;
      int32_t i = y * ww + x + seam;
      i %= n;
      if (i < 0) { i += n; }
      xx = i % ww;
      yy = i / ww;
    }
    else
    {
      xx = x + seam;
      xx %= ww;
      if (xx < 0) { xx += ww; }
      yy = y;
    }
  }
  else
  {
    xx = x;
    yy = y;
  }
  if ((xx >= 0) && (yy >= 0) && (xx < ww) && (yy < hh))
  {
    fb[(uint32_t)yy * (uint32_t)ww + (uint32_t)xx] = col;
  }
}

static void live_plot_w(uint16_t *fb, int x, int y, uint16_t col, int wid,
                       int w, int h, int seam, uint8_t mode)
{
  int t, u, hw;
  if (wid < 1) { wid = 1; }
  hw = wid / 2;
  for (t = -hw; t <= hw; t++)
  {
    for (u = -hw; u <= hw; u++)
    {
      if ((t * t + u * u) <= (hw * hw + 1))
      {
        live_plot(fb, x + t, y + u, col, w, h, seam, mode);
      }
    }
  }
}

static void live_line(uint16_t *fb, int x0, int y0, int x1, int y1, uint16_t col, int wid,
                     int w, int h, int seam, uint8_t mode)
{
  int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1), sx = (x0 < x1) ? 1 : -1;
  int dy = (y1 > y0) ? (y0 - y1) : (y1 - y0), sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy, e2;
  for (;;)
  {
    live_plot_w(fb, x0, y0, col, wid, w, h, seam, mode);
    if ((x0 == x1) && (y0 == y1)) { break; }
    e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

#define LIVE_CIRC  0x07E0U  /* green recognised rim */
#define LIVE_PTR   0x029FU  /* blue centre-to-tip, same as analysis overlay */

static void paint_live_dials(uint16_t *fb, uint16_t w, uint16_t h, const dial_report_t *rep)
{
  int i, a;
  int seam = (int)rep->seam_col;
  uint8_t mode = rep->seam_mode;
  if ((fb == 0) || (rep == 0) || (rep->n_dials == 0U)) { return; }
  for (i = 0; i < (int)rep->n_dials; i++)
  {
    const dial_info_t *d = &rep->dial[i];
    int32_t sx, sy, tx, ty, px, py, rx, ry;
    int x0, y0, x1, y1;
    int32_t c, s;
    if (d->r_q8 < (20 * 256)) { continue; }
    inv_rectify_q8(d->cx_q8, d->cy_q8, &sx, &sy);
    inv_rectify_q8(d->tip_x_q8, d->tip_y_q8, &tx, &ty);
    x0 = (int)((sx + 128) >> 8);
    y0 = (int)((sy + 128) >> 8);
    x1 = (int)((tx + 128) >> 8);
    y1 = (int)((ty + 128) >> 8);
    for (a = 0; a < 360; a++)
    {
      c = cos_deg(a);
      s = sin_deg(a);
      px = ray_q8(d->cx_q8, d->r_q8, c);
      py = ray_q8(d->cy_q8, d->r_q8, s);
      inv_rectify_q8(px, py, &rx, &ry);
      live_plot_w(fb, (int)((rx + 128) >> 8), (int)((ry + 128) >> 8),
                  LIVE_CIRC, 3, (int)w, (int)h, seam, mode);
    }
    /* ticks: recognised length, from inside the face to the rim (not past it) */
    {
      const dial_ovl_t *ov = &rep->ovl[i];
      int k, nt = (int)ov->n_tick;
      for (k = 0; k < nt; k++)
      {
        int is_maj, tw;
        int32_t rin, rout, ix, iy, ox, oy, irx, iry, orx, ory;
        if (ov->keep[k] == 0U) { continue; }
        is_maj = ((ov->allow_half == 0U) || ((k % 2) == (int)ov->major_base)) ? 1 : 0;
        rin = (int32_t)(((int64_t)d->r_q8 * (is_maj ? 84 : 90)) / 100);
        rout = d->r_q8;
        c = cos_mdeg(ov->tick_mdeg[k]);
        s = sin_mdeg(ov->tick_mdeg[k]);
        ix = ray_q8(d->cx_q8, rin, c);
        iy = ray_q8(d->cy_q8, rin, s);
        ox = ray_q8(d->cx_q8, rout, c);
        oy = ray_q8(d->cy_q8, rout, s);
        inv_rectify_q8(ix, iy, &irx, &iry);
        inv_rectify_q8(ox, oy, &orx, &ory);
        tw = is_maj ? 3 : 2;
        live_line(fb,
                  (int)((irx + 128) >> 8), (int)((iry + 128) >> 8),
                  (int)((orx + 128) >> 8), (int)((ory + 128) >> 8),
                  LIVE_CIRC, tw, (int)w, (int)h, seam, mode);
      }
    }
    /* Centre through the tip, ending on the rim so alignment is easy to check. */
    live_plot_w(fb, x0, y0, LIVE_PTR, 7, (int)w, (int)h, seam, mode);
    live_plot_w(fb, x1, y1, LIVE_PTR, 5, (int)w, (int)h, seam, mode);
    {
      int64_t tdx = (int64_t)d->tip_x_q8 - d->cx_q8;
      int64_t tdy = (int64_t)d->tip_y_q8 - d->cy_q8;
      int32_t tlen = (int32_t)isqrt64((uint64_t)(tdx * tdx + tdy * tdy));
      int32_t rx1, ry1, qx, qy;
      if (tlen >= (20 * 256))
      {
        rx1 = (int32_t)(d->cx_q8 + (tdx * (int64_t)d->r_q8) / tlen);
        ry1 = (int32_t)(d->cy_q8 + (tdy * (int64_t)d->r_q8) / tlen);
        inv_rectify_q8(rx1, ry1, &qx, &qy);
        x1 = (int)((qx + 128) >> 8);
        y1 = (int)((qy + 128) >> 8);
        live_line(fb, x0, y0, x1, y1, LIVE_PTR, 3, (int)w, (int)h, seam, mode);
      }
    }
  }
}

void DialRead_PaintHud(uint16_t *fb, uint16_t w, uint16_t h, const dial_report_t *rep)
{
  uint16_t ow = s_w, oh = s_h;
  int tx0, ty0, tx1, ty1, x0, y0, x1, y1;
  int has2;
  const int pad_x = 8;
  const int pad_y = 8;
  const int hx = 12;
  const int hy1 = 8;
  const int hy2 = 44;

  if ((fb == 0) || (rep == 0) || (w < 16U) || (h < 16U)) { return; }
  s_w = w;
  s_h = h;
  if (rep->line1[0] == 0)
  {
    s_w = ow;
    s_h = oh;
    return;
  }
  /* dial circles + centre-to-tip lines first, so the text bar stays on top */
  paint_live_dials(fb, w, h, rep);
  has2 = ((rep->status == DIAL_OK) && (rep->line2[0] != 0)) ? 1 : 0;
  text_ink_box(rep->line1, hx, hy1, &tx0, &ty0, &tx1, &ty1);
  if (has2 != 0)
  {
    int ax0, ay0, ax1, ay1;
    text_ink_box(rep->line2, hx, hy2, &ax0, &ay0, &ax1, &ay1);
    if (ax0 < tx0) { tx0 = ax0; }
    if (ay0 < ty0) { ty0 = ay0; }
    if (ax1 > tx1) { tx1 = ax1; }
    if (ay1 > ty1) { ty1 = ay1; }
  }
  x0 = tx0 - pad_x;
  y0 = ty0 - pad_y;
  x1 = tx1 + pad_x;
  y1 = ty1 + pad_y;
  if (x0 < 0) { x0 = 0; }
  if (y0 < 0) { y0 = 0; }
  /* solid white panel; padding is equal above and below the ink so the
   * glyphs sit vertically centered in the bar */
  fill_rect(fb, x0, y0, x1, y1, 0xFFFF);
  draw_str(fb, hx, hy1, rep->line1, TXT_R, TXT_G, TXT_B);
  if (has2 != 0) { draw_str(fb, hx, hy2, rep->line2, TXT_R, TXT_G, TXT_B); }
  s_w = ow;
  s_h = oh;
}

static char *put_n(char *p, uint32_t v, int digits)
{
  int i;
  for (i = digits - 1; i >= 0; i--) { p[i] = (char)('0' + (v % 10U)); v /= 10U; }
  return p + digits;
}

/* ---------------- green completed ticks (dark pixels only) ---------------- */
typedef struct
{
  int32_t cx, cy, R;
  int32_t x0, y0, x1, y1;
  int32_t tipx, tipy;
  ticks_t T;
  int32_t maj[10];          /* 10 majors, 4/6/0-anchored when applicable */
  int16_t hub_x, hub_y;
  uint8_t hub_r;
  int8_t  place;
  int     face_w, face_d;
  uint8_t arc_cov;
} dial_geo_t;


/* Overlay only: fill a short missing chip in the printed rim along the
 * fitted circle (弧度) with the neighbouring stroke width (间距). Does
 * not change the reading. */
#define RIM_STEP_MDEG   250     /* 0.25 deg, 1440 samples */
#define RIM_NANG        1440
#define RIM_DARK        140
#define RIM_GAP         150
#define RIM_MAX_DEG     8
#define RIM_MIN_DEG     1       /* 0.5 deg in 0.25 steps = 2 samples; use >= 0.5 */
static void paint_rim_gaps(uint16_t *fb, const dial_geo_t *g)
{
  static int16_t strength[RIM_NANG];
  static uint8_t is_red_a[RIM_NANG];
  int i, k, n = RIM_NANG;
  int32_t cx = g->cx, cy = g->cy, R = g->R;
  /* per-angle min luma near the rim, and whether the sample is mostly pointer */
  for (i = 0; i < n; i++)
  {
    int32_t a = i * RIM_STEP_MDEG;
    int32_t c = cos_mdeg(a), s = sin_mdeg(a);
    int vr = 255, nr = 0, nt = 0, d;
    for (d = -6; d <= 5; d++)          /* -3.0 .. +2.5 px in 0.5 px steps */
    {
      int32_t px = ray_q8(cx, R + d * 128, c), py = ray_q8(cy, R + d * 128, s);
      int v = L_at(px, py);
      if (v < 0) { continue; }
      if (v < vr) { vr = v; }
      nt++;
      if (red_at(px, py)) { nr++; }
    }
    strength[i] = (int16_t)vr;
    is_red_a[i] = (nt && (nr * 2 >= nt)) ? 1U : 0U;
  }
#define IS_GAP(i)  ((is_red_a[i] == 0) && (strength[i] > RIM_GAP))
#define IS_RIM(i)  ((is_red_a[i] == 0) && (strength[i] <= RIM_DARK))
  /* collect short bright runs with real rim on both sides */
  i = 0;
  while (i < n)
  {
    int j, deg_s, got_l, got_r, need, kk, jj;
    int ds[64], nds, d0, d1;
    int32_t a, c, s, px, py;
    int xi, yi, dx, dy;
    if (!IS_GAP(i)) { i++; continue; }
    j = i;
    while ((j < n) && IS_GAP(j)) { j++; }
    deg_s = j - i;                                   /* in 0.25 deg units */
    /* 0.5 .. 8.0 deg = 2 .. 32 samples */
    if ((deg_s < 2) || (deg_s > 32)) { i = j; continue; }
    need = 16;                                       /* 4 deg */
    got_l = got_r = 0;
    for (kk = 1; kk < need + 8; kk++)
    {
      jj = (i - kk + n) % n;
      if (is_red_a[jj]) { continue; }
      if (IS_RIM(jj)) { got_l++; }
      else if (IS_GAP(jj)) { break; }
    }
    for (kk = 1; kk < need + 8; kk++)
    {
      jj = (j - 1 + kk + n) % n;
      if (is_red_a[jj]) { continue; }
      if (IS_RIM(jj)) { got_r++; }
      else if (IS_GAP(jj)) { break; }
    }
    if ((got_l < need - 2) || (got_r < need - 2)) { i = j; continue; }
    /* stroke width from complete rim on both sides (~8 deg) */
    nds = 0;
    for (kk = -32; kk < 32; kk++)
    {
      if (kk < 0) { jj = (i + kk + n) % n; }
      else { jj = (j + kk) % n; }
      if (!IS_RIM(jj)) { continue; }
      a = jj * RIM_STEP_MDEG;
      c = cos_mdeg(a); s = sin_mdeg(a);
      for (d0 = -16; d0 <= 16; d0++)     /* -4 .. +4 px in 0.25 px */
      {
        int v = L_at(ray_q8(cx, R + d0 * 64, c), ray_q8(cy, R + d0 * 64, s));
        if ((v >= 0) && (v <= RIM_DARK) && (nds < 64)) { ds[nds++] = d0; }
      }
    }
    if (nds < 8) { d0 = -7; d1 = 7; }    /* ±1.75 px in 0.25 units */
    else
    {
      /* insertion sort then 10th / 90th percentile */
      int t, p, q;
      for (p = 1; p < nds; p++)
      {
        t = ds[p]; q = p - 1;
        while ((q >= 0) && (ds[q] > t)) { ds[q + 1] = ds[q]; q--; }
        ds[q + 1] = t;
      }
      d0 = ds[(nds * 10) / 100];
      d1 = ds[(nds * 90) / 100];
      if ((d1 - d0) < 10)
      {
        int mid = (d0 + d1) / 2;
        d0 = mid - 6; d1 = mid + 6;
      }
    }
#ifdef DIAL_HOST_DEBUG
    printf("  rim-gap fill %.1f-%.1f deg (%.1f, stroke %.1f px)\n",
           i * 0.25, j * 0.25, deg_s * 0.25, (d1 - d0) * 0.25);
#endif
    for (k = i; k < j; k++)
    {
      int d;
      a = k * RIM_STEP_MDEG;
      c = cos_mdeg(a); s = sin_mdeg(a);
      for (d = d0; d <= d1; d++)
      {
        px = ray_q8(cx, R + d * 64, c);
        py = ray_q8(cy, R + d * 64, s);
        xi = round_q8(px); yi = round_q8(py);
        for (dy = -1; dy <= 1; dy++)
        {
          for (dx = -1; dx <= 1; dx++)
          {
            int xx = xi + dx, yy = yi + dy;
            int64_t rx, ry, rr;
            uint32_t o;
            if ((xx < g->x0) || (yy < g->y0) || (xx >= g->x1) || (yy >= g->y1)) { continue; }
            o = (uint32_t)yy * s_w + (uint32_t)xx;
            if (s_red[o]) { continue; }
            rx = (int64_t)xx * 256 - cx; ry = (int64_t)yy * 256 - cy;
            rr = (int64_t)isqrt64((uint64_t)(rx * rx + ry * ry));
            if ((rr >= R + d0 * 64 - 154) && (rr <= R + d1 * 64 + 154))
            {
              fb[o] = 0x07E0U;
            }
          }
        }
      }
    }
    i = j;
  }
#undef IS_GAP
#undef IS_RIM
}

/* Blue line from printed-dial center through the tip, ending on the scale rim. */
#define PTR_BLUE  0x029FU   /* RGB565: about (0, 80, 255) */
static void draw_dot(uint16_t *fb, int x, int y, int r, uint16_t col)
{
  int dy, dx;
  for (dy = -r; dy <= r; dy++)
  {
    for (dx = -r; dx <= r; dx++)
    {
      int xx = x + dx, yy = y + dy;
      if ((dx * dx + dy * dy <= r * r) && (xx >= 0) && (yy >= 0) && (xx < (int)s_w) && (yy < (int)s_h))
      {
        fb[(uint32_t)yy * s_w + (uint32_t)xx] = col;
      }
    }
  }
}

static void draw_line_w(uint16_t *fb, int x0, int y0, int x1, int y1, uint16_t col, int wid)
{
  int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1), sx = (x0 < x1) ? 1 : -1;
  int dy = (y1 > y0) ? (y0 - y1) : (y1 - y0), sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy, e2, t;
  int w = (int)s_w, h = (int)s_h, hw = wid / 2;
  for (;;)
  {
    for (t = -hw; t <= hw; t++)
    {
      int xx = x0, yy = y0 + t;
      if ((xx >= 0) && (yy >= 0) && (xx < w) && (yy < h)) { fb[(uint32_t)yy * (uint32_t)w + (uint32_t)xx] = col; }
      xx = x0 + t; yy = y0;
      if ((xx >= 0) && (yy >= 0) && (xx < w) && (yy < h)) { fb[(uint32_t)yy * (uint32_t)w + (uint32_t)xx] = col; }
    }
    if ((x0 == x1) && (y0 == y1)) { break; }
    e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

static void paint_pointer_line(uint16_t *fb, const dial_geo_t *g)
{
  int64_t dx = (int64_t)g->tipx - g->cx, dy = (int64_t)g->tipy - g->cy;
  int32_t len, x1, y1;
  if (dx == 0 && dy == 0) { return; }
  len = (int32_t)isqrt64((uint64_t)(dx * dx + dy * dy));
  if (len < 256) { return; }
  x1 = (int32_t)(g->cx + (dx * (int64_t)g->R) / len);
  y1 = (int32_t)(g->cy + (dy * (int64_t)g->R) / len);
  draw_line_w(fb, (int)(g->cx >> 8), (int)(g->cy >> 8), (int)(x1 >> 8), (int)(y1 >> 8), PTR_BLUE, 3);
  draw_dot(fb, (int)(g->cx >> 8), (int)(g->cy >> 8), 3, PTR_BLUE);
}

static void paint_ticks(uint16_t *fb, const dial_geo_t *g)
{
  int32_t x, y;
  int64_t rin = ((int64_t)g->R * 988) / 1000, rout = ((int64_t)g->R * 1012) / 1000;
  int64_t tin = ((int64_t)g->R * 84) / 100, tout = ((int64_t)g->R * 10036) / 10000;
  int64_t rin2 = rin * rin, rout2 = rout * rout, tin2 = tin * tin, tout2 = tout * tout;
  int64_t min2[N_TICK];
  int k, n_tick = g->T.n_tick ? g->T.n_tick : N_TICK;
  for (k = 0; k < n_tick; k++)
  {
    int is_maj = (!g->T.allow_half || ((k % 2) == g->T.major_base));
    int64_t r0 = ((int64_t)g->R * (is_maj ? 84 : 90)) / 100;
    min2[k] = r0 * r0;
  }
  for (y = g->y0; y < g->y1; y++)
  {
    dial_yield();
    for (x = g->x0; x < g->x1; x++)
    {
      uint32_t o = (uint32_t)y * s_w + (uint32_t)x;
      int64_t dx = (int64_t)x * 256 - g->cx, dy = (int64_t)y * 256 - g->cy, d2 = dx * dx + dy * dy;
      int on = 0;
      if (!((s_L[o] <= s_dark_l) && !s_red[o])) { continue; }
      if ((d2 >= rin2) && (d2 <= rout2)) { on = 1; }
      else if ((d2 >= tin2) && (d2 <= tout2))
      {
        int32_t aa = atan2_mdeg(dy, dx);
        for (k = 0; k < n_tick; k++)
        {
          int is_maj = (!g->T.allow_half || ((k % 2) == g->T.major_base));
          int32_t hw = is_maj ? 800 : 600;
          if (!g->T.keep[k] || (d2 < min2[k])) { continue; }
          if (iabs32(angdiff_mdeg(aa, g->T.tick[k])) <= hw) { on = 1; break; }
        }
      }
      if (on) { fb[o] = 0x07E0U; }
    }
  }
  paint_rim_gaps(fb, g);
}

/* ---------------- hooks ---------------- */
/* Board port overrides this to present a live camera frame mid-read. */
DIAL_WEAK void dial_yield(void)
{
}

DIAL_WEAK void dial_get_time(dial_time_t *t)
{
  t->year = 2026U; t->mon = 1U; t->day = 1U; t->hour = 0U; t->min = 0U; t->sec = 0U; t->msec = 0U;
}

DIAL_WEAK int dial_save_suspect_frame(const uint16_t *frame, uint16_t w, uint16_t h, const char *name)
{
  (void)frame; (void)w; (void)h; (void)name;
  return -1;   /* no storage yet: implement on SD card / QSPI flash */
}

/* ---------------- calibration ---------------- */
void DialRead_SetOrderCal(const dial_cal_t *cal, uint8_t n)
{
  if (n > DIAL_MAX_FOUND) { n = DIAL_MAX_FOUND; }
  memcpy(s_cal, cal, (size_t)n * sizeof(dial_cal_t));
  s_ncal = n;
}

const dial_cal_t *DialRead_GetOrderCal(uint8_t *n)
{
  *n = s_ncal;
  return s_cal;
}

static int match_cal(int32_t cx_q8, int32_t cy_q8)
{
  int i, best = -1;
  int64_t bd = INT64_MAX;
  for (i = 0; i < s_ncal; i++)
  {
    int64_t dx = cx_q8 - (int32_t)s_cal[i].x * 256, dy = cy_q8 - (int32_t)s_cal[i].y * 256;
    int64_t d = dx * dx + dy * dy;
    if (d < bd) { bd = d; best = s_cal[i].place; }
  }
  return (bd < (int64_t)40 * 40 * 65536) ? best : -1;
}

/* Map observed centres to the calibration constellation after one rigid
 * translation, so a camera pan does not drop every place to -1. */
static void assign_places_rigid(const int16_t *xs, const int16_t *ys, int8_t *places, int n)
{
  int i, j, k, t, best_ok = -1, have = 0;
  int64_t best_worst = 0;
  int8_t trial[DIAL_MAX_FOUND], bestp[DIAL_MAX_FOUND];
  for (i = 0; i < n; i++) { places[i] = -1; }
  if ((s_ncal == 0U) || (n <= 0)) { return; }
  for (k = 0; k < (int)s_ncal; k++)
  {
    for (t = 0; t < n; t++)
    {
      int dx = (int)xs[t] - (int)s_cal[k].x;
      int dy = (int)ys[t] - (int)s_cal[k].y;
      int ok = 0, conflict = 0;
      int64_t worst = 0;
      unsigned used = 0U;
      for (i = 0; i < n; i++)
      {
        int64_t bd = (int64_t)40 * 40 + 1, ddx, ddy, d;
        int bp = -1;
        for (j = 0; j < (int)s_ncal; j++)
        {
          ddx = (int64_t)xs[i] - ((int)s_cal[j].x + dx);
          ddy = (int64_t)ys[i] - ((int)s_cal[j].y + dy);
          d = ddx * ddx + ddy * ddy;
          if (d < bd) { bd = d; bp = (int)s_cal[j].place; }
        }
        if ((bd <= (int64_t)40 * 40) && (bp >= 0) && ((used & (1U << bp)) == 0U))
        {
          used |= (1U << bp);
          trial[i] = (int8_t)bp;
          ok++;
          if (bd > worst) { worst = bd; }
        }
        else if (bd <= (int64_t)40 * 40)
        {
          conflict = 1; trial[i] = -1;
        }
        else { trial[i] = -1; }
      }
      if (conflict) { continue; }
      if ((ok > best_ok) || ((ok == best_ok) && ((have == 0) || (worst < best_worst))))
      {
        best_ok = ok; best_worst = worst; have = 1;
        for (i = 0; i < n; i++) { bestp[i] = trial[i]; }
      }
    }
  }
  if (have) { for (i = 0; i < n; i++) { places[i] = bestp[i]; } }
}

/* ---------------- persistent 4-dial track cache ---------------- */
/* Last good constellation: used so a later frame only re-finds the
 * moving red pointers. Printed ticks and rim geometry are reused,
 * rigidly translated by a few pixels when the picture jitters. */
#define TRACK_SHIFT_MAX  16     /* accepted |dx|,|dy| of the constellation */
#define TRACK_RESID_MAX   4     /* max hub residual vs consensus, px       */
#define TRACK_MIN_AGREE   3     /* hubs that must agree on (dx, dy)        */
#define TRACK_HUB_PAD     8     /* extra chamfer margin around last hub    */
#ifndef DIAL_TRACK_FAST
#define DIAL_TRACK_FAST   0     /* 0: locate dials and ticks every frame   */
#endif

typedef struct
{
  uint8_t  valid;
  uint8_t  n;
  int16_t  seam_col;
  uint8_t  seam_mode;
  dial_geo_t g[DIAL_COUNT_CFG];
  int32_t  last_pang[DIAL_COUNT_CFG];
  int32_t  last_value[DIAL_COUNT_CFG];
} track_t;

static track_t s_track;

void DialRead_ResetTrack(void)
{
  s_track.valid = 0U;
  s_track.n = 0U;
}

static int track_complete(const dial_report_t *rep, int nd)
{
  int i, pl, seen = 0;
  if (nd != (int)DIAL_COUNT_CFG) { return 0; }
  for (i = 0; i < nd; i++)
  {
    pl = rep->dial[i].place;
    if ((pl < 0) || (pl >= (int)DIAL_COUNT_CFG)) { return 0; }
    if (seen & (1 << pl)) { return 0; }
    seen |= (1 << pl);
  }
  return (seen == ((1 << (int)DIAL_COUNT_CFG) - 1)) ? 1 : 0;
}


static void report_copy_ticks(dial_report_t *rep, const dial_geo_t *geo, int nd)
{
  int i, k;
  memset(rep->ovl, 0, sizeof(rep->ovl));
  if (nd < 0) { nd = 0; }
  if (nd > (int)DIAL_MAX_FOUND) { nd = (int)DIAL_MAX_FOUND; }
  for (i = 0; i < nd; i++)
  {
    int n = geo[i].T.n_tick ? geo[i].T.n_tick : N_TICK;
    if (n > (int)DIAL_MAX_TICK) { n = (int)DIAL_MAX_TICK; }
    rep->ovl[i].n_tick = (uint8_t)n;
    rep->ovl[i].allow_half = (uint8_t)(geo[i].T.allow_half ? 1 : 0);
    rep->ovl[i].major_base = (uint8_t)geo[i].T.major_base;
    for (k = 0; k < n; k++)
    {
      rep->ovl[i].tick_mdeg[k] = geo[i].T.tick[k];
      rep->ovl[i].keep[k] = geo[i].T.keep[k];
    }
  }
}

static void track_save(const dial_report_t *rep, const dial_geo_t *geo, int nd)
{
  int i;
  if (!track_complete(rep, nd)) { return; }
  s_track.n = (uint8_t)nd;
  s_track.seam_col = rep->seam_col;
  s_track.seam_mode = rep->seam_mode;
  for (i = 0; i < nd; i++)
  {
    s_track.g[i] = geo[i];
    s_track.g[i].place = rep->dial[i].place;
    s_track.g[i].hub_x = rep->dial[i].hub_x;
    s_track.g[i].hub_y = rep->dial[i].hub_y;
    s_track.g[i].hub_r = rep->dial[i].hub_r;
    s_track.last_pang[i] = rep->dial[i].pang_mdeg;
    s_track.last_value[i] = rep->dial[i].value_e4;
  }
  s_track.valid = 1U;
#ifdef DIAL_HOST_DEBUG
  printf("track_save n=%d seam=%d\n", nd, (int)rep->seam_col);
#endif
}

/* Label red pixels only inside [x0,x1) x [y0,y1). Neighbours outside
 * the rectangle are ignored so overlapping dial crops stay independent. */
static void label_red_roi(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t *par)
{
  int32_t x, y;
  uint32_t n = 0U;
  if (x0 < 0) { x0 = 0; }
  if (y0 < 0) { y0 = 0; }
  if (x1 > (int32_t)s_w) { x1 = (int32_t)s_w; }
  if (y1 > (int32_t)s_h) { y1 = (int32_t)s_h; }
  for (y = y0; y < y1; y++)
  {
    dial_yield();
    for (x = x0; x < x1; x++)
    {
      uint32_t o = (uint32_t)y * s_w + (uint32_t)x;
      uint16_t up, lf;
      if (!s_red[o]) { s_lab[o] = 0U; continue; }
      up = (y > y0) ? s_lab[o - s_w] : 0U;
      lf = (x > x0) ? s_lab[o - 1U] : 0U;
      if (up && lf)
      {
        uint16_t a = uf_find(par, up), b = uf_find(par, lf);
        s_lab[o] = up;
        if (a != b) { par[b] = a; }
      }
      else if (up) { s_lab[o] = up; }
      else if (lf) { s_lab[o] = lf; }
      else if (n + 1U < MAX_LABELS) { n++; par[n] = (uint16_t)n; s_lab[o] = (uint16_t)n; }
      else { s_lab[o] = 0U; }
    }
  }
  for (y = y0; y < y1; y++)
  {
    dial_yield();
    for (x = x0; x < x1; x++)
    {
      uint32_t o = (uint32_t)y * s_w + (uint32_t)x;
      if (s_lab[o]) { s_lab[o] = uf_find(par, s_lab[o]); }
    }
  }
}

static uint16_t lab_near(int32_t x, int32_t y, int rad)
{
  int r, dx, dy;
  uint16_t id;
  if ((x >= 0) && (y >= 0) && (x < (int32_t)s_w) && (y < (int32_t)s_h))
  {
    id = s_lab[(uint32_t)y * s_w + (uint32_t)x];
    if (id) { return id; }
  }
  for (r = 1; r <= rad; r++)
  {
    for (dy = -r; dy <= r; dy++)
    {
      for (dx = -r; dx <= r; dx++)
      {
        int xx, yy;
        if ((iabs32(dx) != r) && (iabs32(dy) != r)) { continue; }
        xx = x + dx; yy = y + dy;
        if ((xx < 0) || (yy < 0) || (xx >= (int32_t)s_w) || (yy >= (int32_t)s_h)) { continue; }
        id = s_lab[(uint32_t)yy * s_w + (uint32_t)xx];
        if (id) { return id; }
      }
    }
  }
  return 0U;
}

static uint32_t count_lab_roi(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t id)
{
  int32_t x, y;
  uint32_t a = 0U;
  for (y = y0; y < y1; y++)
  {
    for (x = x0; x < x1; x++)
    {
      if (s_lab[(uint32_t)y * s_w + (uint32_t)x] == id) { a++; }
    }
  }
  return a;
}

/* Find the current red-pointer hub in a small window around the last one. */
static int track_hub_at(int16_t hx0, int16_t hy0, uint8_t hub_r,
                        int32_t *hx, int32_t *hy, uint16_t *dtm)
{
  int32_t wr = (int32_t)hub_r + TRACK_SHIFT_MAX + TRACK_HUB_PAD;
  int32_t wx0, wy0, wx1, wy1;
  uint16_t dt;
  if (wr < TRACK_SHIFT_MAX + 20) { wr = TRACK_SHIFT_MAX + 20; }
  if (wr > RAD_PAD) { wr = RAD_PAD; }
  wx0 = (int32_t)hx0 - wr; if (wx0 < 0) { wx0 = 0; }
  wy0 = (int32_t)hy0 - wr; if (wy0 < 0) { wy0 = 0; }
  wx1 = (int32_t)hx0 + wr; if (wx1 > (int32_t)s_w) { wx1 = (int32_t)s_w; }
  wy1 = (int32_t)hy0 + wr; if (wy1 > (int32_t)s_h) { wy1 = (int32_t)s_h; }
  if (((wx1 - wx0) < 16) || ((wy1 - wy0) < 16)) { return -1; }
  if ((wx1 - wx0) * (wy1 - wy0) > (2 * RAD_PAD) * (2 * RAD_PAD)) { return -1; }
  dt = chamfer(wx0, wy0, wx1 - wx0, wy1 - wy0, mask_red, 0U, hx, hy);
  if (dt < HUB_MIN_DT) { return -1; }
  if ((3 * (*hx) - (int32_t)dt < 3) || (3 * (*hy) - (int32_t)dt < 3) ||
      (3 * (*hx) + (int32_t)dt > 3 * ((int32_t)s_w - 2)) ||
      (3 * (*hy) + (int32_t)dt > 3 * ((int32_t)s_h - 2))) { return -1; }
  *dtm = dt;
  return 0;
}

/* 0 = consensus shift, -1 = fall back to a full keyframe. */
static int track_shift(int *pdx, int *pdy, int32_t *hx, int32_t *hy, uint16_t *dtm, int *found)
{
  int i, n, ok[DIAL_COUNT_CFG];
  int medx, medy, agree, nfound = 0;
  int tmpx[DIAL_COUNT_CFG], tmpy[DIAL_COUNT_CFG];
  if (!s_track.valid || (s_track.n != (uint8_t)DIAL_COUNT_CFG)) { return 1; }
  for (i = 0; i < (int)DIAL_COUNT_CFG; i++)
  {
    found[i] = 0;
    if (track_hub_at(s_track.g[i].hub_x, s_track.g[i].hub_y, s_track.g[i].hub_r,
                     &hx[i], &hy[i], &dtm[i]) != 0)
    {
      continue;
    }
    found[i] = 1;
    nfound++;
  }
  if (nfound < TRACK_MIN_AGREE) { return 10; }
  n = 0;
  for (i = 0; i < (int)DIAL_COUNT_CFG; i++)
  {
    if (!found[i]) { continue; }
    tmpx[n] = (int)hx[i] - (int)s_track.g[i].hub_x;
    tmpy[n] = (int)hy[i] - (int)s_track.g[i].hub_y;
    n++;
  }
  medx = median_i(tmpx, n);
  /* median_i sorts in place; refill y after x, so copy again */
  n = 0;
  for (i = 0; i < (int)DIAL_COUNT_CFG; i++)
  {
    if (!found[i]) { continue; }
    tmpy[n] = (int)hy[i] - (int)s_track.g[i].hub_y;
    n++;
  }
  medy = median_i(tmpy, n);
  if ((iabs32(medx) > TRACK_SHIFT_MAX) || (iabs32(medy) > TRACK_SHIFT_MAX)) { return 11; }
  agree = 0;
  for (i = 0; i < (int)DIAL_COUNT_CFG; i++)
  {
    int rx, ry;
    ok[i] = 0;
    if (!found[i]) { continue; }
    rx = ((int)hx[i] - (int)s_track.g[i].hub_x) - medx;
    ry = ((int)hy[i] - (int)s_track.g[i].hub_y) - medy;
    if ((iabs32(rx) <= TRACK_RESID_MAX) && (iabs32(ry) <= TRACK_RESID_MAX))
    {
      ok[i] = 1; agree++;
    }
  }
  if (agree < TRACK_MIN_AGREE) { return 12; }
  /* Recompute the median on inliers only. */
  n = 0;
  for (i = 0; i < (int)DIAL_COUNT_CFG; i++)
  {
    if (!ok[i]) { continue; }
    tmpx[n] = (int)hx[i] - (int)s_track.g[i].hub_x;
    n++;
  }
  medx = median_i(tmpx, n);
  n = 0;
  for (i = 0; i < (int)DIAL_COUNT_CFG; i++)
  {
    if (!ok[i]) { continue; }
    tmpy[n] = (int)hy[i] - (int)s_track.g[i].hub_y;
    n++;
  }
  medy = median_i(tmpy, n);
  if ((iabs32(medx) > TRACK_SHIFT_MAX) || (iabs32(medy) > TRACK_SHIFT_MAX)) { return 13; }
  *pdx = medx;
  *pdy = medy;
#ifdef DIAL_HOST_DEBUG
  printf("track_shift dx=%d dy=%d found=%d agree=%d\n", medx, medy, nfound, agree);
#endif
  return 0;
}

/* Fast path: translate the cached constellation, then read pointers only.
 * Returns 0 on success (rep/geo filled, n_dials = 4). -1 = fall back. */
static int track_fast(dial_report_t *rep, dial_geo_t *geo, uint16_t *par, uint16_t w, uint16_t h)
{
  int i, dx, dy, found[DIAL_COUNT_CFG];
  int32_t hx[DIAL_COUNT_CFG], hy[DIAL_COUNT_CFG];
  uint16_t dtm[DIAL_COUNT_CFG];
  {
    int sh = track_shift(&dx, &dy, hx, hy, dtm, found);
    if (sh != 0)
    {
#ifdef DIAL_HOST_DEBUG
      printf("track_fast: shift consensus failed %d\n", sh);
#endif
      return sh;
    }
  }
  for (i = 0; i < (int)DIAL_COUNT_CFG; i++)
  {
    dial_geo_t g = s_track.g[i];
    dial_info_t *di;
    int32_t tx, ty, cx, cy;
    uint16_t id, dti;
    int32_t nhx, nhy;
    g.cx += dx * 256;
    g.cy += dy * 256;
    g.x0 += dx; g.y0 += dy; g.x1 += dx; g.y1 += dy;
    if (g.x0 < 0) { g.x0 = 0; }
    if (g.y0 < 0) { g.y0 = 0; }
    if (g.x1 > (int32_t)w) { g.x1 = (int32_t)w; }
    if (g.y1 > (int32_t)h) { g.y1 = (int32_t)h; }
    if (((g.x1 - g.x0) < 80) || ((g.y1 - g.y0) < 80))
    {
#ifdef DIAL_HOST_DEBUG
      printf("track_fast: ROI clipped place=%d\n", (int)g.place);
#endif
      return 20 + ((g.place >= 0) ? (int)g.place : 0);
    }
    if ((g.cx < 0) || (g.cy < 0) || (g.cx >= (int32_t)w * 256) || (g.cy >= (int32_t)h * 256))
    {
      return 30 + ((g.place >= 0) ? (int)g.place : 0);
    }
    s_x0 = g.x0; s_y0 = g.y0; s_x1 = g.x1; s_y1 = g.y1;
    apply_face_thresholds(g.face_w, g.face_d);
    label_red_roi(s_x0, s_y0, s_x1, s_y1, par);
    /* Prefer the blob at the consensus-shifted last hub (or the measured one). */
    nhx = (int32_t)s_track.g[i].hub_x + dx;
    nhy = (int32_t)s_track.g[i].hub_y + dy;
    if (found[i]) { nhx = hx[i]; nhy = hy[i]; }
    id = lab_near(nhx, nhy, 8);
    if (!id || (count_lab_roi(s_x0, s_y0, s_x1, s_y1, id) < CAND_MIN_AREA))
    {
#ifdef DIAL_HOST_DEBUG
      printf("track_fast: no pointer blob place=%d\n", (int)g.place);
#endif
      return 40 + ((g.place >= 0) ? (int)g.place : 0);
    }
    if (pointer_tip(id, &tx, &ty) != 0)
    {
#ifdef DIAL_HOST_DEBUG
      printf("track_fast: pointer_tip failed place=%d\n", (int)g.place);
#endif
      return 50 + ((g.place >= 0) ? (int)g.place : 0);
    }
    cx = g.cx; cy = g.cy;
    g.tipx = tx; g.tipy = ty;
    /* Refresh the hub from this pointer label when possible; else shifted cache. */
    dti = chamfer(s_x0, s_y0, s_x1 - s_x0, s_y1 - s_y0, mask_lab, id, &nhx, &nhy);
    if (dti >= HUB_MIN_DT)
    {
      g.hub_x = (int16_t)nhx; g.hub_y = (int16_t)nhy; g.hub_r = (uint8_t)(dti / 3U);
    }
    else if (found[i])
    {
      g.hub_x = (int16_t)hx[i]; g.hub_y = (int16_t)hy[i]; g.hub_r = (uint8_t)(dtm[i] / 3U);
    }
    else
    {
      g.hub_x = (int16_t)((int)s_track.g[i].hub_x + dx);
      g.hub_y = (int16_t)((int)s_track.g[i].hub_y + dy);
    }
    di = &rep->dial[i];
    memset(di, 0, sizeof(*di));
    di->cx_q8 = cx; di->cy_q8 = cy; di->r_q8 = g.R;
    di->tip_x_q8 = tx; di->tip_y_q8 = ty;
    di->pang_mdeg = atan2_mdeg((int64_t)ty - cy, (int64_t)tx - cx);
    di->value_e4 = read_value(di->pang_mdeg, g.maj);
    if (di->value_e4 >= 100000) { di->value_e4 -= 100000; }
    di->hub_x = g.hub_x; di->hub_y = g.hub_y; di->hub_r = g.hub_r;
    di->n_vis = (uint8_t)g.T.n_vis;
    di->arc_cov = g.arc_cov;
    di->place = g.place;
    geo[i] = g;
  }
  rep->n_dials = (uint8_t)DIAL_COUNT_CFG;
  rep->n_scale_bad = 0U;
#ifdef DIAL_HOST_DEBUG
  printf("track_fast: ok dx=%d dy=%d\n", dx, dy);
#endif
  return 0;
}

/* ---------------- main entry ---------------- */
uint32_t DialRead_WorkBytes(uint16_t w, uint16_t h)
{
  uint32_t n = (uint32_t)w * h;
  /* +128 covers 8-byte padding after each slice (needed on M7: word
   * clear/copy faults on a 2-byte-aligned uint32_t*). */
  return n * 2U            /* restored frame       */
       + n                 /* luma                 */
       + n                 /* red                  */
       + n * 2U            /* labels               */
       + (2U * RAD_PAD) * (2U * RAD_PAD) * 2U   /* chamfer */
       + MAX_LABELS * 2U   /* union-find parents   */
       + MAX_LABELS * 4U   /* areas                */
       + (uint32_t)w * 4U  /* seam column sums     */
       + 128U;
}

static void make_tables(void)
{
  int i;
  for (i = 0; i < 32; i++) { s_r5to8[i] = (uint8_t)((i * 255) / 31); }
  for (i = 0; i < 64; i++) { s_g6to8[i] = (uint8_t)((i * 255) / 63); }
}

static void dbg_begin(void)
{
  uint32_t i;
  g_dial_dbg.magic = DIAL_DBG_MAGIC;
  g_dial_dbg.seq++;
  g_dial_dbg.suspect_flags = 0U;
  g_dial_dbg.seam_col = 0;
  g_dial_dbg.track_fb = 0U;
  g_dial_dbg.seam_mode = 0U;
  g_dial_dbg.seam_reused = 0U;
  g_dial_dbg.n_blobs = 0U;
  g_dial_dbg.n_cand = 0U;
  g_dial_dbg.n_dials = 0U;
  g_dial_dbg.n_scale_bad = 0U;
  g_dial_dbg.status = 0U;
  g_dial_dbg.place_mask = 0U;
  g_dial_dbg.miss_place = 0U;
  g_dial_dbg.cnt_area = 0U;
  g_dial_dbg.cnt_win = 0U;
  g_dial_dbg.cnt_hub_dt = 0U;
  g_dial_dbg.cnt_hub_clip = 0U;
  g_dial_dbg.cnt_arc = 0U;
  g_dial_dbg.cnt_tip = 0U;
  g_dial_dbg.cnt_vis = 0U;
  for (i = 0U; i < DIAL_DBG_NCAND; i++)
  {
    dial_dbg_cand_t *c = &g_dial_dbg.cand[i];
    c->cx = 0; c->cy = 0; c->hx = -1; c->hy = -1;
    c->area = 0U; c->dtm = 0U; c->place = -1;
    c->reason = 0U; c->tip_sub = 0U; c->n_vis = 0U;
  }
}

static void dbg_note(int16_t cx, int16_t cy, int16_t hx, int16_t hy,
                     uint32_t area, uint16_t dtm, int8_t place,
                     uint8_t reason, uint8_t tip_sub, uint8_t n_vis)
{
  uint8_t i = g_dial_dbg.n_cand;
  if (i < (uint8_t)DIAL_DBG_NCAND)
  {
    dial_dbg_cand_t *c = &g_dial_dbg.cand[i];
    c->cx = cx; c->cy = cy; c->hx = hx; c->hy = hy;
    c->area = (area > 65535U) ? 65535U : (uint16_t)area;
    c->dtm = dtm; c->place = place;
    c->reason = reason; c->tip_sub = tip_sub; c->n_vis = n_vis;
    g_dial_dbg.n_cand = (uint8_t)(i + 1U);
  }
  if (reason == DIAL_FAIL_AREA) { g_dial_dbg.cnt_area++; }
  else if (reason == DIAL_FAIL_WIN) { g_dial_dbg.cnt_win++; }
  else if (reason == DIAL_FAIL_HUB_DT) { g_dial_dbg.cnt_hub_dt++; }
  else if (reason == DIAL_FAIL_HUB_CLIP) { g_dial_dbg.cnt_hub_clip++; }
  else if (reason == DIAL_FAIL_ARC) { g_dial_dbg.cnt_arc++; }
  else if (reason == DIAL_FAIL_TIP) { g_dial_dbg.cnt_tip++; }
  else if (reason == DIAL_FAIL_VIS) { g_dial_dbg.cnt_vis++; }
}

static int8_t dbg_place_of(int b, int32_t hx, int32_t hy, int32_t cx0, int32_t cy0)
{
  int8_t pl = ((b >= 0) && (b < (int)DIAL_MAX_BLOBS)) ? s_blob_place[b] : (int8_t)-1;
  if ((pl < 0) && (hx >= 0) && (hy >= 0))
  {
    int g = match_cal((int32_t)hx * 256, (int32_t)hy * 256);
    if (g >= 0) { pl = (int8_t)g; }
  }
  if (pl < 0)
  {
    int g = match_cal(cx0, cy0);
    if (g >= 0) { pl = (int8_t)g; }
  }
  return pl;
}

static void dbg_finish(const dial_report_t *rep, int nb, int nd)
{
  uint8_t have = 0U, miss, p, j, ci;
  int k;
  g_dial_dbg.n_blobs = (nb < 0) ? 0U : ((nb > 255) ? 255U : (uint8_t)nb);
  g_dial_dbg.n_dials = (nd < 0) ? 0U : ((nd > 255) ? 255U : (uint8_t)nd);
  if (rep != 0)
  {
    g_dial_dbg.n_scale_bad = rep->n_scale_bad;
    g_dial_dbg.track_fb = rep->track_fb;
    g_dial_dbg.status = (rep->status < 0) ? 0xFFU : (uint8_t)rep->status;
    g_dial_dbg.suspect_flags = rep->suspect_flags;
    g_dial_dbg.seam_col = rep->seam_col;
    g_dial_dbg.seam_mode = rep->seam_mode;
  }
  /* accepted cand places may have been refined after the locate loop */
  ci = 0U;
  for (j = 0U; (j < g_dial_dbg.n_cand) && (ci < (uint8_t)nd); j++)
  {
    if (g_dial_dbg.cand[j].reason == DIAL_FAIL_OK)
    {
      g_dial_dbg.cand[j].place = rep->dial[ci].place;
      ci++;
    }
  }
  for (k = 0; k < nd; k++)
  {
    int pl = rep->dial[k].place;
    if ((pl >= 0) && (pl < 8)) { have = (uint8_t)(have | (uint8_t)(1U << pl)); }
  }
  miss = (uint8_t)(((1U << DIAL_COUNT_CFG) - 1U) & (uint8_t)~have);
  g_dial_dbg.place_mask = have;
  g_dial_dbg.miss_place = miss;
  if (miss != 0U)
  {
    g_dial_dbg.fail_seq = g_dial_dbg.seq;
    g_dial_dbg.last_n_dials = g_dial_dbg.n_dials;
    g_dial_dbg.last_miss_place = miss;
    g_dial_dbg.last_n_cand = g_dial_dbg.n_cand;
    g_dial_dbg.last_seam_reused = g_dial_dbg.seam_reused;
    g_dial_dbg.last_seam_col = g_dial_dbg.seam_col;
    for (p = 0U; p < 4U; p++)
    {
      int16_t cx = 0, cy = 0, hx = -1, hy = -1;
      uint8_t reason = DIAL_FAIL_NO_BLOB, tip = 0U;
      if ((have & (uint8_t)(1U << p)) != 0U)
      {
        reason = DIAL_FAIL_OK;
        for (j = 0U; j < g_dial_dbg.n_cand; j++)
        {
          if ((g_dial_dbg.cand[j].place == (int8_t)p) && (g_dial_dbg.cand[j].reason == DIAL_FAIL_OK))
          {
            cx = g_dial_dbg.cand[j].cx; cy = g_dial_dbg.cand[j].cy;
            hx = g_dial_dbg.cand[j].hx; hy = g_dial_dbg.cand[j].hy;
            tip = g_dial_dbg.cand[j].tip_sub;
            break;
          }
        }
      }
      else
      {
        for (j = 0U; j < g_dial_dbg.n_cand; j++)
        {
          if (g_dial_dbg.cand[j].place == (int8_t)p)
          {
            reason = g_dial_dbg.cand[j].reason;
            tip = g_dial_dbg.cand[j].tip_sub;
            cx = g_dial_dbg.cand[j].cx; cy = g_dial_dbg.cand[j].cy;
            hx = g_dial_dbg.cand[j].hx; hy = g_dial_dbg.cand[j].hy;
            break;
          }
        }
      }
      g_dial_dbg.last_reason[p] = reason;
      g_dial_dbg.last_tip_sub[p] = tip;
      g_dial_dbg.last_cx[p] = cx; g_dial_dbg.last_cy[p] = cy;
      g_dial_dbg.last_hx[p] = hx; g_dial_dbg.last_hy[p] = hy;
    }
  }
}

int DialRead_Run(const uint16_t *raw, uint16_t *out, uint16_t w, uint16_t h,
                 uint8_t *work, uint32_t work_bytes, dial_report_t *rep)
{
  static blob_t bl[DIAL_MAX_BLOBS];
  static dial_geo_t geo[DIAL_MAX_FOUND];
  static int16_t hist[360];
  static peak_t pk[64];
  uint32_t npx = (uint32_t)w * h, i;
  uint8_t *p = work;
  uint16_t *rest, *par;
  uint32_t *area, *colsum;
  uint8_t *L, *red;
  int nb, b, nd = 0, seam;
  uint8_t mode = 0U;
  dial_time_t ts;
  if (!raw || !out || !work || !rep || (w < 16U) || (h < 16U)) { return DIAL_ERR_ARG; }
  if (work_bytes < DialRead_WorkBytes(w, h)) { return DIAL_ERR_WORK; }
  memset(rep, 0, sizeof(*rep));
  dbg_begin();
  make_tables();
  p = (uint8_t *)(((uintptr_t)p + 7U) & ~(uintptr_t)7U);
  rest = (uint16_t *)p; p += npx * 2U;
  p = (uint8_t *)(((uintptr_t)p + 7U) & ~(uintptr_t)7U);
  s_lab = (uint16_t *)p; p += npx * 2U;
  p = (uint8_t *)(((uintptr_t)p + 7U) & ~(uintptr_t)7U);
  s_dt = (uint16_t *)p; p += (2U * RAD_PAD) * (2U * RAD_PAD) * 2U;
  p = (uint8_t *)(((uintptr_t)p + 7U) & ~(uintptr_t)7U);
  par = (uint16_t *)p; p += MAX_LABELS * 2U;
  p = (uint8_t *)(((uintptr_t)p + 7U) & ~(uintptr_t)7U);
  area = (uint32_t *)p; p += MAX_LABELS * 4U;
  p = (uint8_t *)(((uintptr_t)p + 7U) & ~(uintptr_t)7U);
  colsum = (uint32_t *)p; p += (uint32_t)w * 4U;
  p = (uint8_t *)(((uintptr_t)p + 7U) & ~(uintptr_t)7U);
  L = p; p += npx;
  p = (uint8_t *)(((uintptr_t)p + 7U) & ~(uintptr_t)7U);
  red = p; p += npx;
  s_w = w; s_h = h; s_L = L; s_red = red;

  /* 1 seam restore. Reuse the last good frame's seam (including a
   * nonzero wrap column) so hubs stay in the same unwrapped
   * coordinate system. Detect only when there is no cache, or when
   * a fixed column is configured. */
  /* Reuse last good seam even when locating every frame: x0.1 sits
   * near the wrap, and a jittering seam splits that dial so it
   * disappears from the overlay. */
  if ((s_track.valid) && (DIAL_SEAM_COL_CFG < 0))
  {
    seam = s_track.seam_col; mode = s_track.seam_mode;
    g_dial_dbg.seam_reused = 1U;
  }
  else if (DIAL_SEAM_COL_CFG < 0) { seam = seam_detect(raw, w, h, &mode, colsum); }
  else { seam = DIAL_SEAM_COL_CFG; mode = (DIAL_SEAM_MODE_CFG == 1) ? 1U : 0U; if (DIAL_SEAM_MODE_CFG == 2) { (void)seam_detect(raw, w, h, &mode, colsum); } }
  rep->seam_col = (int16_t)seam;
  rep->seam_mode = mode;
  g_dial_dbg.seam_col = (int16_t)seam;
  g_dial_dbg.seam_mode = mode;
  if (seam <= 0)
  {
    uint32_t y, x;
    for (y = 0U; y < h; y++)
    {
      const uint16_t *src = raw + y * w;
      uint16_t *dst = rest + y * w;
      dial_yield();
      for (x = 0U; x < w; x++) { dst[x] = src[x]; }
    }
  }
  else if (mode == 1U)
  {
    for (i = 0U; i < npx; i++)
    {
      uint32_t j = i + (uint32_t)seam;
      if (j >= npx) { j -= npx; }
      rest[i] = raw[j];
      if ((i % (uint32_t)w) == 0U) { dial_yield(); }
    }
  }
  else
  {
    uint32_t y, x;
    for (y = 0U; y < h; y++)
    {
      const uint16_t *src = raw + y * w;
      uint16_t *dst = rest + y * w;
      dial_yield();
      for (x = 0U; x < w; x++) { uint32_t j = x + (uint32_t)seam; if (j >= w) { j -= w; } dst[x] = src[j]; }
    }
  }
  /* 2 Fast path: reuse last-keyframe level LUT and the four cached
   * dial windows. Only level/rectify those ROIs (expanded so a few
   * pixels of jitter still sit inside the buffers), then track the
   * pointer angles. A failed track falls through to a full-frame
   * level + rectify so s_L/s_red are complete before find_blobs. */
  rep->track_mode = 0U;
  rep->track_fb = 1U; /* 1 = no cache, full keyframe */
  if (DIAL_TRACK_FAST && s_track.valid)
  {
    int ti;
    uint32_t yy;
    /* last keyframe's gain/offset and s_lvl stay in force */
    rep->level_gain_q8 = (int16_t)s_gain_q8;
    rep->level_off = (int16_t)(s_off / 256);
    /* L/red are a fresh work buffer. Pixels outside the ROIs must be
     * 0, not uninitialized garbage, or track_hub_at's chamfer window
     * can pick up false red and fail the shift consensus every cycle. */
    memset(L, 0, npx);
    memset(red, 0, npx);
    /* cheap unwrapped copy so the display is not left as a wrapped raw frame */
    if ((void *)out != (void *)rest)
    {
      for (yy = 0U; yy < h; yy++)
      {
        dial_yield();
        memcpy(out + (uint32_t)yy * w, rest + (uint32_t)yy * w, (size_t)w * 2U);
      }
    }
    for (ti = 0; ti < (int)s_track.n; ti++)
    {
      int32_t pad = TRACK_SHIFT_MAX + TRACK_HUB_PAD;
      int32_t wr = (int32_t)s_track.g[ti].hub_r + pad;
      int32_t rx0, ry0, rx1, ry1, hx0, hy0, hx1, hy1;
      if (wr < TRACK_SHIFT_MAX + 20) { wr = TRACK_SHIFT_MAX + 20; }
      rx0 = (int32_t)s_track.g[ti].x0 - pad;
      ry0 = (int32_t)s_track.g[ti].y0 - pad;
      rx1 = (int32_t)s_track.g[ti].x1 + pad;
      ry1 = (int32_t)s_track.g[ti].y1 + pad;
      hx0 = (int32_t)s_track.g[ti].hub_x - wr;
      hy0 = (int32_t)s_track.g[ti].hub_y - wr;
      hx1 = (int32_t)s_track.g[ti].hub_x + wr;
      hy1 = (int32_t)s_track.g[ti].hub_y + wr;
      if (hx0 < rx0) { rx0 = hx0; }
      if (hy0 < ry0) { ry0 = hy0; }
      if (hx1 > rx1) { rx1 = hx1; }
      if (hy1 > ry1) { ry1 = hy1; }
      rectify_roi(rest, out, L, red, w, h, rx0, ry0, rx1, ry1);
    }
    s_rgb = out;
#ifdef DIAL_HOST_DEBUG
    { FILE *fp_ = fopen("dbg_L.raw", "wb"); if (fp_) { fwrite(L, 1, npx, fp_); fwrite(red, 1, npx, fp_); fclose(fp_); } }
#endif
    {
      int fb = track_fast(rep, geo, par, w, h);
      if (fb == 0)
      {
        rep->track_mode = 1U;
        rep->track_fb = 0U;
        nd = (int)rep->n_dials;
        goto after_locate;
      }
      rep->track_fb = (uint16_t)fb;
    }
    memset(rep->dial, 0, sizeof(rep->dial));
    rep->n_dials = 0U;
    rep->n_scale_bad = 0U;
    rep->suspect_flags = 0U;
    rep->ambig_places = 0U;
  }

  /* 2b keyframe (or fast-path fallback): full-frame level + rectify */
  dial_yield();
  level_setup(rest, npx, area);
  rep->level_gain_q8 = (int16_t)s_gain_q8;
  rep->level_off = (int16_t)(s_off / 256);
  rectify(rest, out, L, red, w, h);
  s_rgb = out;
#ifdef DIAL_HOST_DEBUG
  { FILE *fp_ = fopen("dbg_L.raw", "wb"); if (fp_) { fwrite(L, 1, npx, fp_); fwrite(red, 1, npx, fp_); fclose(fp_); } }
#endif

  /* 3 red blobs (keyframe: full-frame locate + ticks) */
  dial_yield();
  nb = find_blobs(par, area, bl, DIAL_MAX_BLOBS);
  /* first pass: hubs only, so places can follow a camera pan (rigid shift) */
  {
    int16_t hxs[DIAL_MAX_FOUND], hys[DIAL_MAX_FOUND];
    int8_t hpl[DIAL_MAX_FOUND];
    int hb[DIAL_MAX_FOUND], nhub = 0, i;
    for (b = 0; (b < nb) && (nhub < (int)DIAL_MAX_FOUND); b++)
    {
      int32_t cx0, cy0, hx, hy, wx0, wy0, wx1, wy1;
      uint16_t dtm;
      if (bl[b].area < CAND_MIN_AREA) { continue; }
      cx0 = (int32_t)((bl[b].sx * 256) / bl[b].area);
      cy0 = (int32_t)((bl[b].sy * 256) / bl[b].area);
      wx0 = (cx0 >> 8) - HUB_WIN; if (wx0 < 0) { wx0 = 0; }
      wy0 = (cy0 >> 8) - HUB_WIN; if (wy0 < 0) { wy0 = 0; }
      wx1 = (cx0 + HUB_WIN * 256) >> 8; if (wx1 > (int32_t)w) { wx1 = w; }
      wy1 = (cy0 + HUB_WIN * 256) >> 8; if (wy1 > (int32_t)h) { wy1 = h; }
      if ((wx1 - wx0) * (wy1 - wy0) > (2 * RAD_PAD) * (2 * RAD_PAD)) { continue; }
      dtm = chamfer(wx0, wy0, wx1 - wx0, wy1 - wy0, mask_red, 0U, &hx, &hy);
      if (dtm < HUB_MIN_DT) { continue; }
      if ((3 * hx - dtm < 3) || (3 * hy - dtm < 3) ||
          (3 * hx + dtm > 3 * ((int32_t)w - 2)) || (3 * hy + dtm > 3 * ((int32_t)h - 2))) { continue; }
      hxs[nhub] = (int16_t)hx; hys[nhub] = (int16_t)hy; hb[nhub] = b; nhub++;
    }
    assign_places_rigid(hxs, hys, hpl, nhub);
    /* stash place in the unused high half of first? we'll look up by blob index below via a small table */
    for (i = 0; i < (int)DIAL_MAX_BLOBS; i++) { s_blob_place[i] = -1; }
    for (i = 0; i < nhub; i++) { if (hb[i] < (int)DIAL_MAX_BLOBS) { s_blob_place[hb[i]] = hpl[i]; } }
  }
  for (b = 0; (b < nb) && (nd < (int)DIAL_MAX_FOUND); b++)
  {
    dial_yield();
    int32_t cx0 = (int32_t)((bl[b].sx * 256) / bl[b].area), cy0 = (int32_t)((bl[b].sy * 256) / bl[b].area);
    int32_t hx, hy, cx, cy, R, tx, ty, wx0, wy0, wx1, wy1, maj[10];
    uint16_t dtm;
    int cov, k, k0 = 0;
    int32_t bdu = 0x7fffffff;
    dial_info_t *di;
    if (bl[b].area < CAND_MIN_AREA) {
      dbg_note((int16_t)(cx0 >> 8), (int16_t)(cy0 >> 8), -1, -1, bl[b].area, 0U,
               dbg_place_of(b, -1, -1, cx0, cy0), DIAL_FAIL_AREA, 0U, 0U);
      continue;
    }
    /* hub: max chamfer distance of red inside +-120 px */
    wx0 = (cx0 >> 8) - HUB_WIN; if (wx0 < 0) { wx0 = 0; }
    wy0 = (cy0 >> 8) - HUB_WIN; if (wy0 < 0) { wy0 = 0; }
    wx1 = (cx0 + HUB_WIN * 256) >> 8; if (wx1 > (int32_t)w) { wx1 = w; }
    wy1 = (cy0 + HUB_WIN * 256) >> 8; if (wy1 > (int32_t)h) { wy1 = h; }
    if ((wx1 - wx0) * (wy1 - wy0) > (2 * RAD_PAD) * (2 * RAD_PAD)) {
      dbg_note((int16_t)(cx0 >> 8), (int16_t)(cy0 >> 8), -1, -1, bl[b].area, 0U,
               dbg_place_of(b, -1, -1, cx0, cy0), DIAL_FAIL_WIN, 0U, 0U);
      continue;
    }
    dtm = chamfer(wx0, wy0, wx1 - wx0, wy1 - wy0, mask_red, 0U, &hx, &hy);
    if (dtm < HUB_MIN_DT) {
      dbg_note((int16_t)(cx0 >> 8), (int16_t)(cy0 >> 8), (int16_t)hx, (int16_t)hy, bl[b].area, dtm,
               dbg_place_of(b, hx, hy, cx0, cy0), DIAL_FAIL_HUB_DT, 0U, 0U);
      continue;
    }                  /* red mark, not a pointer */
    if ((3 * hx - dtm < 3) || (3 * hy - dtm < 3) ||
        (3 * hx + dtm > 3 * ((int32_t)w - 2)) || (3 * hy + dtm > 3 * ((int32_t)h - 2))) {
      dbg_note((int16_t)(cx0 >> 8), (int16_t)(cy0 >> 8), (int16_t)hx, (int16_t)hy, bl[b].area, dtm,
               dbg_place_of(b, hx, hy, cx0, cy0), DIAL_FAIL_HUB_CLIP, 0U, 0U);
      continue;
    }  /* hub clipped */
    /* dial crop */
    s_x0 = (cx0 >> 8) - RAD_PAD; if (s_x0 < 0) { s_x0 = 0; }
    s_y0 = (cy0 >> 8) - RAD_PAD; if (s_y0 < 0) { s_y0 = 0; }
    s_x1 = (cx0 >> 8) + RAD_PAD; if (s_x1 > (int32_t)w) { s_x1 = w; }
    s_y1 = (cy0 >> 8) + RAD_PAD; if (s_y1 > (int32_t)h) { s_y1 = h; }
    di = &rep->dial[nd];
    di->hub_x = (int16_t)hx; di->hub_y = (int16_t)hy; di->hub_r = (uint8_t)(dtm / 3U);
    /* 4 rim circle from an arc segment; thresholds from this dial's white face */
    {
      int Wf, Df;
      estimate_face_ink((int)hx, (int)hy, &Wf, &Df);
      apply_face_thresholds(Wf, Df);
#ifdef DIAL_HOST_DEBUG
      printf("  dial@(%d,%d) face W=%d ink D=%d WHITE=%d STRONG=%d CONTRAST=%d\n",
             (int)hx, (int)hy, Wf, Df, s_arc_white, s_arc_t_strong, s_arc_contrast);
#endif
    }
    if ((arc_circle(cx0, cy0, &cx, &cy, &R, &cov) != 0) || (R < R_MIN_Q8) || (R > R_MAX_Q8))
    {
      rep->n_scale_bad++;
      dbg_note((int16_t)(cx0 >> 8), (int16_t)(cy0 >> 8), (int16_t)hx, (int16_t)hy, bl[b].area, dtm,
               dbg_place_of(b, hx, hy, cx0, cy0), DIAL_FAIL_ARC, 0U, 0U);
      continue;
    }
    /* place from the rigidly shifted calibration (never left/right) */
    di->place = (b < (int)DIAL_MAX_BLOBS) ? s_blob_place[b] : (int8_t)-1;
    if (di->place < 0)
    {
      int place_guess = match_cal((int32_t)hx * 256, (int32_t)hy * 256);
      if (place_guess < 0) { place_guess = match_cal(cx0, cy0); }
      di->place = (int8_t)place_guess;
    }
    /* 6 pointer first: pang is needed by tick filters (rules 2-3) */
    if (pointer_tip(bl[b].id, &tx, &ty) != 0) {
      rep->n_scale_bad++;
      dbg_note((int16_t)(cx0 >> 8), (int16_t)(cy0 >> 8), (int16_t)hx, (int16_t)hy, bl[b].area, dtm,
               (int8_t)di->place, DIAL_FAIL_TIP, s_tip_why, 0U);
      continue;
    }
    di->pang_mdeg = atan2_mdeg((int64_t)ty - cy, (int64_t)tx - cx);
    /* 5 ticks. Peak angles stay on the 1 px gap profile so a close
     * tadpole merge cannot slide a local max (235° -> 234°, 8.89 -> 8.90).
     * Close gaps are merged when measuring each detected tick. */
    tick_len_profile(cx, cy, R, hist);
    {
      int allow_half = (di->place == 3);   /* only x0.0001; unknown -> no half ticks */
      int np, off, mb;
      np = find_peaks(hist, pk, 64, 7);
#ifdef DIAL_HOST_DEBUG
      { int q_; printf("  dial@(%d,%d) place=%d allow_half=%d c=(%.2f,%.2f,%.2f)\n  raw peaks %d:", cx0 >> 8, cy0 >> 8, di->place, allow_half, cx/256.0, cy/256.0, R/256.0, np); for (q_ = 0; q_ < np; q_++) printf(" %d:%d", pk[q_].a / 1000, pk[q_].h); printf("\n"); }
#endif
      np = filter_tick_peaks(pk, np, cx, cy, R, di->pang_mdeg, 1, allow_half);
      phase_offset(pk, np, &off, &mb, allow_half);
      build_ticks(hist, pk, np, off, mb, allow_half, &geo[nd].T);
#ifdef DIAL_HOST_DEBUG
      { int q_; printf("  after filter %d: ", np); for (q_ = 0; q_ < np; q_++) printf(" %d:%d", pk[q_].a / 1000, pk[q_].h);
        printf("  off=%d mb=%d n_tick=%d vis=%d ticks:", off, mb, geo[nd].T.n_tick, geo[nd].T.n_vis); for (q_ = 0; q_ < geo[nd].T.n_tick; q_++) printf(" %s%.1f", geo[nd].T.vis[q_] ? "" : "~", geo[nd].T.tick[q_] / 1000.0); printf("\n"); }
#endif
    }
    /* 7 value: interpolate between printed majors (10 of them either way) */
    for (k = 0; k < 10; k++)
    {
      int32_t ang = geo[nd].T.allow_half
                  ? geo[nd].T.tick[geo[nd].T.major_base + 2 * k]
                  : geo[nd].T.tick[k];
      int32_t d = iabs32(angdiff_mdeg(ang, 270000));
      if (d < bdu) { bdu = d; k0 = k; }
    }
    for (k = 0; k < 10; k++)
    {
      int ik = (k0 + k) % 10;
      maj[k] = geo[nd].T.allow_half
             ? geo[nd].T.tick[geo[nd].T.major_base + 2 * ik]
             : geo[nd].T.tick[ik];
    }
    if (!geo[nd].T.allow_half) { anchor_majors_4_6_0(maj); }
    di->value_e4 = read_value(di->pang_mdeg, maj);
    if (di->value_e4 >= 100000) { di->value_e4 -= 100000; }
    snap_completed(cx, cy, R, &geo[nd].T);
    di->cx_q8 = cx; di->cy_q8 = cy; di->r_q8 = R; di->tip_x_q8 = tx; di->tip_y_q8 = ty;
    di->n_vis = (uint8_t)geo[nd].T.n_vis; di->arc_cov = (uint8_t)cov;
    /* 8 validity: n_vis in [MIN_VIS_TICKS, n_tick] */
    if ((geo[nd].T.n_vis < MIN_VIS_TICKS) || (geo[nd].T.n_vis > geo[nd].T.n_tick) ||
        (cx < 0) || (cy < 0) || (cx >= (int32_t)w * 256) || (cy >= (int32_t)h * 256))
    {
      rep->n_scale_bad++;
      dbg_note((int16_t)(cx0 >> 8), (int16_t)(cy0 >> 8), (int16_t)hx, (int16_t)hy, bl[b].area, dtm,
               (int8_t)di->place, DIAL_FAIL_VIS, s_tip_why, (uint8_t)geo[nd].T.n_vis);
      continue;
    }
    geo[nd].cx = cx; geo[nd].cy = cy; geo[nd].R = R;
    geo[nd].x0 = s_x0; geo[nd].y0 = s_y0; geo[nd].x1 = s_x1; geo[nd].y1 = s_y1;
    geo[nd].tipx = tx; geo[nd].tipy = ty;
    for (k = 0; k < 10; k++) { geo[nd].maj[k] = maj[k]; }
    geo[nd].hub_x = (int16_t)hx; geo[nd].hub_y = (int16_t)hy;
    geo[nd].hub_r = (uint8_t)(dtm / 3U);
    geo[nd].place = di->place;
    geo[nd].face_w = s_face_w; geo[nd].face_d = s_face_d;
    geo[nd].arc_cov = (uint8_t)cov;
    dbg_note((int16_t)(cx0 >> 8), (int16_t)(cy0 >> 8), (int16_t)hx, (int16_t)hy, bl[b].area, dtm,
             (int8_t)di->place, DIAL_FAIL_OK, s_tip_why, (uint8_t)geo[nd].T.n_vis);
    nd++;
  }
  /* refine places from the fitted rim centres (same rigid shift) */
  if (nd > 0)
  {
    int16_t xs[DIAL_MAX_FOUND], ys[DIAL_MAX_FOUND];
    int8_t pl[DIAL_MAX_FOUND];
    int j;
    for (j = 0; j < nd; j++)
    {
      xs[j] = (int16_t)(rep->dial[j].cx_q8 >> 8);
      ys[j] = (int16_t)(rep->dial[j].cy_q8 >> 8);
    }
    assign_places_rigid(xs, ys, pl, nd);
    for (j = 0; j < nd; j++) { if (pl[j] >= 0) { rep->dial[j].place = pl[j]; } }
  }
after_locate:
  rep->n_dials = (uint8_t)nd;
  /* Cache a complete 4-dial constellation (and its seam). A partial
   * result must not overwrite a good seam: x0.1 is next to the wrap. */
  if (nd == (int)DIAL_COUNT_CFG)
  {
    track_save(rep, geo, nd);
  }
  report_copy_ticks(rep, geo, nd);
  if (rep->n_scale_bad) { rep->suspect_flags |= DIAL_SUS_SCALE; }
  if (nd != (int)DIAL_COUNT_CFG) { rep->suspect_flags |= DIAL_SUS_COUNT; }
  {
    int32_t val[DIAL_MAX_FOUND];
    int have[DIAL_MAX_FOUND], dg[DIAL_MAX_FOUND], j, np = (int)DIAL_COUNT_CFG;
    for (j = 0; j < (int)DIAL_MAX_FOUND; j++) { have[j] = 0; val[j] = 0; dg[j] = 0; }
    for (j = 0; j < nd; j++)
    {
      int pl = rep->dial[j].place;
      if ((pl < 0) || (pl >= np) || have[pl]) { rep->suspect_flags |= DIAL_SUS_CAL; continue; }
      have[pl] = 1; val[pl] = rep->dial[j].value_e4;
    }
    for (j = 0; j < np; j++) { if (!have[j]) { rep->suspect_flags |= DIAL_SUS_CAL; } }
    if (rep->suspect_flags == 0U)
    {
      /* x0.001 prior on x0.0001: the 0-5 / 5-10 half-range is only
       * a disambiguation aid at the x0.001 5-tick crossing (the
       * original SUSPECT case, |v001-5| <= 0.50). A clear tip such
       * as 6.8 is never folded to 1.8 just because x0.001 is past 5
       * (e.g. at 6). Inside that window, 0.0001 in [0,3] => already
       * past 5; [7,10] => not yet; else use 0.001. */
      if (have[2] && have[3])
      {
        int32_t v001 = val[2], v0001 = val[3];
        int passed;
        int d;
        if (iabs32(v001 - 50000) <= 5000)
        {
          if (v0001 < 30000) { passed = 1; }
          else if (v0001 >= 70000) { passed = 0; }
          else { passed = (v001 >= 50000); }
          if (passed) { if (v0001 > 50000) { v0001 -= 50000; } }
          else { if (v0001 < 50000) { v0001 += 50000; } }
          if (v0001 != val[3])
          {
            val[3] = v0001;
            for (d = 0; d < nd; d++)
            {
              if (rep->dial[d].place == 3) { rep->dial[d].value_e4 = v0001; }
            }
          }
        }
      }
      /* 10 boundary rules (lowest dial x0.0001 excluded).
       * Uncertain window follows how close this pointer is to a tick and
       * sits at the 0-3 / 7-10 decision edges, not a fixed cover on 5. */
      for (j = 0; j < np; j++) { dg[j] = (val[j] / 10000) % 10; }
      for (j = 0; j < np - 1; j++)
      {
        int32_t f = val[j] % 10000, nv = val[j + 1];
        int32_t edge = (f < (10000 - f)) ? f : (10000 - f);
        int32_t half = 0;
        int32_t d3, d7;
        if (edge < 1000) { half = (4000 * (1000 - edge)) / 1000; }  /* e4 units, max 0.40 */
        d3 = iabs32(nv - 30000);
        d7 = iabs32(nv - 70000);
        if ((half > 0) && ((d3 <= half) || (d7 <= half)))
        {
          rep->ambig_places |= (uint8_t)(1U << j);
          continue;
        }
        if ((f >= 7500) && (nv < 25000)) { dg[j] = (dg[j] + 1) % 10; }
        else if ((f <= 2500) && (nv >= 80000)) { dg[j] = (dg[j] + 9) % 10; }
      }
      if (rep->ambig_places) { rep->suspect_flags |= DIAL_SUS_AMBIG; }
      /* 11 combined: higher dials integer digit, lowest dial to 0.01 tick */
      {
        uint32_t lo = (uint32_t)((val[np - 1] + 50) / 100) % 1000U, r = 0U;
        for (j = 0; j < np - 1; j++) { r = r * 10U + (uint32_t)dg[j]; }
        rep->reading_e6 = r * 1000U + lo;
      }
      for (j = 0; j < nd; j++) { rep->dial[j].digit = (uint8_t)dg[rep->dial[j].place]; }
    }
  }
  /* timestamp */
  dial_get_time(&ts);
  rep->ts = ts;
  {
    char *q = rep->line1;
    q = put_n(q, ts.year, 4); *q++ = '-'; q = put_n(q, ts.mon, 2); *q++ = '-'; q = put_n(q, ts.day, 2);
    *q++ = ' '; q = put_n(q, ts.hour, 2); *q++ = ':'; q = put_n(q, ts.min, 2); *q++ = ':'; q = put_n(q, ts.sec, 2);
    *q++ = ' '; *q++ = '['; q = put_n(q, (uint32_t)s_read_seq, 3); *q++ = ']';
    *q = 0;
    s_read_seq++;
    if (s_read_seq > 999U) { s_read_seq = 0U; }
    q = rep->ts_name;
    q = put_n(q, ts.year, 4); q = put_n(q, ts.mon, 2); q = put_n(q, ts.day, 2); *q++ = '_';
    q = put_n(q, ts.hour, 2); q = put_n(q, ts.min, 2); q = put_n(q, ts.sec, 2); *q = 0;
  }
  if (rep->suspect_flags != 0U)
  {
    /* SUSPECT: raw (restored + rectified) frame, timestamp only, nothing else */
    rep->status = DIAL_SUSPECT;
    draw_text(out, 12, 8, rep->line1);
    (void)dial_save_suspect_frame(out, w, h, rep->ts_name);
    dbg_finish(rep, nb, nd);
    return DIAL_SUSPECT;
  }
  {
    char *q = rep->line2;
    /* reading_e6 is millionths of a m3, i.e. milliliters; 3 decimal liters */
    uint32_t v = rep->reading_e6;
    uint32_t lit = v / 1000U;
    uint32_t frac = v % 1000U;
    int nd = 1;
    uint32_t t = lit;
    while (t >= 10U) { t /= 10U; nd++; }
    q = put_n(q, lit, nd);
    *q++ = '.';
    q = put_n(q, frac, 3);
    *q++ = ' ';
    *q++ = 'L';
    *q = 0;
  }
  /* 12 overlay */
  {
    int j;
    for (j = 0; j < nd; j++)
    {
      if (geo[j].face_w > 0) { apply_face_thresholds(geo[j].face_w, geo[j].face_d); }
      paint_ticks(out, &geo[j]);
      paint_pointer_line(out, &geo[j]);
    }
    draw_text(out, 12, 8, rep->line1);
    draw_text(out, 12, 44, rep->line2);
    for (j = 0; j < nd; j++)
    {
      char s[2];
      int32_t ty = (geo[j].cy + geo[j].R + 2 * 256) >> 8;
      if (ty > (int32_t)h - 36) { ty = (int32_t)h - 36; }
      s[0] = (char)('0' + rep->dial[j].digit); s[1] = 0;
      draw_text(out, (int)((geo[j].cx - 8 * 256) >> 8), (int)ty, s);
    }
  }
  rep->status = DIAL_OK;
  dbg_finish(rep, nb, nd);
  return DIAL_OK;
}

/* ---------------- speed-based auto ordering ---------------- */
#define ORD_MAXF  DIAL_MAX_FOUND
static struct
{
  int n;
  int16_t x[ORD_MAXF], y[ORD_MAXF];
  int32_t last[ORD_MAXF];
  int64_t acc[ORD_MAXF];      /* |unwrapped angle| travelled, mdeg */
  uint32_t t0, t1;
  uint32_t frames;
} s_ord;

void DialOrder_Reset(void)
{
  memset(&s_ord, 0, sizeof(s_ord));
}

/* The fastest (lowest) dial must turn < 180 deg between two fed frames. */
int DialOrder_Feed(const dial_report_t *rep, uint32_t t_ms)
{
  int j, k;
  if (rep->n_dials == 0U) { return -1; }
  for (j = 0; j < rep->n_dials; j++)
  {
    const dial_info_t *d = &rep->dial[j];
    int32_t x = d->cx_q8 >> 8, y = d->cy_q8 >> 8;
    int m = -1;
    for (k = 0; k < s_ord.n; k++)
    {
      if ((iabs32(x - s_ord.x[k]) < 40) && (iabs32(y - s_ord.y[k]) < 40)) { m = k; break; }
    }
    if (m < 0)
    {
      if (s_ord.n >= (int)ORD_MAXF) { continue; }
      m = s_ord.n++;
      s_ord.x[m] = (int16_t)x; s_ord.y[m] = (int16_t)y; s_ord.last[m] = d->pang_mdeg; s_ord.acc[m] = 0;
      continue;
    }
    s_ord.acc[m] += iabs32(angdiff_mdeg(d->pang_mdeg, s_ord.last[m]));
    s_ord.last[m] = d->pang_mdeg;
  }
  if (s_ord.frames == 0U) { s_ord.t0 = t_ms; }
  s_ord.t1 = t_ms;
  s_ord.frames++;
  return (int)s_ord.frames;
}

/* slowest dial = place 0.  Neighbour speed ratio must be 4..25 (nominal 10). */
int DialOrder_Finish(dial_cal_t *cal_out, uint8_t *n_out)
{
  int idx[ORD_MAXF], i, j, n = s_ord.n;
  if ((s_ord.frames < 5U) || (n != (int)DIAL_COUNT_CFG)) { return -1; }
  for (i = 0; i < n; i++) { idx[i] = i; }
  for (i = 1; i < n; i++)
  {
    int t = idx[i];
    j = i - 1;
    while ((j >= 0) && (s_ord.acc[idx[j]] > s_ord.acc[t])) { idx[j + 1] = idx[j]; j--; }
    idx[j + 1] = t;
  }
  if (s_ord.acc[idx[n - 1]] < 90000) { return -2; }    /* lowest dial moved < 90 deg: not enough flow */
  for (i = 1; i < n; i++)
  {
    int64_t lo = s_ord.acc[idx[i - 1]], hi = s_ord.acc[idx[i]];
    if (lo <= 0) { continue; }                          /* top dial may not have moved yet */
    if ((hi < 4 * lo) || (hi > 25 * lo)) { return -3; }
  }
  for (i = 0; i < n; i++)
  {
    cal_out[i].x = s_ord.x[idx[i]]; cal_out[i].y = s_ord.y[idx[i]]; cal_out[i].place = (int8_t)i;
  }
  *n_out = (uint8_t)n;
  DialRead_SetOrderCal(cal_out, (uint8_t)n);
  return 0;
}
