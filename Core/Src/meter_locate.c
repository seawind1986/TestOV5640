#include "meter_locate.h"
#include <math.h>
#include <stdint.h>

#define PTR_BLUE   0x029FU
#define MAX_BLOBS  24U
#define PI_MILLI   3142
#define TWO_PI_MILLI 6283

/* ===== 可调参数：暂时做成全局变量，调好后再放回函数内 =====
 * 已标 volatile，可在 MDK 的 Watch 窗口里运行中直接改，不必每次重编。
 */

/* 轨迹圆/搜索半径 = 圆A半径 * g_search_mul，再限制在 [g_search_min, g_search_max] 像素。
 * g_search_mul  建议 2~5，默认 3。太小找不到圆B，太大会搜到旁边另一根指针。
 * g_search_min  建议 32~80，默认 48。
 * g_search_max  建议 80~200，默认 160。 */
volatile int g_search_mul = 3;
volatile int g_search_min = 48;
volatile int g_search_max = 160;

/* 圆B最小半径 = 圆A半径 / g_b_rmin_div，再限制在 [g_b_rmin_min, g_b_rmin_max] 像素。
 * g_b_rmin_div  建议 3~8，默认 5。
 * g_b_rmin_min  建议 2~8，默认 4。太小会把噪声当成尖端。
 * g_b_rmin_max  建议 8~20，默认 12。太大则圆B塞不进指针尖角。 */
volatile int g_b_rmin_div = 5;
volatile int g_b_rmin_min = 4;
volatile int g_b_rmin_max = 12;

/* 圆A与圆B之间的最小间隙（像素），保证圆B在尖角内而不贴在轴心圆上。建议 1~8，默认 3。 */
volatile int g_tip_gap = 3;

/* 红色像素判定（通道已扩展到 0~255）。
 * g_red_min     R 下限，建议 90~140，默认 115。
 * g_red_over_g  R 比 G 至少大多少，建议 8~30，默认 16。
 * g_red_over_b  R 比 B 至少大多少，建议 8~30，默认 20。
 * g_red_g_max   G 上限，排除发白高光，建议 180~240，默认 220。
 * g_red_b_max   B 上限，建议 180~240，默认 200。 */
volatile int g_red_min = 115;
volatile int g_red_over_g = 16;
volatile int g_red_over_b = 20;
volatile int g_red_g_max = 220;
volatile int g_red_b_max = 200;

/* 定位时红色色块最小面积（像素）。建议 150~500，默认 280。太小会把噪点当指针。 */
volatile int g_min_area = 280;

/* 表盘背景过滤：轴心附近浅色像素至少这么多、金色像素不能超过这么多，才当作指针。
 * g_pale_min  浅色下限，建议 400~800，默认 650。
 * g_gold_max  金色上限，建议 50~200，默认 100。 */
volatile int g_pale_min = 650;
volatile int g_gold_max = 100;

/* 跟踪时，轴心圆附近（半径+4 像素内）至少要有这么多红色像素，否则丢锁。建议 10~40，默认 20。 */
volatile int g_track_near_min = 20;

/* 角度低通，只用于 ang_milli（以后算流量），不再改 LCD 上的轮廓位置。
 * 相邻帧跳变小于 g_ang_jump_milli 毫弧度时，新角度权重为 g_ang_alpha_num/10；
 * 超过则只采用 1/g_ang_jump_div，避免被噪声带跑。
 * g_ang_alpha_num    建议 3~10（10=几乎不滤波），默认 3，即 0.3。
 * g_ang_jump_milli   建议 800~2000（约 46°~115°），默认 1200（约 69°）。
 * g_ang_jump_div     建议 2~8，默认 4。 */
volatile int g_ang_alpha_num = 3;
volatile int g_ang_jump_milli = 1200;
volatile int g_ang_jump_div = 4;

/* LCD 调试颜色（RGB565）：绿圈为 A/B，红圈为轨迹圆，品红用于把画面里的黑色刻度盘描出来。 */
volatile uint16_t g_color_ab = 0x07E0u;
volatile uint16_t g_color_search = 0xF800u;
volatile uint16_t g_color_dial = 0xF81Fu;

/* 描黑范围：环带内径/外径 = 圆A半径 * mul / 10，再限制在 [g_dial_rmin, g_dial_rmax] 像素。
 * g_dial_rmul_min  建议 8~16（0.8~1.6 倍），默认 10，从轴心圆外侧开始。
 * g_dial_rmul_max  建议 28~40（2.8~4.0 倍），默认 36。太大会描到旁边表盘或文字。 */
volatile int g_dial_rmul_min = 10;
volatile int g_dial_rmul_max = 36;
volatile int g_dial_rmin = 16;
volatile int g_dial_rmax = 160;

/* 把多暗的像素当成刻度盘上的黑墨。亮度 0~255，建议 70~140，默认 110。偏高会把灰影也描上，偏低会漏掉浅黑刻度。 */
volatile int g_dial_ink_luma = 110;
/* 黑墨的色度上限（最大通道-最小通道）。用来排除红色指针。建议 20~60，默认 40。 */
volatile int g_dial_ink_chroma = 40;

/* 算出的（画面上没有墨、补出来的）刻度线和圆弧，用品红的互补色绿色。 */
volatile uint16_t g_color_dial_est = 0x07E0u;
/* LCD 左上角读数颜色，默认黄。 */
volatile uint16_t g_color_text = 0xFFE0u;
/* 读数底色，默认黑。 */
volatile uint16_t g_color_text_bg = 0x0000u;

/* 刻度数量：普通表盘 10 格；最左边（圆心 x 最小）的 ×0.0001 表盘 20 格（含半格）。 */
volatile int g_tick_n = 10;
volatile int g_tick_n_left = 20;

/* 主刻度、半刻度从圆弧向内的长度（像素）。建议 4~16，默认 8 / 5。 */
volatile int g_tick_len_major = 8;
volatile int g_tick_len_minor = 5;

typedef struct
{
  uint32_t area;
  uint32_t sumx;
  uint32_t sumy;
  uint16_t minx;
  uint16_t miny;
  uint16_t maxx;
  uint16_t maxy;
} blob_t;

static int is_red(uint16_t p)
{
  uint32_t r = ((uint32_t)(p >> 11) & 0x1FU) << 3;
  uint32_t g = ((uint32_t)(p >> 5) & 0x3FU) << 2;
  uint32_t b = ((uint32_t)p & 0x1FU) << 3;

  return ((r >= (uint32_t)g_red_min) &&
          (r > (g + (uint32_t)g_red_over_g)) &&
          (r > (b + (uint32_t)g_red_over_b)) &&
          (g < (uint32_t)g_red_g_max) &&
          (b < (uint32_t)g_red_b_max)) ? 1 : 0;
}

static void pix_rgb(uint16_t p, uint32_t *r, uint32_t *g, uint32_t *b)
{
  *r = ((uint32_t)(p >> 11) & 0x1FU) << 3;
  *g = ((uint32_t)(p >> 5) & 0x3FU) << 2;
  *b = ((uint32_t)p & 0x1FU) << 3;
}

static void pale_gold(const uint16_t *rgb, uint16_t w, uint16_t h,
                      int cx, int cy, int rad, int *pale_ppm, int *gold_ppm)
{
  int x0, y0, x1, y1, x, y, n, pale, gold, rad2, rad22;
  uint32_t r, g, b, mx, mn, chroma, luma;

  *pale_ppm = 0;
  *gold_ppm = 0;
  if (rad < 4)
  {
    return;
  }
  rad2 = rad * rad;
  rad22 = (rad * 2) * (rad * 2);
  x0 = cx - (rad * 2);
  y0 = cy - (rad * 2);
  x1 = cx + (rad * 2);
  y1 = cy + (rad * 2);
  if (x0 < 0) { x0 = 0; }
  if (y0 < 0) { y0 = 0; }
  if (x1 > (int)w - 1) { x1 = (int)w - 1; }
  if (y1 > (int)h - 1) { y1 = (int)h - 1; }
  n = pale = gold = 0;
  for (y = y0; y <= y1; y++)
  {
    for (x = x0; x <= x1; x++)
    {
      int dx = x - cx;
      int dy = y - cy;
      int d2 = dx * dx + dy * dy;
      uint16_t p;

      if ((d2 < rad2) || (d2 > rad22))
      {
        continue;
      }
      p = rgb[((uint32_t)y * w) + (uint32_t)x];
      pix_rgb(p, &r, &g, &b);
      mx = r;
      mn = r;
      if (g > mx) { mx = g; }
      if (b > mx) { mx = b; }
      if (g < mn) { mn = g; }
      if (b < mn) { mn = b; }
      chroma = mx - mn;
      luma = (r * 77U + g * 150U + b * 29U) >> 8;
      n++;
      if ((luma > 140U) && (chroma < 50U))
      {
        pale++;
      }
      if ((r > 140U) && (g > 100U) && (chroma >= 50U))
      {
        gold++;
      }
    }
  }
  if (n == 0)
  {
    return;
  }
  *pale_ppm = (pale * 1000) / n;
  *gold_ppm = (gold * 1000) / n;
}

static void chamfer_dt(int32_t *dt, int bw, int bh)
{
  int x, y;

  for (y = 0; y < bh; y++)
  {
    for (x = 0; x < bw; x++)
    {
      int32_t *v = &dt[((uint32_t)y * (uint32_t)bw) + (uint32_t)x];
      int32_t t;

      if (*v == 0)
      {
        continue;
      }
      t = *v;
      if (x > 0)
      {
        int32_t n = dt[((uint32_t)y * (uint32_t)bw) + (uint32_t)(x - 1)] + 3;
        if (n < t) { t = n; }
      }
      if (y > 0)
      {
        int32_t n = dt[((uint32_t)(y - 1) * (uint32_t)bw) + (uint32_t)x] + 3;
        if (n < t) { t = n; }
        if (x > 0)
        {
          n = dt[((uint32_t)(y - 1) * (uint32_t)bw) + (uint32_t)(x - 1)] + 4;
          if (n < t) { t = n; }
        }
        if (x + 1 < bw)
        {
          n = dt[((uint32_t)(y - 1) * (uint32_t)bw) + (uint32_t)(x + 1)] + 4;
          if (n < t) { t = n; }
        }
      }
      *v = t;
    }
  }
  for (y = bh - 1; y >= 0; y--)
  {
    for (x = bw - 1; x >= 0; x--)
    {
      int32_t *v = &dt[((uint32_t)y * (uint32_t)bw) + (uint32_t)x];
      int32_t t;

      if (*v == 0)
      {
        continue;
      }
      t = *v;
      if (x + 1 < bw)
      {
        int32_t n = dt[((uint32_t)y * (uint32_t)bw) + (uint32_t)(x + 1)] + 3;
        if (n < t) { t = n; }
      }
      if (y + 1 < bh)
      {
        int32_t n = dt[((uint32_t)(y + 1) * (uint32_t)bw) + (uint32_t)x] + 3;
        if (n < t) { t = n; }
        if (x + 1 < bw)
        {
          n = dt[((uint32_t)(y + 1) * (uint32_t)bw) + (uint32_t)(x + 1)] + 4;
          if (n < t) { t = n; }
        }
        if (x > 0)
        {
          n = dt[((uint32_t)(y + 1) * (uint32_t)bw) + (uint32_t)(x - 1)] + 4;
          if (n < t) { t = n; }
        }
      }
      *v = t;
    }
  }
}

static int dt_radius(int32_t v)
{
  int r;

  if (v <= 0)
  {
    return 0;
  }
  r = (int)(((float)v / 3.0f) * 0.98f);
  if (r < 1)
  {
    r = 1;
  }
  return r;
}


static int search_radius(int r)
{
  int s;
  int mul = g_search_mul;
  int mn = g_search_min;
  int mx = g_search_max;

  if (mul < 1)
  {
    mul = 1;
  }
  s = r * mul;
  if (s < mn)
  {
    s = mn;
  }
  if (s > mx)
  {
    s = mx;
  }
  return s;
}

static int circle_b_rmin(int r)
{
  int rmin;
  int div = g_b_rmin_div;

  if (div < 1)
  {
    div = 1;
  }
  rmin = r / div;
  if (rmin < g_b_rmin_min)
  {
    rmin = g_b_rmin_min;
  }
  if (rmin > g_b_rmin_max)
  {
    rmin = g_b_rmin_max;
  }
  return rmin;
}

/* 在距离变换里找圆 B：半径够大、与圆 A 分开、在搜索范围内，取离 A 最远的。 */
static int find_circle_b(const int32_t *dt, int x0, int y0, int bw, int bh,
                         int cx, int cy, int r,
                         int *obx, int *oby, int *obr)
{
  int x, y, rmin, search, best_d2, bx, by, br;

  rmin = circle_b_rmin(r);
  search = search_radius(r);
  best_d2 = -1;
  bx = cx;
  by = cy;
  br = 0;
  for (y = 0; y < bh; y++)
  {
    for (x = 0; x < bw; x++)
    {
      int32_t v = dt[((uint32_t)y * (uint32_t)bw) + (uint32_t)x];
      int rb, ix, iy, dx, dy, d2, sep, lim;

      if (v == 0)
      {
        continue;
      }
      rb = dt_radius(v);
      if (rb < rmin)
      {
        continue;
      }
      ix = x + x0;
      iy = y + y0;
      dx = ix - cx;
      dy = iy - cy;
      d2 = dx * dx + dy * dy;
      sep = r + rb + g_tip_gap;
      if (d2 < (sep * sep))
      {
        continue;
      }
      lim = search - rb;
      if (lim < 1)
      {
        continue;
      }
      if (d2 > (lim * lim))
      {
        continue;
      }
      if (d2 > best_d2)
      {
        best_d2 = d2;
        bx = ix;
        by = iy;
        br = rb;
      }
    }
  }
  if (best_d2 < 0)
  {
    return 0;
  }
  *obx = bx;
  *oby = by;
  *obr = br;
  return 1;
}

static void tip_on_ray(const int32_t *dt, int x0, int y0, int bw, int bh,
                       int cx, int cy, int bx, int by,
                       int *tx, int *ty)
{
  float dx, dy, d, ux, uy;
  int i, lastx, lasty, maxs;

  dx = (float)(bx - cx);
  dy = (float)(by - cy);
  d = sqrtf((dx * dx) + (dy * dy));
  if (d < 1.0f)
  {
    *tx = bx;
    *ty = by;
    return;
  }
  ux = dx / d;
  uy = dy / d;
  lastx = bx;
  lasty = by;
  maxs = (int)d + 80;
  for (i = 0; i <= maxs; i++)
  {
    int ix = (int)lroundf((float)cx + ux * (float)i);
    int iy = (int)lroundf((float)cy + uy * (float)i);
    int lx = ix - x0;
    int ly = iy - y0;

    if ((lx < 0) || (ly < 0) || (lx >= bw) || (ly >= bh))
    {
      break;
    }
    if (dt[((uint32_t)ly * (uint32_t)bw) + (uint32_t)lx] == 0)
    {
      break;
    }
    lastx = ix;
    lasty = iy;
  }
  *tx = lastx;
  *ty = lasty;
}

static int32_t wrap_da(int32_t da)
{
  while (da > PI_MILLI)
  {
    da -= TWO_PI_MILLI;
  }
  while (da < -PI_MILLI)
  {
    da += TWO_PI_MILLI;
  }
  return da;
}

static void apply_angle(meter_pointer_t *out, int cx, int cy, int r,
                        int bx, int by, int br, int tx, int ty)
{
  int32_t a = (int32_t)lroundf(atan2f((float)(by - cy), (float)(bx - cx)) * 1000.0f);

  /* 轮廓按本帧测到的 A/B/尖端来画，不要用滤波后的角度去改位置，否则会跟不上画面。 */
  out->cx = (uint16_t)cx;
  out->cy = (uint16_t)cy;
  out->r = (uint16_t)r;
  out->bx = (uint16_t)bx;
  out->by = (uint16_t)by;
  out->br = (uint16_t)br;
  out->tx = (uint16_t)tx;
  out->ty = (uint16_t)ty;
  if (out->have_ang != 0U)
  {
    int32_t da = wrap_da(a - out->ang_milli);
    if ((da > g_ang_jump_milli) || (da < -g_ang_jump_milli))
    {
      int div = g_ang_jump_div;
      if (div < 1)
      {
        div = 1;
      }
      a = out->ang_milli + (da / div);
    }
    else
    {
      a = out->ang_milli + ((da * g_ang_alpha_num) / 10);
    }
    a = wrap_da(a);
  }
  out->ang_milli = a;
  out->have_ang = 1U;
  out->locked = 1U;
}

static int fill_dt_from_red(const uint16_t *rgb, uint16_t w, uint16_t h,
                            int x0, int y0, int x1, int y1,
                            int32_t *dt, int bw, int bh)
{
  int x, y;
  uint32_t area = 0U;
  uint32_t n = (uint32_t)bw * (uint32_t)bh;
  uint32_t i;

  for (i = 0; i < n; i++)
  {
    dt[i] = 0;
  }
  for (y = y0; y <= y1; y++)
  {
    for (x = x0; x <= x1; x++)
    {
      if (is_red(rgb[((uint32_t)y * w) + (uint32_t)x]) != 0)
      {
        dt[((uint32_t)(y - y0) * (uint32_t)bw) + (uint32_t)(x - x0)] = 1000000000;
        area++;
      }
    }
  }
  if (area < 20U)
  {
    return 0;
  }
  chamfer_dt(dt, bw, bh);
  return (int)area;
}

static int fit_from_dt(int32_t *dt, int x0, int y0, int bw, int bh,
                       meter_pointer_t *out)
{
  int x, y, cx, cy, r, peak, ix, iy, bx, by, br, tx, ty;

  peak = 0;
  ix = bw / 2;
  iy = bh / 2;
  for (y = 0; y < bh; y++)
  {
    for (x = 0; x < bw; x++)
    {
      int32_t v = dt[((uint32_t)y * (uint32_t)bw) + (uint32_t)x];
      if (v > peak)
      {
        peak = v;
        ix = x;
        iy = y;
      }
    }
  }
  if (peak < 12)
  {
    return 0;
  }
  {
    int sx = 0, sy = 0, n = 0, thr = (peak * 9) / 10;
    for (y = 0; y < bh; y++)
    {
      for (x = 0; x < bw; x++)
      {
        if (dt[((uint32_t)y * (uint32_t)bw) + (uint32_t)x] >= thr)
        {
          sx += x;
          sy += y;
          n++;
        }
      }
    }
    if (n > 0)
    {
      ix = sx / n;
      iy = sy / n;
    }
  }
  cx = ix + x0;
  cy = iy + y0;
  r = dt_radius(peak);
  if (r < 4)
  {
    r = 4;
  }
  if (find_circle_b(dt, x0, y0, bw, bh, cx, cy, r, &bx, &by, &br) == 0)
  {
    return 0;
  }
  tip_on_ray(dt, x0, y0, bw, bh, cx, cy, bx, by, &tx, &ty);
  out->have_ang = 0U;
  apply_angle(out, cx, cy, r, bx, by, br, tx, ty);
  return 1;
}

static int fit_blob(const uint16_t *rgb, uint16_t w, uint16_t h,
                    const blob_t *b, uint32_t work, uint32_t work_bytes,
                    meter_pointer_t *out)
{
  int x0, y0, x1, y1, bw, bh;
  int32_t *dt;
  uint32_t need;

  x0 = (int)b->minx;
  y0 = (int)b->miny;
  x1 = (int)b->maxx;
  y1 = (int)b->maxy;
  if (x0 > 0) { x0--; }
  if (y0 > 0) { y0--; }
  if (x1 + 1 < (int)w) { x1++; }
  if (y1 + 1 < (int)h) { y1++; }
  bw = x1 - x0 + 1;
  bh = y1 - y0 + 1;
  if ((bw < 6) || (bh < 6))
  {
    return 0;
  }
  need = (uint32_t)bw * (uint32_t)bh * (uint32_t)sizeof(int32_t);
  if ((work == 0U) || (work_bytes < need))
  {
    return 0;
  }
  dt = (int32_t *)work;
  if (fill_dt_from_red(rgb, w, h, x0, y0, x1, y1, dt, bw, bh) == 0)
  {
    return 0;
  }
  return fit_from_dt(dt, x0, y0, bw, bh, out);
}

int MeterLocate_Find(const uint16_t *rgb, uint16_t w, uint16_t h,
                     uint32_t work, uint32_t work_bytes,
                     meter_pointer_t *out, uint8_t max_out)
{
  uint8_t *seen;
  uint32_t *queue;
  uint32_t qneed, mneed, qh, qt, cap;
  blob_t blobs[MAX_BLOBS];
  uint8_t nblob, i, j, nout;
  uint32_t x, y;

  if ((rgb == 0) || (out == 0) || (w < 8U) || (h < 8U) || (max_out == 0U))
  {
    return 0;
  }
  mneed = (uint32_t)w * (uint32_t)h;
  cap = (mneed / 2U) + 16U;
  qneed = cap * 4U;
  if ((work == 0U) || (work_bytes < (mneed + 16U + qneed)))
  {
    return 0;
  }
  seen = (uint8_t *)work;
  queue = (uint32_t *)((work + mneed + 3U) & ~3UL);
  {
    uint32_t k;
    for (k = 0; k < mneed; k++)
    {
      seen[k] = 0U;
    }
  }
  nblob = 0U;
  for (y = 0; y < h; y++)
  {
    for (x = 0; x < w; x++)
    {
      uint32_t idx = (y * (uint32_t)w) + x;
      blob_t *b;
      if ((seen[idx] != 0U) || (is_red(rgb[idx]) == 0))
      {
        continue;
      }
      if (nblob >= MAX_BLOBS)
      {
        break;
      }
      b = &blobs[nblob];
      b->area = 0U;
      b->sumx = 0U;
      b->sumy = 0U;
      b->minx = (uint16_t)x;
      b->maxx = (uint16_t)x;
      b->miny = (uint16_t)y;
      b->maxy = (uint16_t)y;
      qh = 0U;
      qt = 0U;
      queue[qt++] = (x << 16) | y;
      seen[idx] = 1U;
      while (qh < qt)
      {
        uint32_t v = queue[qh++];
        uint16_t px = (uint16_t)(v >> 16);
        uint16_t py = (uint16_t)(v & 0xFFFFU);
        int k;
        static const int dx[4] = { 1, -1, 0, 0 };
        static const int dy[4] = { 0, 0, 1, -1 };

        b->area++;
        b->sumx += px;
        b->sumy += py;
        if (px < b->minx) { b->minx = px; }
        if (px > b->maxx) { b->maxx = px; }
        if (py < b->miny) { b->miny = py; }
        if (py > b->maxy) { b->maxy = py; }
        for (k = 0; k < 4; k++)
        {
          int nx = (int)px + dx[k];
          int ny = (int)py + dy[k];
          uint32_t nidx;
          if ((nx < 0) || (ny < 0) || (nx >= (int)w) || (ny >= (int)h))
          {
            continue;
          }
          nidx = ((uint32_t)ny * (uint32_t)w) + (uint32_t)nx;
          if ((seen[nidx] != 0U) || (is_red(rgb[nidx]) == 0))
          {
            continue;
          }
          if (qt >= cap)
          {
            break;
          }
          seen[nidx] = 1U;
          queue[qt++] = ((uint32_t)nx << 16) | (uint32_t)ny;
        }
      }
      if (b->area >= (uint32_t)g_min_area)
      {
        nblob++;
      }
    }
    if (nblob >= MAX_BLOBS)
    {
      break;
    }
  }

  for (i = 0; i < nblob; i++)
  {
    uint8_t best = i;
    for (j = (uint8_t)(i + 1U); j < nblob; j++)
    {
      if (blobs[j].area > blobs[best].area)
      {
        best = j;
      }
    }
    if (best != i)
    {
      blob_t tmp = blobs[i];
      blobs[i] = blobs[best];
      blobs[best] = tmp;
    }
  }

  if (max_out > METER_MAX_POINTERS)
  {
    max_out = METER_MAX_POINTERS;
  }
  nout = 0U;
  for (i = 0; (i < nblob) && (nout < max_out); i++)
  {
    blob_t *b = &blobs[i];
    int cx, cy, rad, pale, gold;
    int bw = (int)b->maxx - (int)b->minx + 1;
    int bh = (int)b->maxy - (int)b->miny + 1;

    cx = (int)(b->sumx / b->area);
    cy = (int)(b->sumy / b->area);
    rad = (6 * ((bw > bh) ? bw : bh)) / 10;
    pale_gold(rgb, w, h, cx, cy, rad, &pale, &gold);
    if ((pale < g_pale_min) || (gold >= g_gold_max))
    {
      continue;
    }
    if (fit_blob(rgb, w, h, b, work, work_bytes, &out[nout]) != 0)
    {
      nout++;
    }
  }
  return (int)nout;
}

int MeterLocate_Track(const uint16_t *rgb, uint16_t w, uint16_t h,
                      uint32_t work, uint32_t work_bytes,
                      meter_pointer_t *ptrs, uint8_t n)
{
  uint8_t i;
  int alive = 0;

  if ((rgb == 0) || (ptrs == 0))
  {
    return 0;
  }
  for (i = 0; i < n; i++)
  {
    meter_pointer_t *p = &ptrs[i];
    int cx, cy, r, rad, x0, y0, x1, y1, bw, bh, near, rnear, x, y;
    int bx, by, br, tx, ty;
    int32_t *dt;
    uint32_t need;

    if (p->locked == 0U)
    {
      continue;
    }
    cx = (int)p->cx;
    cy = (int)p->cy;
    r = (int)p->r;
    rad = search_radius(r);
    x0 = cx - rad;
    y0 = cy - rad;
    x1 = cx + rad;
    y1 = cy + rad;
    if (x0 < 0) { x0 = 0; }
    if (y0 < 0) { y0 = 0; }
    if (x1 > (int)w - 1) { x1 = (int)w - 1; }
    if (y1 > (int)h - 1) { y1 = (int)h - 1; }
    bw = x1 - x0 + 1;
    bh = y1 - y0 + 1;
    need = (uint32_t)bw * (uint32_t)bh * (uint32_t)sizeof(int32_t);
    if ((work == 0U) || (work_bytes < need) || (bw < 6) || (bh < 6))
    {
      p->locked = 0U;
      continue;
    }
    dt = (int32_t *)work;
    if (fill_dt_from_red(rgb, w, h, x0, y0, x1, y1, dt, bw, bh) == 0)
    {
      p->locked = 0U;
      continue;
    }
    near = 0;
    rnear = (r + 4) * (r + 4);
    for (y = y0; y <= y1; y++)
    {
      for (x = x0; x <= x1; x++)
      {
        int dx = x - cx;
        int dy = y - cy;
        int d2 = dx * dx + dy * dy;
        int lx, ly;
        if (d2 > rnear)
        {
          continue;
        }
        lx = x - x0;
        ly = y - y0;
        if (dt[((uint32_t)ly * (uint32_t)bw) + (uint32_t)lx] != 0)
        {
          near++;
        }
      }
    }
    if (near < g_track_near_min)
    {
      p->locked = 0U;
      continue;
    }
    if (find_circle_b(dt, x0, y0, bw, bh, cx, cy, r, &bx, &by, &br) == 0)
    {
      p->locked = 0U;
      continue;
    }
    tip_on_ray(dt, x0, y0, bw, bh, cx, cy, bx, by, &tx, &ty);
    if (tx < 0) { tx = 0; }
    if (ty < 0) { ty = 0; }
    if (tx >= (int)w) { tx = (int)w - 1; }
    if (ty >= (int)h) { ty = (int)h - 1; }
    if (bx < 0) { bx = 0; }
    if (by < 0) { by = 0; }
    if (bx >= (int)w) { bx = (int)w - 1; }
    if (by >= (int)h) { by = (int)h - 1; }
    apply_angle(p, cx, cy, r, bx, by, br, tx, ty);
    alive++;
  }
  return alive;
}

static void plot(volatile uint16_t *fb, uint16_t w, uint16_t h,
                 int x, int y, uint16_t color)
{
  if ((x >= 0) && (y >= 0) && (x < (int)w) && (y < (int)h))
  {
    fb[((uint32_t)y * w) + (uint32_t)x] = color;
  }
}

static void fill_circle(volatile uint16_t *fb, uint16_t w, uint16_t h,
                        int cx, int cy, int r, uint16_t color)
{
  int x, y, r2;

  if (r < 1)
  {
    return;
  }
  r2 = r * r;
  for (y = -r; y <= r; y++)
  {
    for (x = -r; x <= r; x++)
    {
      if ((x * x + y * y) <= r2)
      {
        plot(fb, w, h, cx + x, cy + y, color);
      }
    }
  }
}

static void stroke_circle(volatile uint16_t *fb, uint16_t w, uint16_t h,
                          int cx, int cy, int r, uint16_t color)
{
  int x, y, inner2, outer2;

  if (r < 1)
  {
    return;
  }
  inner2 = (r - 1) * (r - 1);
  if (inner2 < 0)
  {
    inner2 = 0;
  }
  outer2 = (r + 1) * (r + 1);
  for (y = -r - 1; y <= r + 1; y++)
  {
    for (x = -r - 1; x <= r + 1; x++)
    {
      int d2 = (x * x) + (y * y);
      if ((d2 >= inner2) && (d2 <= outer2))
      {
        plot(fb, w, h, cx + x, cy + y, color);
      }
    }
  }
}

static int same_side(int ax, int ay, int bx, int by, int cx, int cy,
                     int xx, int yy)
{
  int32_t abx = (int32_t)bx - ax;
  int32_t aby = (int32_t)by - ay;
  int32_t c1 = (abx * ((int32_t)cy - ay)) - (aby * ((int32_t)cx - ax));
  int32_t c2 = (abx * ((int32_t)yy - ay)) - (aby * ((int32_t)xx - ax));
  return ((c1 >= 0 && c2 >= 0) || (c1 <= 0 && c2 <= 0)) ? 1 : 0;
}

static void fill_triangle(volatile uint16_t *fb, uint16_t w, uint16_t h,
                          int x0, int y0, int x1, int y1, int x2, int y2,
                          uint16_t color)
{
  int minx, miny, maxx, maxy, x, y;

  minx = x0;
  if (x1 < minx) { minx = x1; }
  if (x2 < minx) { minx = x2; }
  miny = y0;
  if (y1 < miny) { miny = y1; }
  if (y2 < miny) { miny = y2; }
  maxx = x0;
  if (x1 > maxx) { maxx = x1; }
  if (x2 > maxx) { maxx = x2; }
  maxy = y0;
  if (y1 > maxy) { maxy = y1; }
  if (y2 > maxy) { maxy = y2; }
  if (minx < 0) { minx = 0; }
  if (miny < 0) { miny = 0; }
  if (maxx >= (int)w) { maxx = (int)w - 1; }
  if (maxy >= (int)h) { maxy = (int)h - 1; }
  for (y = miny; y <= maxy; y++)
  {
    for (x = minx; x <= maxx; x++)
    {
      if ((same_side(x0, y0, x1, y1, x2, y2, x, y) != 0) &&
          (same_side(x1, y1, x2, y2, x0, y0, x, y) != 0) &&
          (same_side(x2, y2, x0, y0, x1, y1, x, y) != 0))
      {
        fb[((uint32_t)y * w) + (uint32_t)x] = color;
      }
    }
  }
}


static int is_dial_ink(uint16_t p)
{
  uint32_t r, g, b, mx, mn, chroma, luma;

  if (is_red(p) != 0)
  {
    return 0;
  }
  pix_rgb(p, &r, &g, &b);
  luma = (r + (g * 2U) + b) >> 2;
  mx = r;
  if (g > mx) { mx = g; }
  if (b > mx) { mx = b; }
  mn = r;
  if (g < mn) { mn = g; }
  if (b < mn) { mn = b; }
  chroma = mx - mn;
  if ((int)luma > g_dial_ink_luma)
  {
    return 0;
  }
  if ((int)chroma > g_dial_ink_chroma)
  {
    return 0;
  }
  return 1;
}

/* 收集该指针周围环带里、画面上已经印着的黑色刻度盘像素，稍后描成品红，并用这些点拟合刻度、补缺。 */
static int collect_dial_ink(const uint16_t *rgb, uint16_t w, uint16_t h,
                            const meter_pointer_t *p,
                            uint32_t *pts, int max_pts, int npts)
{
  int cx = (int)p->cx;
  int cy = (int)p->cy;
  int r = (int)p->r;
  int rmin;
  int rmax;
  int rmin2;
  int rmax2;
  int x0, y0, x1, y1, x, y;

  if ((r < 2) || (npts >= max_pts))
  {
    return npts;
  }
  rmin = (r * g_dial_rmul_min) / 10;
  rmax = (r * g_dial_rmul_max) / 10;
  if (rmin < g_dial_rmin)
  {
    rmin = g_dial_rmin;
  }
  if (rmax > g_dial_rmax)
  {
    rmax = g_dial_rmax;
  }
  if (rmin < (r + 2))
  {
    rmin = r + 2;
  }
  if (rmax <= rmin)
  {
    return npts;
  }
  rmin2 = rmin * rmin;
  rmax2 = rmax * rmax;
  x0 = cx - rmax;
  y0 = cy - rmax;
  x1 = cx + rmax;
  y1 = cy + rmax;
  if (x0 < 0) { x0 = 0; }
  if (y0 < 0) { y0 = 0; }
  if (x1 > (int)w - 1) { x1 = (int)w - 1; }
  if (y1 > (int)h - 1) { y1 = (int)h - 1; }
  for (y = y0; y <= y1; y++)
  {
    int dy = y - cy;
    int dy2 = dy * dy;
    for (x = x0; x <= x1; x++)
    {
      int dx = x - cx;
      int d2 = (dx * dx) + dy2;
      uint16_t pix;
      if ((d2 < rmin2) || (d2 > rmax2))
      {
        continue;
      }
      pix = rgb[((uint32_t)y * w) + (uint32_t)x];
      if (is_dial_ink(pix) == 0)
      {
        continue;
      }
      if (npts >= max_pts)
      {
        return npts;
      }
      pts[npts] = (((uint32_t)y) << 16) | ((uint32_t)x & 0xFFFFU);
      npts++;
    }
  }
  return npts;
}


typedef struct
{
  int ok;
  int rim;
  int n_ticks;
  int step;
  int have_zero;
  int zero_deg;
  int major_base;
  int tick_deg[20];
  int have_read;
  int tenths;
} dial_fit_t;

static int iabs(int v)
{
  return (v < 0) ? -v : v;
}

static int wrap_deg(int a)
{
  a %= 360;
  if (a < 0)
  {
    a += 360;
  }
  return a;
}

/* 从 from 沿顺时针走到 to 的角度，0~359。屏幕坐标 y 向下时，顺时针等于 atan2 增大。 */
static int deg_cw(int from, int to)
{
  return wrap_deg(to - from);
}

static int deg_to_rad_xy(int deg, int rad, int *ox, int *oy)
{
  float th;
  int r;

  r = rad;
  if (r < 1)
  {
    r = 1;
  }
  th = ((float)wrap_deg(deg)) * 3.14159265f / 180.0f;
  *ox = (int)lroundf(cosf(th) * (float)r);
  *oy = (int)lroundf(sinf(th) * (float)r);
  return r;
}

static void plot_est(volatile uint16_t *fb, uint16_t w, uint16_t h,
                     int x, int y, uint16_t color)
{
  uint16_t cur;
  if ((x < 0) || (y < 0) || (x >= (int)w) || (y >= (int)h))
  {
    return;
  }
  cur = fb[((uint32_t)y * w) + (uint32_t)x];
  /* 已识别出的品红墨、以及蓝色指针轮廓，不再盖成绿色。 */
  if ((cur == g_color_dial) || (cur == PTR_BLUE))
  {
    return;
  }
  fb[((uint32_t)y * w) + (uint32_t)x] = color;
}

static void stroke_line_est(volatile uint16_t *fb, uint16_t w, uint16_t h,
                            int x0, int y0, int x1, int y1, uint16_t color)
{
  int dx, dy, sx, sy, err, e2, x, y;

  dx = iabs(x1 - x0);
  dy = -iabs(y1 - y0);
  sx = (x0 < x1) ? 1 : -1;
  sy = (y0 < y1) ? 1 : -1;
  err = dx + dy;
  x = x0;
  y = y0;
  for (;;)
  {
    plot_est(fb, w, h, x, y, color);
    plot_est(fb, w, h, x + 1, y, color);
    plot_est(fb, w, h, x, y + 1, color);
    if ((x == x1) && (y == y1))
    {
      break;
    }
    e2 = err * 2;
    if (e2 >= dy)
    {
      err += dy;
      x += sx;
    }
    if (e2 <= dx)
    {
      err += dx;
      y += sy;
    }
  }
}

static void stroke_circle_est(volatile uint16_t *fb, uint16_t w, uint16_t h,
                              int cx, int cy, int r, uint16_t color)
{
  int x, y, inner2, outer2;

  if (r < 1)
  {
    return;
  }
  inner2 = (r - 1) * (r - 1);
  if (inner2 < 0)
  {
    inner2 = 0;
  }
  outer2 = (r + 1) * (r + 1);
  for (y = -r - 1; y <= r + 1; y++)
  {
    for (x = -r - 1; x <= r + 1; x++)
    {
      int d2 = (x * x) + (y * y);
      if ((d2 >= inner2) && (d2 <= outer2))
      {
        plot_est(fb, w, h, cx + x, cy + y, color);
      }
    }
  }
}

static void fill_rect(volatile uint16_t *fb, uint16_t w, uint16_t h,
                      int x0, int y0, int x1, int y1, uint16_t color)
{
  int x, y;
  if (x0 < 0) { x0 = 0; }
  if (y0 < 0) { y0 = 0; }
  if (x1 >= (int)w) { x1 = (int)w - 1; }
  if (y1 >= (int)h) { y1 = (int)h - 1; }
  if ((x1 < x0) || (y1 < y0))
  {
    return;
  }
  for (y = y0; y <= y1; y++)
  {
    for (x = x0; x <= x1; x++)
    {
      fb[((uint32_t)y * w) + (uint32_t)x] = color;
    }
  }
}

/* 8x8 点阵，bit7 为左。0-9 随后是 '.' 和 '-'。 */
static const uint8_t FONT8[12][8] = {
  {0x3Cu, 0x66u, 0x66u, 0x66u, 0x66u, 0x66u, 0x66u, 0x3Cu},
  {0x18u, 0x38u, 0x18u, 0x18u, 0x18u, 0x18u, 0x18u, 0x7Eu},
  {0x3Cu, 0x66u, 0x06u, 0x0Cu, 0x18u, 0x30u, 0x60u, 0x7Eu},
  {0x3Cu, 0x66u, 0x06u, 0x1Cu, 0x06u, 0x06u, 0x66u, 0x3Cu},
  {0x0Cu, 0x1Cu, 0x2Cu, 0x4Cu, 0x7Eu, 0x0Cu, 0x0Cu, 0x1Eu},
  {0x7Eu, 0x60u, 0x60u, 0x7Cu, 0x06u, 0x06u, 0x66u, 0x3Cu},
  {0x3Cu, 0x66u, 0x60u, 0x7Cu, 0x66u, 0x66u, 0x66u, 0x3Cu},
  {0x7Eu, 0x06u, 0x0Cu, 0x18u, 0x18u, 0x30u, 0x30u, 0x30u},
  {0x3Cu, 0x66u, 0x66u, 0x3Cu, 0x66u, 0x66u, 0x66u, 0x3Cu},
  {0x3Cu, 0x66u, 0x66u, 0x66u, 0x3Eu, 0x06u, 0x66u, 0x3Cu},
  {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x18u, 0x18u},
  {0x00u, 0x00u, 0x00u, 0x7Eu, 0x7Eu, 0x00u, 0x00u, 0x00u}
};

static int font_index(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (int)(c - '0');
  }
  if (c == '.')
  {
    return 10;
  }
  if (c == '-')
  {
    return 11;
  }
  return -1;
}

static void draw_char_scaled(volatile uint16_t *fb, uint16_t w, uint16_t h,
                             int x, int y, char c, int scale, uint16_t color)
{
  int idx, row, col, yy, xx, si, sj;
  const uint8_t *g;

  idx = font_index(c);
  if ((idx < 0) || (scale < 1))
  {
    return;
  }
  g = FONT8[idx];
  for (row = 0; row < 8; row++)
  {
    for (col = 0; col < 8; col++)
    {
      if ((g[row] & (uint8_t)(0x80u >> col)) == 0U)
      {
        continue;
      }
      for (si = 0; si < scale; si++)
      {
        yy = y + (row * scale) + si;
        for (sj = 0; sj < scale; sj++)
        {
          xx = x + (col * scale) + sj;
          plot(fb, w, h, xx, yy, color);
        }
      }
    }
  }
}

static void draw_text_scaled(volatile uint16_t *fb, uint16_t w, uint16_t h,
                             int x, int y, const char *s, int scale,
                             uint16_t color)
{
  int i;
  if (s == 0)
  {
    return;
  }
  for (i = 0; s[i] != 0; i++)
  {
    draw_char_scaled(fb, w, h, x + (i * 8 * scale), y, s[i], scale, color);
  }
}

/* 在表盘数字位置给 “0” 打分：中间要亮（封闭孔），四周要有墨，左右上下尽量对称。 */
static int score_digit_zero(const uint16_t *rgb, uint16_t w, uint16_t h,
                            int cx, int cy, int rad, int ang)
{
  float th;
  int px, py, s, x, y, t, k;
  int n_c, n_r, sum_c, sum_r, ink_n, contrast, rays, sym, score;
  int ink_q[4];
  int di[8];
  int dj[8];
  uint32_t r, g, b, luma;
  uint16_t p;
  int dx, dy, hit, mean_c, mean_r, s01, s23;

  if (rad < 6)
  {
    return -1000;
  }
  th = ((float)wrap_deg(ang)) * 3.14159265f / 180.0f;
  px = cx + (int)lroundf(cosf(th) * (float)rad);
  py = cy + (int)lroundf(sinf(th) * (float)rad);
  s = rad / 5;
  if (s < 6) { s = 6; }
  if (s > 14) { s = 14; }

  n_c = n_r = sum_c = sum_r = ink_n = 0;
  ink_q[0] = ink_q[1] = ink_q[2] = ink_q[3] = 0;
  for (y = py - s; y <= py + s; y++)
  {
    for (x = px - s; x <= px + s; x++)
    {
      if ((x < 0) || (y < 0) || (x >= (int)w) || (y >= (int)h))
      {
        continue;
      }
      dx = x - px;
      dy = y - py;
      if (((dx * dx) + (dy * dy)) > (s * s))
      {
        continue;
      }
      p = rgb[((uint32_t)y * w) + (uint32_t)x];
      pix_rgb(p, &r, &g, &b);
      luma = (r + (g * 2U) + b) >> 2;
      if (((dx * dx) + (dy * dy)) <= ((s * s) / 9))
      {
        sum_c += (int)luma;
        n_c++;
      }
      else
      {
        sum_r += (int)luma;
        n_r++;
      }
      if (((int)luma <= g_dial_ink_luma) && (is_red(p) == 0))
      {
        ink_n++;
        if ((dx >= 0) && (dy < 0)) { ink_q[0]++; }
        else if ((dx >= 0) && (dy >= 0)) { ink_q[1]++; }
        else if ((dx < 0) && (dy >= 0)) { ink_q[2]++; }
        else { ink_q[3]++; }
      }
    }
  }
  if ((n_c < 4) || (n_r < 8) || (ink_n < 8))
  {
    return -1000;
  }
  mean_c = sum_c / n_c;
  mean_r = sum_r / n_r;
  contrast = mean_c - mean_r;

  di[0] = 1; di[1] = 1; di[2] = 0; di[3] = -1;
  di[4] = -1; di[5] = -1; di[6] = 0; di[7] = 1;
  dj[0] = 0; dj[1] = 1; dj[2] = 1; dj[3] = 1;
  dj[4] = 0; dj[5] = -1; dj[6] = -1; dj[7] = -1;
  rays = 0;
  for (k = 0; k < 8; k++)
  {
    hit = 0;
    for (t = 2; t <= s; t++)
    {
      x = px + (di[k] * t);
      y = py + (dj[k] * t);
      if ((x < 0) || (y < 0) || (x >= (int)w) || (y >= (int)h))
      {
        break;
      }
      p = rgb[((uint32_t)y * w) + (uint32_t)x];
      pix_rgb(p, &r, &g, &b);
      luma = (r + (g * 2U) + b) >> 2;
      if (((int)luma <= g_dial_ink_luma) && (is_red(p) == 0))
      {
        hit = 1;
        break;
      }
    }
    rays += hit;
  }
  s01 = iabs(ink_q[0] - ink_q[2]);
  s23 = iabs(ink_q[1] - ink_q[3]);
  sym = 40 - (s01 + s23);
  if (sym < 0)
  {
    sym = 0;
  }
  if (contrast < 6)
  {
    score = contrast;
  }
  else
  {
    score = (contrast * 3) + (rays * 8) + sym;
  }
  return score;
}

static int is_leftmost_ptr(const meter_pointer_t *ptrs, uint8_t n, uint8_t idx)
{
  uint8_t i;
  int x = (int)ptrs[idx].cx;
  for (i = 0; i < n; i++)
  {
    if ((ptrs[i].locked != 0U) && ((int)ptrs[i].cx < x))
    {
      return 0;
    }
  }
  return 1;
}

/* 按画面位置给四个指针安排位数：最左 ×0.0001，最右 ×0.1，中间靠左 ×0.001、靠右 ×0.01。
 * 返回值是“该盘一个十分之一整刻度”对应的 0.00001 单位数。 */
static int place_units(const meter_pointer_t *ptrs, uint8_t n, uint8_t idx)
{
  uint8_t i;
  int xmin, xmax, x;
  int left, right, mid_lo, mid_hi;

  xmin = 10000;
  xmax = -1;
  left = right = mid_lo = mid_hi = -1;
  for (i = 0; i < n; i++)
  {
    if (ptrs[i].locked == 0U)
    {
      continue;
    }
    x = (int)ptrs[i].cx;
    if (x < xmin)
    {
      xmin = x;
      left = (int)i;
    }
    if (x > xmax)
    {
      xmax = x;
      right = (int)i;
    }
  }
  if ((int)idx == left)
  {
    return 1;
  }
  if ((int)idx == right)
  {
    return 1000;
  }
  for (i = 0; i < n; i++)
  {
    if ((ptrs[i].locked == 0U) || ((int)i == left) || ((int)i == right))
    {
      continue;
    }
    if ((mid_lo < 0) || ((int)ptrs[i].cx < (int)ptrs[mid_lo].cx))
    {
      mid_lo = (int)i;
    }
  }
  for (i = 0; i < n; i++)
  {
    if ((ptrs[i].locked == 0U) || ((int)i == left) || ((int)i == right) ||
        ((int)i == mid_lo))
    {
      continue;
    }
    if ((mid_hi < 0) || ((int)ptrs[i].cx > (int)ptrs[mid_hi].cx))
    {
      mid_hi = (int)i;
    }
  }
  if ((int)idx == mid_lo)
  {
    return 10;
  }
  if ((int)idx == mid_hi)
  {
    return 100;
  }
  return 0;
}

static void fit_dial(const uint16_t *rgb, uint16_t w, uint16_t h,
                     const meter_pointer_t *p,
                     const uint32_t *pts, int beg, int end,
                     int n_ticks, dial_fit_t *out)
{
  int hist_r[161];
  int hist_a[360];
  int i, r, a, k, off, cx, cy, dx, dy, rr, rmin, rmax;
  int best_r, best_rv, v, rv, rim, band, nink, step, best_off, best_s, s;
  int win, peak_a, peak_v, grid, lo, avg, tot, nbin;
  int major_step, zk, zs, sc, zero, digit_r, pang, tk, span, dlt, frac10;
  int a0, a1;
  float d2;

  out->ok = 0;
  out->have_zero = 0;
  out->have_read = 0;
  out->tenths = 0;
  out->n_ticks = n_ticks;
  out->rim = 0;
  out->major_base = 0;
  if ((n_ticks != 10) && (n_ticks != 20))
  {
    return;
  }
  if ((end - beg) < 24)
  {
    return;
  }
  cx = (int)p->cx;
  cy = (int)p->cy;
  rmin = ((int)p->r * g_dial_rmul_min) / 10;
  rmax = ((int)p->r * g_dial_rmul_max) / 10;
  if (rmin < g_dial_rmin) { rmin = g_dial_rmin; }
  if (rmax > g_dial_rmax) { rmax = g_dial_rmax; }
  if (rmin < ((int)p->r + 2)) { rmin = (int)p->r + 2; }
  if (rmax > 160) { rmax = 160; }
  if (rmax <= rmin)
  {
    return;
  }
  for (i = 0; i <= 160; i++)
  {
    hist_r[i] = 0;
  }
  nink = 0;
  for (i = beg; i < end; i++)
  {
    dx = (int)(pts[i] & 0xFFFFU) - cx;
    dy = (int)(pts[i] >> 16) - cy;
    d2 = ((float)dx * (float)dx) + ((float)dy * (float)dy);
    rr = (int)lroundf(sqrtf(d2));
    if ((rr >= rmin) && (rr <= rmax) && (rr <= 160))
    {
      hist_r[rr]++;
      nink++;
    }
  }
  if (nink < 24)
  {
    return;
  }
  /* 3 点平滑后，在偏外的半径上取最强峰，那一圈就是刻度圆弧。 */
  best_r = rmin;
  best_rv = -1;
  for (r = rmin + 1; r < rmax; r++)
  {
    v = hist_r[r - 1] + (hist_r[r] * 2) + hist_r[r + 1];
    /* 偏向外侧，避免把数字圈当成圆弧。 */
    rv = v * r;
    if (rv > best_rv)
    {
      best_rv = rv;
      best_r = r;
    }
  }
  if (best_rv < 8)
  {
    return;
  }
  rim = best_r;
  out->rim = rim;

  for (i = 0; i < 360; i++)
  {
    hist_a[i] = 0;
  }
  band = g_tick_len_major + 2;
  if (band < 6) { band = 6; }
  tot = 0;
  for (i = beg; i < end; i++)
  {
    dx = (int)(pts[i] & 0xFFFFU) - cx;
    dy = (int)(pts[i] >> 16) - cy;
    d2 = ((float)dx * (float)dx) + ((float)dy * (float)dy);
    rr = (int)lroundf(sqrtf(d2));
    if (iabs(rr - rim) > band)
    {
      continue;
    }
    a = wrap_deg((int)lroundf(atan2f((float)dy, (float)dx) * 180.0f / 3.14159265f));
    hist_a[a]++;
    tot++;
  }
  if (tot < 16)
  {
    return;
  }
  /* 角度直方图轻微平滑 */
  {
    int tmp[360];
    for (i = 0; i < 360; i++)
    {
      tmp[i] = hist_a[wrap_deg(i - 1)] + (hist_a[i] * 2) + hist_a[wrap_deg(i + 1)];
    }
    for (i = 0; i < 360; i++)
    {
      hist_a[i] = tmp[i];
    }
  }

  step = 360 / n_ticks;
  best_off = 0;
  best_s = -1;
  for (off = 0; off < step; off++)
  {
    s = 0;
    for (k = 0; k < n_ticks; k++)
    {
      a = wrap_deg(off + (k * step));
      s += hist_a[a];
      s += hist_a[wrap_deg(a - 1)];
      s += hist_a[wrap_deg(a + 1)];
    }
    if (s > best_s)
    {
      best_s = s;
      best_off = off;
    }
  }
  if (best_s < 8)
  {
    return;
  }
  out->step = step;
  out->major_base = 0;
  avg = 0;
  nbin = 0;
  for (i = 0; i < 360; i++)
  {
    avg += hist_a[i];
    nbin++;
  }
  avg = (nbin > 0) ? (avg / nbin) : 0;
  win = step / 3;
  if (win < 2) { win = 2; }
  for (k = 0; k < n_ticks; k++)
  {
    grid = wrap_deg(best_off + (k * step));
    peak_a = grid;
    peak_v = -1;
    for (off = -win; off <= win; off++)
    {
      a = wrap_deg(grid + off);
      if (hist_a[a] > peak_v)
      {
        peak_v = hist_a[a];
        peak_a = a;
      }
    }
    /* 看得见的峰就落到实刻度上；被指针挡住、峰很弱的，用等间隔补出来。 */
    if ((peak_v >= 3) && (peak_v > (avg * 2)))
    {
      out->tick_deg[k] = peak_a;
    }
    else
    {
      out->tick_deg[k] = grid;
    }
  }
  out->ok = 1;

  /* 20 格时，主刻度是墨更多的那一组（偶数或奇数下标），避免把半格当成 0~9。 */
  if (n_ticks == 20)
  {
    int sum_even = 0;
    int sum_odd = 0;
    for (k = 0; k < n_ticks; k += 2)
    {
      sum_even += hist_a[out->tick_deg[k]];
      sum_odd += hist_a[out->tick_deg[k + 1]];
    }
    if (sum_odd > sum_even)
    {
      out->major_base = 1;
    }
  }

  /* 主刻度（10 个数字格）上找印刷的 “0”，作为读数零点。 */
  major_step = (n_ticks == 20) ? (step * 2) : step;
  digit_r = rim - g_tick_len_major - (rim / 6);
  if (digit_r < ((int)p->r + 8))
  {
    digit_r = (int)p->r + 8;
  }
  zk = 0;
  zs = -100000;
  for (k = 0; k < 10; k++)
  {
    if (n_ticks == 20)
    {
      a = out->tick_deg[(out->major_base + (k * 2)) % n_ticks];
    }
    else
    {
      a = out->tick_deg[k];
    }
    sc = score_digit_zero(rgb, w, h, cx, cy, digit_r, a);
    if (sc > zs)
    {
      zs = sc;
      zk = k;
    }
  }
  if (zs >= 20)
  {
    out->have_zero = 1;
    zero = zk;
  }
  else
  {
    /* 分数不够时，退到最靠近画面上方的主刻度（多数表盘 0 在正上）。 */
    int best_d, d, up;
    up = 270;
    best_d = 1000;
    zero = 0;
    for (k = 0; k < 10; k++)
    {
      if (n_ticks == 20)
      {
        a = out->tick_deg[(out->major_base + (k * 2)) % n_ticks];
      }
      else
      {
        a = out->tick_deg[k];
      }
      d = iabs(wrap_deg(a - up));
      if (d > 180)
      {
        d = 360 - d;
      }
      if (d < best_d)
      {
        best_d = d;
        zero = k;
      }
    }
    out->have_zero = 0;
  }
  if (n_ticks == 20)
  {
    out->zero_deg = out->tick_deg[(out->major_base + (zero * 2)) % n_ticks];
  }
  else
  {
    out->zero_deg = out->tick_deg[zero];
  }

  /* 用指针尖相对相邻主刻度的位置读数：落在第 k 与 k+1 格之间，再把这一整格分成 10 份。 */
  if ((p->have_ang != 0U) || ((int)p->tx != 0) || ((int)p->ty != 0))
  {
    pang = wrap_deg((int)lroundf(atan2f((float)((int)p->ty - cy),
                                        (float)((int)p->tx - cx)) * 180.0f / 3.14159265f));
    tk = 0;
    for (k = 0; k < 10; k++)
    {
      int kk = (zero + k) % 10;
      int kn = (zero + k + 1) % 10;
      if (n_ticks == 20)
      {
        a0 = out->tick_deg[(out->major_base + (kk * 2)) % n_ticks];
        a1 = out->tick_deg[(out->major_base + (kn * 2)) % n_ticks];
      }
      else
      {
        a0 = out->tick_deg[kk];
        a1 = out->tick_deg[kn];
      }
      span = deg_cw(a0, a1);
      if (span < 8)
      {
        span = major_step;
      }
      dlt = deg_cw(a0, pang);
      if (dlt < span)
      {
        tk = k;
        frac10 = ((dlt * 10) + (span / 2)) / span;
        if (frac10 >= 10)
        {
          tk = (k + 1) % 10;
          frac10 = 0;
        }
        out->tenths = (tk * 10) + frac10;
        out->have_read = 1;
        break;
      }
    }
  }
}

static void draw_dial_est(volatile uint16_t *fb, uint16_t w, uint16_t h,
                          const meter_pointer_t *p, const dial_fit_t *d)
{
  int k, len, x0, y0, x1, y1, ox, oy;
  int cx, cy, major;

  if ((d->ok == 0) || (d->rim < 4))
  {
    return;
  }
  cx = (int)p->cx;
  cy = (int)p->cy;
  stroke_circle_est(fb, w, h, cx, cy, d->rim, g_color_dial_est);
  major = (d->n_ticks == 20) ? 2 : 1;
  for (k = 0; k < d->n_ticks; k++)
  {
    len = (((k + d->n_ticks - d->major_base) % major) == 0) ? g_tick_len_major : g_tick_len_minor;
    if (len < 2)
    {
      len = 2;
    }
    deg_to_rad_xy(d->tick_deg[k], d->rim, &ox, &oy);
    x0 = cx + ox;
    y0 = cy + oy;
    deg_to_rad_xy(d->tick_deg[k], d->rim - len, &ox, &oy);
    x1 = cx + ox;
    y1 = cy + oy;
    stroke_line_est(fb, w, h, x0, y0, x1, y1, g_color_dial_est);
  }
}

static void format_reading(char *buf, int32_t units, int have)
{
  int32_t ip, frac, p10, i;
  if (have == 0)
  {
    buf[0] = '-';
    buf[1] = '-';
    buf[2] = '.';
    buf[3] = '-';
    buf[4] = '-';
    buf[5] = '-';
    buf[6] = '-';
    buf[7] = '-';
    buf[8] = 0;
    return;
  }
  if (units < 0)
  {
    units = 0;
  }
  ip = units / 100000;
  frac = units % 100000;
  if (ip > 9)
  {
    /* 超出一位整数时仍显示，避免把进位丢了。 */
    buf[0] = (char)('0' + ((ip / 10) % 10));
    buf[1] = (char)('0' + (ip % 10));
    buf[2] = '.';
    i = 3;
  }
  else
  {
    buf[0] = (char)('0' + (int)ip);
    buf[1] = '.';
    i = 2;
  }
  p10 = 10000;
  while (p10 > 0)
  {
    buf[i++] = (char)('0' + (int)((frac / p10) % 10));
    p10 /= 10;
  }
  buf[i] = 0;
}

void MeterLocate_DrawPointers(volatile uint16_t *fb, uint16_t w, uint16_t h,
                              const meter_pointer_t *ptrs, uint8_t n)
{
  enum { DIAL_INK_MAX = 8192, DIAL_MAX = 16 };
  static uint32_t ink_pts[DIAL_INK_MAX];
  static dial_fit_t dials[DIAL_MAX];
  uint8_t i;
  int nink = 0;
  int k;
  int ink_beg[DIAL_MAX];
  int ink_end[DIAL_MAX];
  int32_t total = 0;
  int have_read = 0;
  int n_ticks;
  char text[16];
  int tw, th, scale;
  int pu;

  if ((fb == 0) || (ptrs == 0))
  {
    return;
  }

  /* 先在原图上记下黑色刻度盘像素并拟合刻度，再画指针，最后描品红、用绿色补缺，左上角写读数。 */
  for (i = 0; i < n; i++)
  {
    const meter_pointer_t *p;
    if (i >= DIAL_MAX)
    {
      break;
    }
    ink_beg[i] = nink;
    ink_end[i] = nink;
    dials[i].ok = 0;
    dials[i].have_read = 0;
    p = &ptrs[i];
    if ((p->locked == 0U) || (p->r < 2U))
    {
      continue;
    }
    nink = collect_dial_ink((const uint16_t *)fb, w, h, p, ink_pts, DIAL_INK_MAX, nink);
    ink_end[i] = nink;
    n_ticks = (is_leftmost_ptr(ptrs, n, i) != 0) ? g_tick_n_left : g_tick_n;
    fit_dial((const uint16_t *)fb, w, h, p, ink_pts, ink_beg[i], ink_end[i],
             n_ticks, &dials[i]);
  }

  for (i = 0; i < n; i++)
  {
    const meter_pointer_t *p = &ptrs[i];
    float cx, cy, r, tx, ty, dx, dy, d, alpha, theta;
    int x1, y1, x2, y2;

    if ((p->locked == 0U) || (p->r < 2U))
    {
      continue;
    }
    cx = (float)p->cx;
    cy = (float)p->cy;
    r = (float)p->r;
    tx = (float)p->tx;
    ty = (float)p->ty;
    fill_circle(fb, w, h, (int)p->cx, (int)p->cy, (int)p->r, PTR_BLUE);
    dx = tx - cx;
    dy = ty - cy;
    d = sqrtf((dx * dx) + (dy * dy));
    if (d > (r + 0.5f))
    {
      theta = atan2f(dy, dx);
      {
        float c = r / d;
        if (c > 1.0f) { c = 1.0f; }
        if (c < -1.0f) { c = -1.0f; }
        alpha = acosf(c);
      }
      x1 = (int)lroundf(cx + r * cosf(theta + alpha));
      y1 = (int)lroundf(cy + r * sinf(theta + alpha));
      x2 = (int)lroundf(cx + r * cosf(theta - alpha));
      y2 = (int)lroundf(cy + r * sinf(theta - alpha));
      fill_triangle(fb, w, h, x1, y1, (int)p->tx, (int)p->ty, x2, y2, PTR_BLUE);
    }
    stroke_circle(fb, w, h, (int)p->cx, (int)p->cy, search_radius((int)p->r),
                  g_color_search);
    stroke_circle(fb, w, h, (int)p->cx, (int)p->cy, (int)p->r, g_color_ab);
    if (p->br >= 1U)
    {
      stroke_circle(fb, w, h, (int)p->bx, (int)p->by, (int)p->br, g_color_ab);
    }
  }

  for (k = 0; k < nink; k++)
  {
    int x = (int)(ink_pts[k] & 0xFFFFU);
    int y = (int)(ink_pts[k] >> 16);
    plot(fb, w, h, x, y, g_color_dial);
  }

  for (i = 0; i < n; i++)
  {
    if ((i >= DIAL_MAX) || (ptrs[i].locked == 0U))
    {
      continue;
    }
    draw_dial_est(fb, w, h, &ptrs[i], &dials[i]);
    if (dials[i].have_read != 0)
    {
      pu = place_units(ptrs, n, i);
      if (pu > 0)
      {
        total += (int32_t)dials[i].tenths * (int32_t)pu;
        have_read = 1;
      }
    }
  }

  format_reading(text, total, have_read);
  scale = 3;
  tw = (int)(8 * scale * 8);
  /* "0.12345" 是 7 个字符，两位数整数时 8 个。按实际长度画底。 */
  {
    int len = 0;
    while (text[len] != 0) { len++; }
    tw = 8 * scale * len;
  }
  th = 8 * scale;
  fill_rect(fb, w, h, 4, 4, 4 + tw + 8, 4 + th + 8, g_color_text_bg);
  draw_text_scaled(fb, w, h, 8, 8, text, scale, g_color_text);
}
