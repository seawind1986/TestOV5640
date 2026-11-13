#include "speed_track.h"
#include "dial_font.h"
#include <string.h>

#define RED_RMIN   95
#define RED_DG     16
#define RED_DB     16
#define RED_GMAX   200
#define RED_BMAX   190
#define DARK_Y     115
#define MAX_BLOB   16
#define LIVE_PATH  0x0000U  /* 轨迹圆：黑线 */
#define LIVE_LEAD  0xFFFFU  /* 前沿点：白点 */
#define LIVE_HUB   0x001FU  /* 轴心轮廓：蓝线 */
#define LIVE_CIRC  0x07E0U
#define LIVE_PTR   0x029FU
#define LIVE_STAR  0xFFE0U
#define TXT_R      255
#define TXT_G      0
#define TXT_B      0
#define CROP_MARGIN 80
#define CROP_LEFT_GUARD 360 /* 若第一刀落在右侧邻针，先向左扩出升针再精裁 */
#define NEEDLE_MIN_R 50   /* 升针在当前物距下半径约 80–110，拒掉壳上小红斑 */
#define NEEDLE_MIN_R_LO 30 /* 被裁切的升针可能略小于 50 */
#define NEEDLE_LEFT_GAP 40 /* 左边再有一根明显更靠左的针，就用左边这根 */     /* 贴合升针，但留足边距，避免画面过小 */
#define FLOW_WIN_MIN_US 1000000U  /* 至少跨 1 s 再报流速 */
#define FLOW_WIN_US     2000000U  /* 按约 2 s 窗口滑动 */
#define TIP_HIST        80
#define HUB_LOCK_MDEG   108000   /* 0.3 L = 108°，用针尖轨迹拟合圆心 */
#define RING_SHRINK_N   70       /* 锁定后轨迹半径收进针身，避免扫空 */
#define RING_SHRINK_D   100
#define REV_HUNT_MDEG  300000    /* 走出约 0.83 圈后再找与起点最接近的位置 */
#define REV_PASS_MDEG    4000    /* 比最近点再偏 4°，视为已经越过 */
#define MIN_FPS_X10 20   /* 10 齿/s 的奈奎斯特下限：每秒至少 20 帧 */

/* Inverse rectify (rectified -> captured), copied from dial_read.c; that file is not edited. */
#define HI11  1071152
#define HI12  5069
#define HI13  (-4317748)
#define HI21  5544
#define HI22  1071955
#define HI23  (-5061496)
#define HI31  75697
#define HI32  86945

typedef struct
{
  int32_t n;
  int32_t sx;
  int32_t sy;
  int16_t x0, y0, x1, y1;
} blob_t;

static const uint16_t atan32_mdeg[33] = {
  0, 1790, 3576, 5356, 7125, 8881, 10620, 12339, 14036, 15709,
  17354, 18970, 20556, 22109, 23629, 25115, 26565, 27979, 29358, 30700,
  32005, 33275, 34509, 35707, 36870, 37999, 39094, 40156, 41186, 42184,
  43152, 44091, 45000
};

static const int16_t cosq8[91] = {
  256, 256, 256, 256, 255, 255, 255, 254, 254, 253, 252, 251, 250, 249, 248,
  247, 246, 245, 243, 242, 241, 239, 237, 236, 234, 232, 230, 228, 226, 224,
  222, 219, 217, 215, 212, 210, 207, 204, 202, 199, 196, 193, 190, 187, 184,
  181, 178, 175, 171, 168, 165, 161, 158, 154, 150, 147, 143, 139, 136, 132,
  128, 124, 120, 116, 112, 108, 104, 100, 96, 92, 88, 83, 79, 75, 71, 66, 62,
  58, 53, 49, 44, 40, 36, 31, 27, 22, 18, 13, 9, 4, 0
};

static int16_t icos(int a)
{
  while (a < 0) { a += 360; }
  while (a >= 360) { a -= 360; }
  if (a <= 90) { return cosq8[a]; }
  if (a <= 180) { return (int16_t)(-cosq8[180 - a]); }
  if (a <= 270) { return (int16_t)(-cosq8[a - 180]); }
  return cosq8[360 - a];
}

static int16_t isin(int a)
{
  return icos(a - 90);
}

static uint32_t isqrt32(uint32_t x)
{
  uint32_t r = 0U;
  uint32_t b = 1U << 15;
  uint32_t y;
  if (x >= 0x40000000UL) { b = 1U << 30; }
  while (b != 0U)
  {
    y = r + b;
    if (((uint64_t)y * (uint64_t)y) <= (uint64_t)x) { r = y; }
    b >>= 1;
  }
  return r;
}

static void unpack(uint16_t p, int *r, int *g, int *b)
{
  int r5 = (int)((p >> 11) & 31);
  int g6 = (int)((p >> 5) & 63);
  int b5 = (int)(p & 31);
  *r = (r5 * 255 + 15) / 31;
  *g = (g6 * 255 + 31) / 63;
  *b = (b5 * 255 + 15) / 31;
}

static int is_red(int r, int g, int b)
{
  return (r >= RED_RMIN) && (r > (g + RED_DG)) && (r > (b + RED_DB)) &&
         (g < RED_GMAX) && (b < RED_BMAX);
}

static int luma(int r, int g, int b)
{
  return ((r * 2) + (g * 5) + b) / 8;
}

static uint16_t pix(const uint16_t *rgb, uint16_t stride, int x, int y)
{
  return rgb[((uint32_t)y * (uint32_t)stride) + (uint32_t)x];
}

/* Image convention: y down, result in [0, 360000). */
static int32_t atan2_mdeg(int32_t y, int32_t x)
{
  int32_t ax = (x < 0) ? -x : x;
  int32_t ay = (y < 0) ? -y : y;
  int32_t a;
  if ((ax == 0) && (ay == 0)) { return 0; }
  if (ax >= ay)
  {
    int32_t t = (int32_t)((((int64_t)ay * 32) + (ax / 2)) / ax);
    if (t > 32) { t = 32; }
    a = (int32_t)atan32_mdeg[t];
  }
  else
  {
    int32_t t = (int32_t)((((int64_t)ax * 32) + (ay / 2)) / ay);
    if (t > 32) { t = 32; }
    a = 90000 - (int32_t)atan32_mdeg[t];
  }
  if ((x >= 0) && (y >= 0)) { return a; }
  if ((x < 0) && (y >= 0)) { return 180000 - a; }
  if ((x < 0) && (y < 0)) { return 180000 + a; }
  return 360000 - a;
}

static int32_t wrap_diff_mdeg(int32_t now, int32_t prev)
{
  int32_t d = now - prev;
  if (d > 180000) { d -= 360000; }
  if (d < -180000) { d += 360000; }
  return d;
}

static int norm_mdeg(int32_t a)
{
  while (a < 0) { a += 360000; }
  while (a >= 360000) { a -= 360000; }
  return (int)a;
}

static void inv_rectify_q8(int32_t x_q8, int32_t y_q8, int32_t *sx_q8, int32_t *sy_q8)
{
#if DIAL_RECTIFY_ON
  int64_t X = (((int64_t)HI11 * x_q8) + ((int64_t)HI12 * y_q8) + 128) >> 8;
  int64_t Y = (((int64_t)HI21 * x_q8) + ((int64_t)HI22 * y_q8) + 128) >> 8;
  int64_t D = (((int64_t)HI31 * x_q8) + ((int64_t)HI32 * y_q8) + 128) >> 8;
  int64_t e30;
  int64_t e2;
  int64_t inv;
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

static void map_to_live(int32_t x_q8, int32_t y_q8, int seam, uint8_t mode,
                        int w, int h, int *ox, int *oy)
{
  int32_t sx_q8;
  int32_t sy_q8;
  int32_t x;
  int32_t y;
  int32_t ww = (int32_t)w;
  int32_t hh = (int32_t)h;
  inv_rectify_q8(x_q8, y_q8, &sx_q8, &sy_q8);
  x = (sx_q8 + 128) >> 8;
  y = (sy_q8 + 128) >> 8;
  if ((ww <= 0) || (hh <= 0))
  {
    *ox = (int)x;
    *oy = (int)y;
    return;
  }
  if (seam > 0)
  {
    if (mode == 1U)
    {
      int32_t n = ww * hh;
      int32_t i = (y * ww) + x + (int32_t)seam;
      i %= n;
      if (i < 0) { i += n; }
      *ox = (int)(i % ww);
      *oy = (int)(i / ww);
      return;
    }
    x += (int32_t)seam;
    x %= ww;
    if (x < 0) { x += ww; }
  }
  *ox = (int)x;
  *oy = (int)y;
}

static int bit_get(const uint8_t *b, uint32_t i)
{
  return (int)((b[i >> 3] >> (i & 7U)) & 1U);
}

static void bit_set(uint8_t *b, uint32_t i)
{
  b[i >> 3] = (uint8_t)(b[i >> 3] | (uint8_t)(1U << (i & 7U)));
}

static int flood_blobs(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                       int x0lim, int y0lim, int x1lim, int y1lim,
                       uint8_t *work, uint32_t work_bytes, blob_t *out, int max_out,
                       int mode)
{
  uint16_t dw;
  uint16_t dh;
  uint32_t bits;
  uint8_t *vis;
  uint16_t *qx;
  uint16_t *qy;
  uint32_t qmax;
  int nout = 0;
  int x;
  int y;

  if ((work == 0) || (w < 4U) || (h < 4U)) { return 0; }
  dw = (uint16_t)(w / 2U);
  dh = (uint16_t)(h / 2U);
  bits = (((uint32_t)dw * (uint32_t)dh) + 7U) / 8U;
  if (work_bytes < (bits + 512U)) { return 0; }
  vis = work;
  memset(vis, 0, bits);
  qmax = (work_bytes - ((bits + 1U) & ~1U)) / (2U * sizeof(uint16_t));
  if (qmax > 8192U) { qmax = 8192U; }
  if (qmax < 128U) { return 0; }
  qx = (uint16_t *)(vis + ((bits + 1U) & ~1U));
  qy = qx + qmax;

  if (x0lim < 0) { x0lim = 0; }
  if (y0lim < 0) { y0lim = 0; }
  if (x1lim > (int)w) { x1lim = (int)w; }
  if (y1lim > (int)h) { y1lim = (int)h; }

  for (y = (y0lim & ~1); y < y1lim; y += 2)
  {
    for (x = (x0lim & ~1); x < x1lim; x += 2)
    {
      uint32_t id = (((uint32_t)(y / 2) * (uint32_t)dw) + (uint32_t)(x / 2));
      int r, g, b, ok;
      uint32_t qh;
      uint32_t qt;
      blob_t bl;
      if (bit_get(vis, id)) { continue; }
      unpack(pix(rgb, stride, x, y), &r, &g, &b);
      if (mode == 0) { ok = is_red(r, g, b); }
      else { ok = (luma(r, g, b) < DARK_Y) && (is_red(r, g, b) == 0); }
      if (ok == 0) { continue; }

      qh = 0U;
      qt = 0U;
      qx[0] = (uint16_t)(x / 2);
      qy[0] = (uint16_t)(y / 2);
      bit_set(vis, id);
      qt = 1U;
      memset(&bl, 0, sizeof(bl));
      bl.x0 = (int16_t)w;
      bl.y0 = (int16_t)h;
      while (qh < qt)
      {
        int cx = (int)qx[qh];
        int cy = (int)qy[qh];
        int k;
        int px;
        int py;
        qh++;
        px = cx * 2;
        py = cy * 2;
        unpack(pix(rgb, stride, px, py), &r, &g, &b);
        if (mode == 0) { ok = is_red(r, g, b); }
        else { ok = (luma(r, g, b) < DARK_Y) && (is_red(r, g, b) == 0); }
        if (ok == 0) { continue; }
        bl.n++;
        bl.sx += px;
        bl.sy += py;
        if (px < bl.x0) { bl.x0 = (int16_t)px; }
        if (py < bl.y0) { bl.y0 = (int16_t)py; }
        if (px > bl.x1) { bl.x1 = (int16_t)px; }
        if (py > bl.y1) { bl.y1 = (int16_t)py; }
        for (k = 0; k < 4; k++)
        {
          static const int8_t ddx[4] = { 1, -1, 0, 0 };
          static const int8_t ddy[4] = { 0, 0, 1, -1 };
          int nx = cx + ddx[k];
          int ny = cy + ddy[k];
          uint32_t nid;
          if ((nx < 0) || (ny < 0) || (nx >= (int)dw) || (ny >= (int)dh)) { continue; }
          nid = (((uint32_t)ny * (uint32_t)dw) + (uint32_t)nx);
          if (bit_get(vis, nid)) { continue; }
          bit_set(vis, nid);
          if (qt < qmax)
          {
            qx[qt] = (uint16_t)nx;
            qy[qt] = (uint16_t)ny;
            qt++;
          }
        }
      }
      if ((bl.n >= ((mode == 0) ? 20 : 80)) && (nout < max_out))
      {
        out[nout++] = bl;
      }
    }
  }
  return nout;
}

static int flood_dark(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                      int x0lim, int y0lim, int x1lim, int y1lim,
                      uint8_t *work, uint32_t work_bytes, blob_t *out, int max_out)
{
  return flood_blobs(rgb, stride, w, h, x0lim, y0lim, x1lim, y1lim,
                     work, work_bytes, out, max_out, 1);
}

static int blob_cx(const blob_t *b)
{
  return (b->n > 0) ? (int)(b->sx / b->n) : 0;
}

static int blob_cy(const blob_t *b)
{
  return (b->n > 0) ? (int)(b->sy / b->n) : 0;
}

static int blob_r(const blob_t *b)
{
  int bw = (int)b->x1 - (int)b->x0;
  int bh = (int)b->y1 - (int)b->y0;
  int m = (bw < bh) ? bw : bh;
  return (m / 2);
}

static int count_star_teeth(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                            int cx, int cy, int R, uint8_t *n_out, int32_t *phase_out)
{
  enum { NB = 180 };
  int32_t acc[NB];
  int32_t cnt[NB];
  int a;
  int n;
  int best_n = 4;
  int64_t best_e = -1;
  int32_t best_ph = 0;
  int r0;
  int r1;
  int x;
  int y;

  if (R < 12) { return -1; }
  memset(acc, 0, sizeof(acc));
  memset(cnt, 0, sizeof(cnt));
  r0 = (R * 40) / 100;
  r1 = (R * 95) / 100;
  for (y = cy - R; y <= (cy + R); y++)
  {
    if ((y < 0) || (y >= (int)h)) { continue; }
    for (x = cx - R; x <= (cx + R); x++)
    {
      int dx;
      int dy;
      int d2;
      int r, g, b, rr, bin;
      if ((x < 0) || (x >= (int)w)) { continue; }
      dx = x - cx;
      dy = y - cy;
      d2 = (dx * dx) + (dy * dy);
      if ((d2 < (r0 * r0)) || (d2 > (r1 * r1))) { continue; }
      unpack(pix(rgb, stride, x, y), &r, &g, &b);
      rr = luma(r, g, b);
      bin = (int)(atan2_mdeg(dy, dx) / (360000 / NB));
      if (bin < 0) { bin = 0; }
      if (bin >= NB) { bin = NB - 1; }
      acc[bin] += (DARK_Y + 20 - rr);
      cnt[bin]++;
    }
  }
  for (a = 0; a < NB; a++)
  {
    if (cnt[a] > 0) { acc[a] = acc[a] / cnt[a]; }
    else { acc[a] = 0; }
  }
  for (n = 2; n <= 12; n++)
  {
    int32_t cs = 0;
    int32_t sn = 0;
    int64_t e;
    for (a = 0; a < NB; a++)
    {
      int ang = (n * a * 360) / NB;
      cs += (acc[a] * (int32_t)icos(ang));
      sn += (acc[a] * (int32_t)isin(ang));
    }
    e = ((int64_t)cs * (int64_t)cs) + ((int64_t)sn * (int64_t)sn);
    if (e > best_e)
    {
      best_e = e;
      best_n = n;
      best_ph = atan2_mdeg(sn, cs);
    }
  }
  if (best_e < 1000) { return -1; }
  *n_out = (uint8_t)best_n;
  *phase_out = best_ph;
  return 0;
}

/* Mid-radius ring, n-th harmonic. A few dozen samples — cheap enough at >20 fps. */
static int track_star_phase(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                            const speed_obj_t *s, int32_t *phase_out)
{
  enum { NB = 90 };
  int32_t cs = 0;
  int32_t sn = 0;
  int a;
  int R = s->r;
  int rring;
  int n = (int)s->n_teeth;
  if ((R < 8) || (n < 2)) { return -1; }
  rring = (R * 70) / 100;
  for (a = 0; a < NB; a++)
  {
    int ang = (a * 360) / NB;
    int x = s->cx + (int)(((int32_t)icos(ang) * rring) / 256);
    int y = s->cy + (int)(((int32_t)isin(ang) * rring) / 256);
    int r, g, b, v, nang;
    if ((x < 0) || (y < 0) || (x >= (int)w) || (y >= (int)h)) { continue; }
    unpack(pix(rgb, stride, x, y), &r, &g, &b);
    v = (DARK_Y + 20 - luma(r, g, b));
    nang = (n * ang) % 360;
    cs += v * (int32_t)icos(nang);
    sn += v * (int32_t)isin(nang);
  }
  *phase_out = atan2_mdeg(sn, cs);
  return 0;
}

static void refine_center(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                          int *cx, int *cy, int R, int mode)
{
  int x;
  int y;
  int64_t sx = 0;
  int64_t sy = 0;
  int64_t n = 0;
  int r0 = (R > 8) ? (R + 8) : 16;
  for (y = *cy - r0; y <= (*cy + r0); y++)
  {
    if ((y < 0) || (y >= (int)h)) { continue; }
    for (x = *cx - r0; x <= (*cx + r0); x++)
    {
      int dx;
      int dy;
      int r, g, b, ok;
      if ((x < 0) || (x >= (int)w)) { continue; }
      dx = x - *cx;
      dy = y - *cy;
      if (((dx * dx) + (dy * dy)) > (r0 * r0)) { continue; }
      unpack(pix(rgb, stride, x, y), &r, &g, &b);
      if (mode == 0) { ok = is_red(r, g, b); }
      else { ok = (luma(r, g, b) < DARK_Y) && (is_red(r, g, b) == 0); }
      if (ok == 0) { continue; }
      sx += x;
      sy += y;
      n++;
    }
  }
  if (n >= 20)
  {
    *cx = (int)(sx / n);
    *cy = (int)(sy / n);
  }
}

/* Walk rays in a +/-win_deg window around pred_mdeg (or full circle if pred_mdeg < 0). */
static int needle_tip(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                      int cx, int cy, int R, int32_t pred_mdeg, int win_deg,
                      int *tx, int *ty, int32_t *ang)
{
  int a0;
  int a1;
  int a;
  int r0 = (R * 45) / 100;
  int r1 = R + 4;
  int best_t = -1;
  int64_t sx = 0;
  int64_t sy = 0;
  int64_t n = 0;
  int step = 2;

  if (R < 8) { return -1; }
  if (r0 < 4) { r0 = 4; }
  if (pred_mdeg < 0)
  {
    a0 = 0;
    a1 = 359;
    step = 3;
  }
  else
  {
    int mid = norm_mdeg(pred_mdeg) / 1000;
    a0 = mid - win_deg;
    a1 = mid + win_deg;
  }
  for (a = a0; a <= a1; a += step)
  {
    int t;
    int last_hit = -1;
    for (t = r0; t <= r1; t++)
    {
      int x = cx + (int)(((int32_t)icos(a) * t) / 256);
      int y = cy + (int)(((int32_t)isin(a) * t) / 256);
      int r, g, b;
      if ((x < 0) || (y < 0) || (x >= (int)w) || (y >= (int)h)) { continue; }
      unpack(pix(rgb, stride, x, y), &r, &g, &b);
      if (is_red(r, g, b)) { last_hit = t; }
    }
    if (last_hit > best_t) { best_t = last_hit; }
  }
  if (best_t < r0) { return -1; }
  {
    int cut = best_t - 3;
    if (cut < r0) { cut = r0; }
    for (a = a0; a <= a1; a += step)
    {
      int t;
      for (t = cut; t <= r1; t++)
      {
        int x = cx + (int)(((int32_t)icos(a) * t) / 256);
        int y = cy + (int)(((int32_t)isin(a) * t) / 256);
        int r, g, b;
        if ((x < 0) || (y < 0) || (x >= (int)w) || (y >= (int)h)) { continue; }
        unpack(pix(rgb, stride, x, y), &r, &g, &b);
        if (is_red(r, g, b) == 0) { continue; }
        sx += x;
        sy += y;
        n++;
      }
    }
  }
  if (n < 1) { return -1; }
  *tx = (int)(sx / n);
  *ty = (int)(sy / n);
  *ang = atan2_mdeg(*ty - cy, *tx - cx);
  return 0;
}

static int32_t iir_e2(int32_t filt, int32_t inst, uint32_t dt_us)
{
  uint32_t tau = 300000U;
  int64_t a;
  int64_t b;
  if (dt_us >= tau) { return inst; }
  a = (int64_t)inst * (int64_t)dt_us;
  b = (int64_t)filt * (int64_t)(tau - dt_us);
  return (int32_t)((a + b) / (int64_t)tau);
}

static int32_t iabs32(int32_t v)
{
  return (v < 0) ? -v : v;
}

/* 针尖轨迹，供 J-Link 钩取；走够约 0.3 L 后拟合圆心。 */
int16_t  g_tip_x[TIP_HIST];
int16_t  g_tip_y[TIP_HIST];
uint16_t g_tip_n;
int32_t  g_span_mdeg;
int32_t  g_path_px;
int32_t  g_fit_span_mdeg;
uint32_t g_rev_t0;          /* 一圈测流的起点时刻 */
int32_t  g_rev_ang0;        /* 起点角度 */
int32_t  g_rev_accum;       /* 自起点展开的角度，mdeg */
uint8_t  g_rev_hunt;
int32_t  g_rev_best_d;      /* 与起点的最小角距，mdeg */
int32_t  g_rev_best_accum;
uint32_t g_rev_best_t;
int32_t  g_rev_best_ang;
int16_t  g_locate_cx;
int16_t  g_locate_cy;
int16_t  g_locate_r;
uint16_t g_track_miss;
uint8_t  g_need_relocate;
uint8_t  g_need_refine;
uint8_t  g_n_needles;
int16_t  g_tip_xmax;

static void tip_hist_reset(void)
{
  g_tip_n = 0U;
  g_span_mdeg = 0;
  g_path_px = 0;
  g_fit_span_mdeg = 0;
  memset(g_tip_x, 0, sizeof(g_tip_x));
  memset(g_tip_y, 0, sizeof(g_tip_y));
}

static void rev_start(uint32_t stamp, int32_t ang)
{
  g_rev_t0 = stamp;
  g_rev_ang0 = ang;
  g_rev_accum = 0;
  g_rev_hunt = 0U;
  g_rev_best_d = 180000;
  g_rev_best_accum = 0;
  g_rev_best_t = stamp;
  g_rev_best_ang = ang;
}

/* 用红色列直方图找到升针右侧空隙，避免针尖跟到旁边那个红指针。 */
static int count_red_near(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                          int cx, int cy, int rad);

static int left_red_xmax(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h)
{
  int x, y;
  int ww = (int)w;
  int col[360];
  int start = -1;
  int gap0 = -1;
  int gaplen = 0;
  if (ww > 360) { ww = 360; }
  for (x = 0; x < ww; x++) { col[x] = 0; }
  for (y = 0; y < (int)h; y++)
  {
    for (x = 0; x < ww; x++)
    {
      int r, g, b;
      unpack(pix(rgb, stride, x, y), &r, &g, &b);
      if (is_red(r, g, b)) { col[x]++; }
    }
  }
  for (x = 0; x < ww; x++)
  {
    if (col[x] >= 6)
    {
      if (start < 0) { start = x; }
      gaplen = 0;
    }
    else if (start >= 0)
    {
      if (gaplen == 0) { gap0 = x; }
      gaplen++;
      if (gaplen >= 16)
      {
        int k;
        int saw = 0;
        for (k = x; k < ww; k++)
        {
          if (col[k] >= 6) { saw = 1; break; }
        }
        if (saw != 0) { return (gap0 - 1); }
      }
    }
  }
  if (start < 0) { return ((int)w - 1); }
  return (ww - 1);
}

/* 直接取升针最外端：只在左侧红块里找质心，再取离质心最远的红点。
 * 不绕假定圆心扫圆环，也不跟丢失后的 56 像素窗口跑到邻针。 */
static int free_tip(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                    int approx_x, int approx_y, int search_r,
                    int pred_x, int pred_y,
                    int *tx, int *ty)
{
  int x0, y0, x1, y1;
  int x, y;
  int r, g, b;
  int64_t sx = 0;
  int64_t sy = 0;
  int64_t n = 0;
  int mx, my;
  int best = -1;
  int bx = -1, by = -1;
  int64_t ax = 0, ay = 0, an = 0;
  int xmax = left_red_xmax(rgb, stride, w, h);

  g_tip_xmax = (int16_t)xmax;
  (void)approx_x;
  (void)approx_y;
  (void)search_r;
  (void)pred_x;
  (void)pred_y;
  x0 = 0;
  y0 = 0;
  x1 = xmax;
  y1 = (int)h - 1;
  if (x1 >= (int)w) { x1 = (int)w - 1; }
  if (x1 < 8) { x1 = (int)w - 1; }

  for (y = y0; y <= y1; y++)
  {
    for (x = x0; x <= x1; x++)
    {
      unpack(pix(rgb, stride, x, y), &r, &g, &b);
      if (is_red(r, g, b) == 0) { continue; }
      sx += x;
      sy += y;
      n++;
    }
  }
  if (n < 24) { return -1; }
  mx = (int)(sx / n);
  my = (int)(sy / n);

  /* 先找质心最远端，再找另一端；红点更少的一端是针尖，避免把反光后的针帽当针尖。 */
  for (y = y0; y <= y1; y++)
  {
    for (x = x0; x <= x1; x++)
    {
      int d2;
      unpack(pix(rgb, stride, x, y), &r, &g, &b);
      if (is_red(r, g, b) == 0) { continue; }
      d2 = ((x - mx) * (x - mx)) + ((y - my) * (y - my));
      if (d2 > best) { best = d2; bx = x; by = y; }
    }
  }
  if ((bx < 0) || (best < (18 * 18))) { return -1; }

  {
    int cx2 = -1, cy2 = -1;
    int best2 = -1;
    int da, db;
    int tipx, tipy;
    for (y = y0; y <= y1; y++)
    {
      for (x = x0; x <= x1; x++)
      {
        int d2;
        unpack(pix(rgb, stride, x, y), &r, &g, &b);
        if (is_red(r, g, b) == 0) { continue; }
        d2 = ((x - bx) * (x - bx)) + ((y - by) * (y - by));
        if (d2 > best2) { best2 = d2; cx2 = x; cy2 = y; }
      }
    }
    if ((cx2 < 0) || (best2 < (18 * 18))) { return -1; }
    da = count_red_near(rgb, stride, w, h, bx, by, 7);
    db = count_red_near(rgb, stride, w, h, cx2, cy2, 7);
    if (da <= db) { tipx = bx; tipy = by; }
    else { tipx = cx2; tipy = cy2; }

    /* 针尖附近几个红点取平均，避免单像素抖动。 */
    {
      int thr = ((da <= db) ? best : best2);
      thr = (thr * 85) / 100;
      for (y = y0; y <= y1; y++)
      {
        for (x = x0; x <= x1; x++)
        {
          int d2;
          unpack(pix(rgb, stride, x, y), &r, &g, &b);
          if (is_red(r, g, b) == 0) { continue; }
          d2 = ((x - tipx) * (x - tipx)) + ((y - tipy) * (y - tipy));
          if (d2 > (12 * 12)) { continue; }
          ax += x;
          ay += y;
          an++;
        }
      }
    }
    if (an >= 3)
    {
      *tx = (int)((ax + (an / 2)) / an);
      *ty = (int)((ay + (an / 2)) / an);
    }
    else
    {
      *tx = tipx;
      *ty = tipy;
    }
  }
  return 0;
}

/* 对针尖点列做代数圆拟合（先减质心，再 Kåsa 最小二乘）。 */
static int fit_circle(const int16_t *xs, const int16_t *ys, int n, int *ocx, int *ocy, int *orad)
{
  int i;
  int64_t sx = 0;
  int64_t sy = 0;
  int mx;
  int my;
  int64_t sxx = 0;
  int64_t syy = 0;
  int64_t sxy = 0;
  int64_t sx1 = 0;
  int64_t sy1 = 0;
  int64_t sr2 = 0;
  int64_t sxr = 0;
  int64_t syr = 0;
  int64_t a11, a12, a13, a21, a22, a23, a31, a32, a33;
  int64_t b1, b2, b3;
  int64_t det, detx, dety, detc;
  int64_t A, B, C;
  int cx;
  int cy;
  int64_t sabs = 0;
  int rad;
  int used = 0;

  if (n < 10) { return -1; }
  for (i = 0; i < n; i++)
  {
    sx += xs[i];
    sy += ys[i];
  }
  mx = (int)(sx / n);
  my = (int)(sy / n);
  for (i = 0; i < n; i++)
  {
    int64_t x = (int64_t)xs[i] - mx;
    int64_t y = (int64_t)ys[i] - my;
    int64_t r2 = (x * x) + (y * y);
    sxx += x * x;
    syy += y * y;
    sxy += x * y;
    sx1 += x;
    sy1 += y;
    sr2 += r2;
    sxr += x * r2;
    syr += y * r2;
  }
  /* 2 cx x + 2 cy y + C = x^2+y^2，未知数取 A=2cx, B=2cy, C。 */
  a11 = sxx; a12 = sxy; a13 = sx1;
  a21 = sxy; a22 = syy; a23 = sy1;
  a31 = sx1; a32 = sy1; a33 = (int64_t)n;
  b1 = sxr; b2 = syr; b3 = sr2;
  det = a11 * (a22 * a33 - a23 * a32)
      - a12 * (a21 * a33 - a23 * a31)
      + a13 * (a21 * a32 - a22 * a31);
  if (det == 0) { return -1; }
  detx = b1 * (a22 * a33 - a23 * a32)
       - a12 * (b2 * a33 - a23 * b3)
       + a13 * (b2 * a32 - a22 * b3);
  dety = a11 * (b2 * a33 - a23 * b3)
       - b1 * (a21 * a33 - a23 * a31)
       + a13 * (a21 * b3 - b2 * a31);
  detc = a11 * (a22 * b3 - b2 * a32)
       - a12 * (a21 * b3 - b2 * a31)
       + b1 * (a21 * a32 - a22 * a31);
  A = detx / det;
  B = dety / det;
  C = detc / det;
  /* A=2cx, B=2cy，相对质心。 */
  cx = mx + (int)((A + ((A >= 0) ? 1 : -1)) / 2);
  cy = my + (int)((B + ((B >= 0) ? 1 : -1)) / 2);
  for (i = 0; i < n; i++)
  {
    int64_t dx = (int64_t)xs[i] - cx;
    int64_t dy = (int64_t)ys[i] - cy;
    int d = (int)isqrt32((uint32_t)((dx * dx) + (dy * dy)));
    sabs += d;
    used++;
  }
  if (used < 10) { return -1; }
  rad = (int)(sabs / used);
  if ((rad < 50) || (rad > 220)) { return -1; }
  /* 直径必须覆盖点列最长弦，否则短弧会被拟成偏向弧段的小圆。 */
  {
    int j;
    int chord2 = 0;
    int64_t eres = 0;
    for (i = 0; i < n; i++)
    {
      for (j = i + 1; j < n; j++)
      {
        int dx = (int)xs[i] - (int)xs[j];
        int dy = (int)ys[i] - (int)ys[j];
        int d2 = (dx * dx) + (dy * dy);
        if (d2 > chord2) { chord2 = d2; }
      }
    }
    if ((rad * 2) < (int)isqrt32((uint32_t)chord2)) { return -1; }
    for (i = 0; i < n; i++)
    {
      int64_t dx = (int64_t)xs[i] - cx;
      int64_t dy = (int64_t)ys[i] - cy;
      int d = (int)isqrt32((uint32_t)((dx * dx) + (dy * dy)));
      eres += (d > rad) ? (d - rad) : (rad - d);
    }
    if ((eres / used) > 3) { return -1; }
  }
  *ocx = cx;
  *ocy = cy;
  *orad = rad;
  (void)C;
  return 0;
}

void SpeedTrack_Reset(speed_state_t *st)
{
  if (st == 0) { return; }
  memset(st, 0, sizeof(*st));
  tip_hist_reset();
  rev_start(0U, 0);
  g_locate_cx = 0;
  g_locate_cy = 0;
  g_locate_r = 0;
  g_track_miss = 0U;
  g_need_relocate = 0U;
  g_need_refine = 0U;
  g_n_needles = 0U;
}

void SpeedTrack_Shift(speed_state_t *st, int16_t dx, int16_t dy)
{
  if (st == 0) { return; }
  st->needle.cx = (int16_t)(st->needle.cx + dx);
  st->needle.cy = (int16_t)(st->needle.cy + dy);
  st->needle.tip_x = (int16_t)(st->needle.tip_x + dx);
  st->needle.tip_y = (int16_t)(st->needle.tip_y + dy);
  st->star.cx = (int16_t)(st->star.cx + dx);
  st->star.cy = (int16_t)(st->star.cy + dy);
  st->crop.x = 0;
  st->crop.y = 0;
  st->have_prev = 0U;
  st->have_speed = 0U;
  st->hub_locked = 0;
  st->dir_sign = 0;
  tip_hist_reset();
  rev_start(0U, 0);
  g_locate_cx = st->needle.cx;
  g_locate_cy = st->needle.cy;
  g_locate_r = st->needle.r;
  g_track_miss = 0U;
}

int SpeedTrack_RefineCrop(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                          uint8_t *work, uint32_t work_bytes, speed_state_t *st,
                          int16_t *ox, int16_t *oy, uint16_t *nw, uint16_t *nh)
{
  int pad;
  int tw;
  int th;
  int x0;
  int y0;
  int need = 0;

  if (ox != 0) { *ox = 0; }
  if (oy != 0) { *oy = 0; }
  if (nw != 0) { *nw = w; }
  if (nh != 0) { *nh = h; }
  if (SpeedTrack_FindNeedle(rgb, stride, w, h, work, work_bytes, st) != 0)
  {
    return -1;
  }
  g_locate_cx = st->needle.cx;
  g_locate_cy = st->needle.cy;
  g_locate_r = st->needle.r;
  pad = (int)st->needle.r + CROP_MARGIN;
  if (pad < 80) { pad = 80; }
  tw = pad * 2;
  th = pad * 2;
  if (tw > 1024) { tw = 1024; }
  if (th > 600) { th = 600; }
  tw &= ~1;
  th &= ~1;
  if (tw < 80) { tw = 80; }
  if (th < 80) { th = 80; }
  x0 = (int)st->needle.cx - (tw / 2);
  y0 = (int)st->needle.cy - (th / 2);
  x0 &= ~1;
  y0 &= ~1;
  /* 第一刀常是加宽窗口，即使邻针不贴边也要收成以最左针为中心的紧窗。 */
  if ((x0 > -8) && (x0 < 8) && (y0 > -8) && (y0 < 8) &&
      (((int)w - tw) > -16) && (((int)w - tw) < 16) &&
      (((int)h - th) > -16) && (((int)h - th) < 16))
  {
    return 0;
  }
  if (ox != 0) { *ox = (int16_t)x0; }
  if (oy != 0) { *oy = (int16_t)y0; }
  if (nw != 0) { *nw = (uint16_t)tw; }
  if (nh != 0) { *nh = (uint16_t)th; }
  (void)need;
  return 1;
}


static int count_red_near(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                          int cx, int cy, int rad)
{
  int x, y, n = 0;
  int r, g, b;
  for (y = cy - rad; y <= (cy + rad); y++)
  {
    if ((y < 0) || (y >= (int)h)) { continue; }
    for (x = cx - rad; x <= (cx + rad); x++)
    {
      if ((x < 0) || (x >= (int)w)) { continue; }
      if ((((x - cx) * (x - cx)) + ((y - cy) * (y - cy))) > (rad * rad)) { continue; }
      unpack(pix(rgb, stride, x, y), &r, &g, &b);
      if (is_red(r, g, b)) { n++; }
    }
  }
  return n;
}

/* 从红色连通块里取出针根和针尖，枢纽沿轴线往针尖收一点。 */
static int pointer_from_blob(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                             const blob_t *bl, int *hx, int *hy, int *tx, int *ty, int *pr)
{
  int x, y;
  int x0 = (int)bl->x0 - 2;
  int y0 = (int)bl->y0 - 2;
  int x1 = (int)bl->x1 + 2;
  int y1 = (int)bl->y1 + 2;
  int mx = blob_cx(bl);
  int my = blob_cy(bl);
  int ax = -1, ay = -1, bx = -1, by = -1;
  int32_t best = -1;
  int da, db;
  int hx0, hy0, tx0, ty0, rr;

  if (x0 < 0) { x0 = 0; }
  if (y0 < 0) { y0 = 0; }
  if (x1 >= (int)w) { x1 = (int)w - 1; }
  if (y1 >= (int)h) { y1 = (int)h - 1; }

  for (y = y0; y <= y1; y++)
  {
    for (x = x0; x <= x1; x++)
    {
      int r, g, b;
      int32_t d2;
      unpack(pix(rgb, stride, x, y), &r, &g, &b);
      if (is_red(r, g, b) == 0) { continue; }
      d2 = (int32_t)(x - mx) * (x - mx) + (int32_t)(y - my) * (y - my);
      if (d2 > best) { best = d2; ax = x; ay = y; }
    }
  }
  if ((ax < 0) || (best < (20 * 20))) { return -1; }

  best = -1;
  for (y = y0; y <= y1; y++)
  {
    for (x = x0; x <= x1; x++)
    {
      int r, g, b;
      int32_t d2;
      unpack(pix(rgb, stride, x, y), &r, &g, &b);
      if (is_red(r, g, b) == 0) { continue; }
      d2 = (int32_t)(x - ax) * (x - ax) + (int32_t)(y - ay) * (y - ay);
      if (d2 > best) { best = d2; bx = x; by = y; }
    }
  }
  if ((bx < 0) || (best < (20 * 20))) { return -1; }

  da = count_red_near(rgb, stride, w, h, ax, ay, 7);
  db = count_red_near(rgb, stride, w, h, bx, by, 7);
  if (da <= db) { tx0 = ax; ty0 = ay; hx0 = bx; hy0 = by; }
  else { tx0 = bx; ty0 = by; hx0 = ax; hy0 = ay; }

  /* 真枢轴在针根往针尖约 18% 处，轨迹半径用枢纽到针尖。 */
  hx0 = hx0 + (((tx0 - hx0) * 18) / 100);
  hy0 = hy0 + (((ty0 - hy0) * 18) / 100);
  rr = (int)isqrt32((uint32_t)((tx0 - hx0) * (tx0 - hx0) + (ty0 - hy0) * (ty0 - hy0)));
  if (rr < 24) { return -1; }
  if (rr > 180) { return -1; }
  /* 在略小于针尖的半径上跟前沿，红弧更宽、更稳。 */
  *pr = (rr * 85) / 100;
  if (*pr < 20) { *pr = 20; }
  *hx = hx0;
  *hy = hy0;
  *tx = tx0;
  *ty = ty0;
  return 0;
}

int SpeedTrack_FindNeedle(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                          uint8_t *work, uint32_t work_bytes, speed_state_t *st)
{
  blob_t red[MAX_BLOB];
  int n;
  int i;
  int best = -1;
  int best_x = 0x7FFFFFFF;
  int hx, hy, tx, ty, rr;

  if ((rgb == 0) || (st == 0)) { return -1; }
  n = flood_blobs(rgb, stride, w, h, 0, 0, (int)w, (int)h,
                  work, work_bytes, red, MAX_BLOB, 0);
  g_n_needles = 0U;
  {
    int cand_hx[MAX_BLOB];
    int cand_hy[MAX_BLOB];
    int cand_tx[MAX_BLOB];
    int cand_ty[MAX_BLOB];
    int cand_rr[MAX_BLOB];
    int nc = 0;
    int n_big = 0;
    int left = -1;
    int left_big = -1;
    int left_x = 0x7FFFFFFF;
    int left_big_x = 0x7FFFFFFF;

    for (i = 0; i < n; i++)
    {
      if (pointer_from_blob(rgb, stride, w, h, &red[i], &hx, &hy, &tx, &ty, &rr) != 0)
      {
        continue;
      }
      if (rr < NEEDLE_MIN_R_LO)
      {
        continue;
      }
      if (nc < MAX_BLOB)
      {
        cand_hx[nc] = hx;
        cand_hy[nc] = hy;
        cand_tx[nc] = tx;
        cand_ty[nc] = ty;
        cand_rr[nc] = rr;
        if (hx < left_x)
        {
          left_x = hx;
          left = nc;
        }
        if (rr >= NEEDLE_MIN_R)
        {
          n_big++;
          if (hx < left_big_x)
          {
            left_big_x = hx;
            left_big = nc;
          }
        }
        nc++;
      }
    }
    g_n_needles = (uint8_t)n_big;
    /* 优先用合格半径里最左边的；若再左边还有一根略小的（多半是被裁切的升针），改用那根。 */
    best = left_big;
    if ((left >= 0) && (left_big >= 0) &&
        ((left_big_x - left_x) >= NEEDLE_LEFT_GAP))
    {
      best = left;
    }
    else if (best < 0)
    {
      best = left;
    }
    if (best >= 0)
    {
      st->needle.found = 1U;
      st->needle.cx = (int16_t)cand_hx[best];
      st->needle.cy = (int16_t)cand_hy[best];
      st->needle.r = (int16_t)cand_rr[best];
      st->needle.hub_r = (int16_t)((cand_rr[best] * 30) / 85);
      if (st->needle.hub_r < 8) { st->needle.hub_r = 8; }
      st->needle.tip_x = (int16_t)cand_tx[best];
      st->needle.tip_y = (int16_t)cand_ty[best];
      st->needle.ang_mdeg = atan2_mdeg(cand_ty[best] - cand_hy[best],
                                       cand_tx[best] - cand_hx[best]);
    }
  }
  if ((best < 0) || (st->needle.found == 0U)) { return -1; }
  return 0;
}

int SpeedTrack_TakeNeedle(const dial_report_t *rep, speed_state_t *st)
{
  const dial_info_t *d = 0;
  uint8_t i;
  int hx, hy, tx, ty, r, hr;
  int seam;
  uint8_t mode;
  int fw;
  int fh;

  if ((rep == 0) || (st == 0)) { return -1; }
  for (i = 0U; i < rep->n_dials; i++)
  {
    if (rep->dial[i].place == 3)
    {
      d = &rep->dial[i];
      break;
    }
  }
  if (d == 0) { return -1; }

  seam = (int)rep->seam_col;
  mode = rep->seam_mode;
  fw = 1024;
  fh = 600;
  map_to_live(d->cx_q8, d->cy_q8, seam, mode, fw, fh, &hx, &hy);
  map_to_live(d->tip_x_q8, d->tip_y_q8, seam, mode, fw, fh, &tx, &ty);
  r = (int)((d->r_q8 + 128) >> 8);
  hr = (int)d->hub_r;
  if (r < 20) { r = 20; }
  if (hr < 8) { hr = r / 3; }

  st->needle.found = 1U;
  st->needle.cx = (int16_t)hx;
  st->needle.cy = (int16_t)hy;
  st->needle.r = (int16_t)r;
  st->needle.hub_r = (int16_t)hr;
  st->needle.tip_x = (int16_t)tx;
  st->needle.tip_y = (int16_t)ty;
  st->needle.ang_mdeg = atan2_mdeg(ty - hy, tx - hx);
  return 0;
}

int SpeedTrack_FindStar(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                        uint8_t *work, uint32_t work_bytes, speed_state_t *st)
{
  blob_t dark[MAX_BLOB];
  int nd;
  int i;
  int best = -1;
  int32_t best_d2 = 0x7FFFFFFF;
  int hx, hy, hr;
  int x0, y0, x1, y1;
  int cx, cy, R;
  uint8_t nteeth = 0;
  int32_t phase = 0;

  if ((rgb == 0) || (st == 0) || (st->needle.found == 0U)) { return -1; }
  hx = st->needle.cx;
  hy = st->needle.cy;
  hr = st->needle.r + 16;
  x0 = hx - 280;
  x1 = hx + 280;
  y0 = 80;
  y1 = hy + 40;
  nd = flood_dark(rgb, stride, w, h, x0, y0, x1, y1, work, work_bytes, dark, MAX_BLOB);
  for (i = 0; i < nd; i++)
  {
    int bcx = blob_cx(&dark[i]);
    int bcy = blob_cy(&dark[i]);
    int br = blob_r(&dark[i]);
    int bw = (int)dark[i].x1 - (int)dark[i].x0 + 1;
    int bh = (int)dark[i].y1 - (int)dark[i].y0 + 1;
    int dx = bcx - hx;
    int dy = bcy - hy;
    int32_t d2 = ((int32_t)dx * dx) + ((int32_t)dy * dy);
    if (dark[i].n < 200) { continue; }
    if ((br < 30) || (br > 120)) { continue; }
    if ((bw * 3 < bh * 2) || (bh * 3 < bw * 2)) { continue; }
    if (d2 < (int32_t)((hr + 20) * (hr + 20))) { continue; }
    if (d2 > (320 * 320)) { continue; }
    if (d2 < best_d2)
    {
      best_d2 = d2;
      best = i;
    }
  }
  if (best < 0) { return -1; }
  cx = blob_cx(&dark[best]);
  cy = blob_cy(&dark[best]);
  R = blob_r(&dark[best]);
  refine_center(rgb, stride, w, h, &cx, &cy, R, 1);
  if (count_star_teeth(rgb, stride, w, h, cx, cy, R, &nteeth, &phase) != 0)
  {
    nteeth = 4U;
    phase = 0;
  }
  st->star.found = 1U;
  st->star.n_teeth = nteeth;
  st->star.cx = (int16_t)cx;
  st->star.cy = (int16_t)cy;
  st->star.r = (int16_t)R;
  st->star.phase_mdeg = phase;
  return 0;
}

void SpeedTrack_FinishLocate(speed_state_t *st, uint16_t frame_w, uint16_t frame_h)
{
  int x0, y0, x1, y1;
  int m = CROP_MARGIN;

  if ((st == 0) || (st->needle.found == 0U)) { return; }
  x0 = st->needle.cx - st->needle.r - m - CROP_LEFT_GUARD;
  y0 = st->needle.cy - st->needle.r - m;
  x1 = st->needle.cx + st->needle.r + m;
  y1 = st->needle.cy + st->needle.r + m;
  if (x0 < 0) { x0 = 0; }
  if (y0 < 0) { y0 = 0; }
  if (x1 > ((int)frame_w - 1)) { x1 = (int)frame_w - 1; }
  if (y1 > ((int)frame_h - 1)) { y1 = (int)frame_h - 1; }
  x0 &= ~1;
  y0 &= ~1;
  x1 = (x1 + 1) & ~1;
  y1 = (y1 + 1) & ~1;
  if ((x1 - x0) < 80) { x1 = x0 + 80; }
  if ((y1 - y0) < 80) { y1 = y0 + 80; }
  if (x1 > (int)frame_w)
  {
    x0 -= (x1 - (int)frame_w);
    x1 = (int)frame_w;
    if (x0 < 0) { x0 = 0; }
  }
  if (y1 > (int)frame_h)
  {
    y0 -= (y1 - (int)frame_h);
    y1 = (int)frame_h;
    if (y0 < 0) { y0 = 0; }
  }
  st->crop.x = (uint16_t)x0;
  st->crop.y = (uint16_t)y0;
  st->crop.w = (uint16_t)(x1 - x0);
  st->crop.h = (uint16_t)(y1 - y0);
  st->loc_ok = 1U;
  st->track_ok = 1U;
}


/* 在冻结的轨迹圆上找红弧沿运动方向的前沿。dir=0 时先扫整圈取最长红弧。 */
static int ring_lead(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                     int cx, int cy, int R, int32_t pred_mdeg, int32_t dir,
                     int *lx, int *ly, int32_t *ang)
{
  uint8_t red[360];
  int a;
  int a0;
  int a1;
  int i;
  int n;
  int best_len = 0;
  int best_s = 0;
  int lead;
  int wrap_scan;

  if (R < 8) { return -1; }
  if ((pred_mdeg < 0) || (dir == 0))
  {
    a0 = 0;
    a1 = 359;
  }
  else
  {
    int mid = norm_mdeg(pred_mdeg) / 1000;
    int back = 15;
    int ahead = 40;
    if (dir > 0) { a0 = mid - back; a1 = mid + ahead; }
    else { a0 = mid - ahead; a1 = mid + back; }
  }

  n = 0;
  memset(red, 0, sizeof(red));
  for (a = a0; a <= a1; a++)
  {
    int aa = a;
    int x, y, r, g, b;
    while (aa < 0) { aa += 360; }
    while (aa >= 360) { aa -= 360; }
    x = cx + (int)(((int32_t)icos(aa) * R) / 256);
    y = cy + (int)(((int32_t)isin(aa) * R) / 256);
    if ((x < 0) || (y < 0) || (x >= (int)w) || (y >= (int)h)) { continue; }
    unpack(pix(rgb, stride, x, y), &r, &g, &b);
    red[aa] = (uint8_t)(is_red(r, g, b) ? 1U : 0U);
    n++;
  }
  if (n < 8) { return -1; }

  /* 填 1～2 度的小孔，避免红弧被拆开。 */
  for (i = 0; i < 360; i++)
  {
    int p = (i + 359) % 360;
    int q = (i + 1) % 360;
    if ((red[i] == 0U) && (red[p] != 0U) && (red[q] != 0U)) { red[i] = 1U; }
  }
  for (i = 0; i < 360; i++)
  {
    int p = (i + 359) % 360;
    int q = (i + 2) % 360;
    int m = (i + 1) % 360;
    if ((red[i] == 0U) && (red[p] != 0U) && (red[q] != 0U))
    {
      red[i] = 1U;
      red[m] = 1U;
    }
  }

  wrap_scan = ((a0 == 0) && (a1 == 359)) ? 1 : 0;
  i = 0;
  while (i < 360)
  {
    int s;
    int e;
    int len;
    if (red[i] == 0U) { i++; continue; }
    s = i;
    e = i;
    while ((e < 359) && (red[e + 1] != 0U)) { e++; }
    if ((wrap_scan != 0) && (s == 0) && (red[359] != 0U))
    {
      /* 整圈扫描时，把跨 0° 的弧并到最后一段去处理。 */
      i = e + 1;
      continue;
    }
    len = e - s + 1;
    if ((wrap_scan != 0) && (e == 359) && (red[0] != 0U))
    {
      int j = 0;
      while ((j < s) && (red[j] != 0U)) { j++; }
      len = (e - s + 1) + j;
      e = j - 1;
    }
    if (len > best_len)
    {
      best_len = len;
      best_s = s;
      /* stash end in best via encode: keep lead later from s/len */
      if (e < s) { best_s = s; /* wrap, end = e */ }
    }
    /* store end in red as we need both; use a second pass on winner */
    i = (e >= s) ? (e + 1) : 360;
  }

  /* 再扫一次拿到最长弧的起止角。 */
  {
    int win_s = 0, win_e = 0;
    best_len = 0;
    i = 0;
    while (i < 360)
    {
      int s, e, len;
      if (red[i] == 0U) { i++; continue; }
      s = i;
      e = i;
      while ((e < 359) && (red[e + 1] != 0U)) { e++; }
      len = e - s + 1;
      if ((wrap_scan != 0) && (e == 359) && (red[0] != 0U))
      {
        int j = 0;
        while ((j < 360) && (red[j] != 0U) && (j < s)) { j++; }
        if (j > 0)
        {
          len = (e - s + 1) + j;
          e = j - 1;
        }
      }
      if ((wrap_scan != 0) && (s == 0) && (red[359] != 0U) && (e < 359))
      {
        i = e + 1;
        continue;
      }
      if (len > best_len)
      {
        best_len = len;
        win_s = s;
        win_e = e;
      }
      i = (e >= s) ? (e + 1) : 360;
    }
    if (best_len < 3) { return -1; }
    if (dir > 0) { lead = win_e; }
    else if (dir < 0) { lead = win_s; }
    else { lead = (win_e >= win_s) ? ((win_s + win_e) / 2) : (((win_s + win_e + 360) / 2) % 360); }
  }

  *lx = cx + (int)(((int32_t)icos(lead) * R) / 256);
  *ly = cy + (int)(((int32_t)isin(lead) * R) / 256);
  *ang = lead * 1000;
  (void)best_s;
  return 0;
}

int SpeedTrack_Tick(const uint16_t *rgb, uint16_t stride, uint16_t w, uint16_t h,
                    uint32_t stamp_us, speed_state_t *st)
{
  int hx, hy, tx, ty, R;
  int32_t ang;
  uint32_t step;
  int tip_ok;

  if ((rgb == 0) || (st == 0) || (st->loc_ok == 0U)) { return -1; }
  hx = st->needle.cx;
  hy = st->needle.cy;
  R = st->needle.r;

  if (st->hub_locked == 0)
  {
    /* 锁定前：直接找红色指针最外端，不绕估错的圆心扫圆环。 */
    {
      int pred_x = (st->have_prev != 0U) ? (int)st->needle.tip_x : -1;
      int pred_y = (st->have_prev != 0U) ? (int)st->needle.tip_y : -1;
      int sr = (R > 16) ? (R + 60) : 140;
      tip_ok = free_tip(rgb, stride, w, h, hx, hy, sr, pred_x, pred_y, &tx, &ty);
      if (tip_ok != 0)
      {
        tip_ok = free_tip(rgb, stride, w, h, hx, hy, 160, -1, -1, &tx, &ty);
      }
      if (tip_ok == 0)
      {
        ang = atan2_mdeg(ty - hy, tx - hx);
      }
    }
  }
  else
  {
    /* 锁定后：在收进针身的轨迹圆上跟运动方向前沿。 */
    tip_ok = ring_lead(rgb, stride, w, h, hx, hy, R,
                       (st->have_prev != 0U) ? st->last_ang : -1, st->dir_sign,
                       &tx, &ty, &ang);
    if (tip_ok != 0)
    {
      tip_ok = ring_lead(rgb, stride, w, h, hx, hy, R, -1, 0, &tx, &ty, &ang);
    }
  }

  if (tip_ok == 0)
  {
    st->needle.tip_x = (int16_t)tx;
    st->needle.tip_y = (int16_t)ty;
    st->needle.ang_mdeg = ang;
    st->needle.found = 1U;
  }
  else
  {
    st->needle.found = 0U;
  }
  st->track_ok = st->needle.found;
  if (st->needle.found == 0U)
  {
    g_track_miss++;
    if (st->hub_locked != 0)
    {
      if (g_track_miss >= 25U)
      {
        /* 跟丢则退回定位时的枢纽，重新描针尖，避免钉在错误小圆上。 */
        st->hub_locked = 0;
        st->have_speed = 0U;
        st->have_prev = 0U;
        st->needle.cx = g_locate_cx;
        st->needle.cy = g_locate_cy;
        st->needle.r = g_locate_r;
        tip_hist_reset();
        rev_start(0U, 0);
        g_track_miss = 0U;
      }
    }
    else if (g_track_miss >= 40U)
    {
      /* 裁窗里一直没有红针，回到全幅重新定位。 */
      g_need_relocate = 1U;
    }
    return 0;
  }
  g_track_miss = 0U;

  if (st->have_prev == 0U)
  {
    st->stamp_us = stamp_us;
    st->last_ang = st->needle.ang_mdeg;
    st->win_t0 = stamp_us;
    st->win_accum = 0;
    st->mid_t = 0U;
    st->mid_accum = 0;
    st->dir_sign = 0;
    st->have_prev = 1U;
    st->have_speed = 0U;
    st->dt_us = 0U;
    if (st->hub_locked == 0)
    {
      g_tip_x[0] = (int16_t)tx;
      g_tip_y[0] = (int16_t)ty;
      g_tip_n = 1U;
      g_span_mdeg = 0;
    }
    return 0;
  }

  step = stamp_us - st->stamp_us;
  if (step < 2000U)
  {
    return 0;
  }
  if (step > 2000000U)
  {
    st->stamp_us = stamp_us;
    st->last_ang = st->needle.ang_mdeg;
    st->win_t0 = stamp_us;
    st->win_accum = 0;
    st->mid_t = 0U;
    st->mid_accum = 0;
    if (st->hub_locked == 0)
    {
      st->have_speed = 0U;
      tip_hist_reset();
      g_tip_x[0] = (int16_t)tx;
      g_tip_y[0] = (int16_t)ty;
      g_tip_n = 1U;
    }
    else
    {
      rev_start(stamp_us, st->needle.ang_mdeg);
    }
    return 0;
  }
  st->stamp_us = stamp_us;

  {
    int32_t dang = wrap_diff_mdeg(st->needle.ang_mdeg, st->last_ang);
    st->last_ang = st->needle.ang_mdeg;
    if ((st->dir_sign == 0) && (iabs32(dang) >= 2000))
    {
      st->dir_sign = (dang > 0) ? 1 : -1;
    }

    if (st->hub_locked == 0)
    {
      /* 针尖走出 2 像素再记一点。缓冲区满仍未锁定时丢掉旧的一半，继续描。 */
      if (g_tip_n >= (uint16_t)TIP_HIST)
      {
        uint16_t keep = (uint16_t)(TIP_HIST / 2);
        uint16_t i;
        uint16_t off = (uint16_t)(TIP_HIST - (int)keep);
        for (i = 0U; i < keep; i++)
        {
          g_tip_x[i] = g_tip_x[i + off];
          g_tip_y[i] = g_tip_y[i + off];
        }
        g_tip_n = keep;
      }
      if (g_tip_n < (uint16_t)TIP_HIST)
      {
        int rec = 1;
        if (g_tip_n > 0U)
        {
          int ldx = tx - (int)g_tip_x[g_tip_n - 1U];
          int ldy = ty - (int)g_tip_y[g_tip_n - 1U];
          if (((ldx * ldx) + (ldy * ldy)) < 9) { rec = 0; }
        }
        if (rec != 0)
        {
          if (g_tip_n > 0U)
          {
            int ldx = tx - (int)g_tip_x[g_tip_n - 1U];
            int ldy = ty - (int)g_tip_y[g_tip_n - 1U];
            g_path_px += (int32_t)isqrt32((uint32_t)((ldx * ldx) + (ldy * ldy)));
          }
          g_tip_x[g_tip_n] = (int16_t)tx;
          g_tip_y[g_tip_n] = (int16_t)ty;
          g_tip_n++;
        }
      }
      g_span_mdeg += dang;
      st->dt_us = stamp_us - st->win_t0;
      if (g_tip_n >= 12U)
      {
        int fcx, fcy, fr;
        if (fit_circle(g_tip_x, g_tip_y, (int)g_tip_n, &fcx, &fcy, &fr) == 0)
        {
          int32_t a_prev;
          int32_t acc = 0;
          uint16_t i;
          a_prev = atan2_mdeg((int)g_tip_y[0] - fcy, (int)g_tip_x[0] - fcx);
          for (i = 1U; i < g_tip_n; i++)
          {
            int32_t ai = atan2_mdeg((int)g_tip_y[i] - fcy, (int)g_tip_x[i] - fcx);
            acc += wrap_diff_mdeg(ai, a_prev);
            a_prev = ai;
          }
          g_fit_span_mdeg = acc;
          /* 必须绕拟合圆心走出约 0.3 L，且半径、圆心要接近定位结果。 */
          if (iabs32(acc) >= HUB_LOCK_MDEG)
          {
            int loc_r = (int)st->needle.r;
            int ok = 1;
            int ring;
            int32_t a1;
            if ((loc_r >= 24) && (fr < ((loc_r * 7) / 10))) { ok = 0; }
            if (ok != 0)
            {
              ring = (fr * RING_SHRINK_N) / RING_SHRINK_D;
              if (ring < 20) { ring = 20; }
              st->needle.cx = (int16_t)fcx;
              st->needle.cy = (int16_t)fcy;
              st->needle.r = (int16_t)ring;
              st->needle.hub_r = (int16_t)(fr / 4);
              if (st->needle.hub_r < 10) { st->needle.hub_r = 10; }
              st->hub_locked = 1;
              a1 = a_prev;
              st->last_ang = a1;
              st->needle.ang_mdeg = a1;
              st->win_t0 = stamp_us;
              st->win_accum = 0;
              st->mid_t = 0U;
              st->mid_accum = 0;
              if (acc > 0) { st->dir_sign = 1; }
              else if (acc < 0) { st->dir_sign = -1; }
              /* 预估流速等走完一圈再报，避免短弧拟合给出离谱值。 */
              rev_start(stamp_us, a1);
              g_track_miss = 0U;
            }
          }
        }
      }
      return 0;
    }

    /* 圆心已锁定：以某一帧为起点，指针走约一圈（1 L），
     * 在最接近起点时用实际转角除以用时得到流量。 */
    {
      int32_t off;
      if ((st->dir_sign == 0) && (iabs32(dang) >= 2000))
      {
        st->dir_sign = (dang > 0) ? 1 : -1;
      }
      if (g_rev_t0 == 0U)
      {
        rev_start(stamp_us, st->needle.ang_mdeg);
      }
      g_rev_accum += dang;
      off = iabs32(wrap_diff_mdeg(st->needle.ang_mdeg, g_rev_ang0));
      if (iabs32(g_rev_accum) >= REV_HUNT_MDEG)
      {
        if (g_rev_hunt == 0U)
        {
          g_rev_hunt = 1U;
          g_rev_best_d = off;
          g_rev_best_accum = g_rev_accum;
          g_rev_best_t = stamp_us;
          g_rev_best_ang = st->needle.ang_mdeg;
        }
        else if (off <= g_rev_best_d)
        {
          g_rev_best_d = off;
          g_rev_best_accum = g_rev_accum;
          g_rev_best_t = stamp_us;
          g_rev_best_ang = st->needle.ang_mdeg;
        }
        else if (off > (g_rev_best_d + REV_PASS_MDEG))
        {
          uint32_t dt = g_rev_best_t - g_rev_t0;
          if ((dt >= 2000000U) && (iabs32(g_rev_best_accum) >= 250000))
          {
            int32_t inst = (int32_t)(((int64_t)iabs32(g_rev_best_accum) * 1000000LL) / (int64_t)dt);
            st->flow_lh_e2 = inst;
            st->flow_filt_e2 = inst;
            st->have_speed = 1U;
            st->dt_us = dt;
          }
          /* 最近点这一帧作为下一圈的起点。 */
          rev_start(g_rev_best_t, g_rev_best_ang);
          g_rev_accum = wrap_diff_mdeg(st->needle.ang_mdeg, g_rev_best_ang);
        }
      }
    }
  }
  (void)MIN_FPS_X10;
  return 0;
}

static void fill_rect(uint16_t *fb, int fb_w, int fb_h, int x0, int y0, int x1, int y1, uint16_t c)
{
  int x;
  int y;
  if (x0 < 0) { x0 = 0; }
  if (y0 < 0) { y0 = 0; }
  if (x1 >= fb_w) { x1 = fb_w - 1; }
  if (y1 >= fb_h) { y1 = fb_h - 1; }
  for (y = y0; y <= y1; y++)
  {
    uint16_t *row = fb + (y * fb_w);
    for (x = x0; x <= x1; x++) { row[x] = c; }
  }
}

static void plot(uint16_t *fb, int fb_w, int fb_h, int x, int y, uint16_t c, int t)
{
  int i;
  int j;
  int hw = (t > 1) ? (t / 2) : 0;
  for (j = -hw; j <= hw; j++)
  {
    for (i = -hw; i <= hw; i++)
    {
      int xx = x + i;
      int yy = y + j;
      if ((xx >= 0) && (yy >= 0) && (xx < fb_w) && (yy < fb_h))
      {
        fb[(yy * fb_w) + xx] = c;
      }
    }
  }
}

static void line_to(uint16_t *fb, int fb_w, int fb_h, int x0, int y0, int x1, int y1, uint16_t c, int t)
{
  int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
  int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  for (;;)
  {
    plot(fb, fb_w, fb_h, x0, y0, c, t);
    if ((x0 == x1) && (y0 == y1)) { break; }
    {
      int e2 = err * 2;
      if (e2 > -dy) { err -= dy; x0 += sx; }
      if (e2 < dx) { err += dx; y0 += sy; }
    }
  }
}

static void draw_circle(uint16_t *fb, int fb_w, int fb_h, int cx, int cy, int R, uint16_t c, int t)
{
  int a;
  for (a = 0; a < 360; a += 2)
  {
    int x = cx + (int)(((int32_t)icos(a) * R) / 256);
    int y = cy + (int)(((int32_t)isin(a) * R) / 256);
    plot(fb, fb_w, fb_h, x, y, c, t);
  }
}

static const uint8_t speed_extra_bits[860] = {
  0xA4,0x1A,0x00,0x00,0x00,0x00,0x60,0xFF,0x02,0x00,0x00,0x00,0x00,0xF6,0x2F,0x00,
  0x00,0x00,0x00,0x60,0xFF,0x02,0x00,0x00,0x00,0x00,0xF6,0x2F,0x00,0x00,0x00,0x00,
  0x60,0xFF,0x02,0x00,0x00,0x00,0x00,0xF6,0x2F,0x00,0x00,0x00,0x00,0x60,0xFF,0x01,
  0xB5,0xEE,0x5B,0x00,0xF6,0x1F,0xF9,0xFF,0xFF,0x7F,0x60,0xFF,0xFD,0x5A,0x84,0xFF,
  0x1F,0xF6,0xFF,0x04,0x00,0x60,0xFF,0x66,0xFF,0x04,0x00,0x00,0xF0,0x9F,0xF6,0x2F,
  0x00,0x00,0x00,0xFE,0x6A,0xFF,0x02,0x00,0x00,0xD0,0xBF,0xF6,0x2F,0x00,0x00,0x00,
  0xFD,0x6B,0xFF,0x02,0x00,0x00,0xD0,0xBF,0xF6,0x2F,0x00,0x00,0x00,0xFD,0x6B,0xFF,
  0x02,0x00,0x00,0xD0,0xBF,0xF6,0x2F,0x00,0x00,0x00,0xFD,0x6B,0xFF,0x02,0x00,0x00,
  0xD0,0xBF,0xF6,0x2F,0x00,0x00,0x00,0xFD,0x6B,0xFF,0x02,0x00,0x00,0xD0,0xBF,0xA4,
  0x1A,0x60,0xFF,0x02,0xF6,0x2F,0x60,0xFF,0x02,0xF6,0x2F,0x60,0xFF,0x02,0xF6,0x2F,
  0x60,0xFF,0x02,0xF6,0x2F,0x60,0xFF,0x02,0xF6,0x2F,0x60,0xFF,0x02,0xF6,0x2F,0x60,
  0xFF,0x02,0xF6,0x2F,0x60,0xFF,0x02,0xF6,0x2F,0x60,0xFF,0x02,0xF6,0x2F,0x60,0xFF,
  0x02,0xF5,0x7F,0x10,0xFF,0x7F,0x50,0xFD,0x09,0xF6,0x0B,0x60,0xFC,0xCE,0x05,0x00,
  0xF6,0x3D,0xFD,0xFF,0xFF,0x9F,0x00,0xF6,0xFF,0x9F,0x35,0xD6,0xFF,0x08,0xF6,0xCF,
  0x02,0x00,0x10,0xFC,0x1F,0xF6,0x2F,0x00,0x00,0x00,0xF5,0x7F,0xF6,0x2F,0x00,0x00,
  0x00,0xF0,0xAF,0xF6,0x2F,0x00,0x00,0x00,0xE0,0xCF,0xF6,0x2F,0x00,0x00,0x00,0xD0,
  0xDF,0xF6,0x2F,0x00,0x00,0x00,0xF0,0xCF,0xF6,0x2F,0x00,0x00,0x00,0xF2,0x9F,0xF6,
  0x2F,0x00,0x00,0x00,0xF9,0x4F,0xF6,0x7F,0x00,0x00,0x40,0xFF,0x0B,0xF6,0xFF,0x6C,
  0x43,0xF8,0xFF,0x02,0xF6,0x9F,0xFF,0xFF,0xFF,0x3D,0x00,0xF6,0x1F,0x92,0xFD,0x8D,
  0x00,0x00,0xF6,0x1F,0x00,0x00,0x00,0x00,0x00,0xF6,0x2F,0x00,0x00,0x00,0x00,0x00,
  0xF6,0x2F,0x00,0x00,0x00,0x00,0x00,0xF6,0x2F,0x00,0x00,0x00,0x00,0x00,0xF6,0x2F,
  0x00,0x00,0x00,0x00,0x00,0xF6,0x2F,0x00,0x00,0x00,0x00,0x00,0xF6,0x2F,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x31,0x01,0x00,0x00,0x00,0x80,0x5F,0x00,0x00,
  0x00,0x00,0xFD,0x00,0x00,0x00,0x00,0xF2,0x0B,0x00,0x00,0x00,0x70,0x6F,0x00,0x00,
  0x00,0x00,0xFC,0x01,0x00,0x00,0x00,0xF1,0x0C,0x00,0x00,0x00,0x60,0x7F,0x00,0x00,
  0x00,0x00,0xFA,0x02,0x00,0x00,0x00,0xF0,0x0D,0x00,0x00,0x00,0x40,0x8F,0x00,0x00,
  0x00,0x00,0xF9,0x03,0x00,0x00,0x00,0xE0,0x0E,0x00,0x00,0x00,0x30,0xAF,0x00,0x00,
  0x00,0x00,0xF8,0x05,0x00,0x00,0x00,0xD0,0x0F,0x00,0x00,0x00,0x20,0xBF,0x00,0x00,
  0x00,0x00,0xF7,0x06,0x00,0x00,0x00,0xC0,0x1F,0x00,0x00,0x00,0x10,0xCF,0x00,0x00,
  0x00,0x00,0xF6,0x07,0x00,0x00,0x00,0xB0,0x2F,0x00,0x00,0x00,0x00,0xDF,0x00,0x00,
  0x00,0x00,0xF5,0x08,0x00,0x00,0x00,0x90,0x3F,0x00,0x00,0x00,0x00,0xEE,0x00,0x00,
  0x00,0x00,0xF3,0x0A,0x00,0x00,0x00,0x80,0x5F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x08,0x00,0x00,
  0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF1,0x0F,0x00,0x00,0xFD,0x04,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xF1,0x0F,0x00,0x00,0xFD,0xFF,0xFF,0xFF,0xFF,0x2F,
  0x00,0x00,0x00,0xF1,0x0F,0x00,0x00,0xFD,0xCD,0xCC,0xCC,0xCC,0x2C,0x00,0x00,0x00,
  0xF1,0x0F,0x00,0x00,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF1,0x0F,0x00,
  0x00,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF1,0x0F,0x00,0x00,0xFD,0x04,
  0x00,0x00,0x00,0x00,0x00,0xE5,0xEE,0xFE,0xEF,0xEE,0xEE,0xFF,0xEF,0xEE,0xEE,0xEE,
  0xEE,0x9E,0xF6,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x9F,0x00,
  0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD8,0x07,0x00,
  0x00,0x00,0xFB,0x05,0x00,0x00,0x40,0xCD,0x00,0x00,0xF9,0x08,0x00,0x00,0x00,0xFF,
  0x01,0x00,0x00,0x40,0xDF,0x00,0x00,0xF9,0x08,0x00,0x00,0x50,0xCF,0x00,0x00,0x00,
  0x40,0xDF,0x00,0x00,0xF9,0x08,0x00,0x00,0xB0,0xAF,0x00,0x00,0x00,0x40,0xDF,0x00,
  0x00,0xF9,0x08,0x00,0x00,0xF4,0xFF,0x1B,0x00,0x00,0x40,0xDF,0x00,0x00,0xF9,0x08,
  0x00,0x00,0xFD,0xD8,0xDF,0x02,0x00,0x40,0xDF,0x00,0x00,0xF9,0x08,0x00,0xA0,0xAF,
  0x00,0xFA,0x5F,0x00,0x40,0xDF,0x00,0x00,0xF9,0x08,0x00,0xF9,0x1E,0x00,0x80,0xFF,
  0x08,0x40,0xDF,0x00,0x00,0xF9,0x08,0xA1,0xFF,0x03,0x00,0x00,0xF5,0xAF,0x40,0xDF,
  0x00,0x00,0xF9,0x48,0xFE,0x2D,0x00,0x00,0x00,0x40,0xFF,0x48,0xDF,0x00,0x00,0xF9,
  0x18,0xBC,0x01,0x00,0x00,0x00,0x00,0xC3,0x41,0xDF,0x00,0x00,0xF9,0x08,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x40,0xDF,0x00,0x00,0xF9,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xDF,0x00,0x00,0xF9,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
  0xDF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0xDF,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x22,0x00,
};

static const dial_glyph_t speed_extra_glyphs[5] = {
  {0x68, 2, 11, 13, 22, 1088, &speed_extra_bits[0]},    /* h */
  {0x6C, 2, 10, 5, 23, 509, &speed_extra_bits[143]},    /* l */
  {0x70, 2, 18, 14, 22, 1111, &speed_extra_bits[201]},  /* p */
  {0x2F, 0, 10, 11, 28, 702, &speed_extra_bits[355]},   /* / */
  {0xD7, 1, 9, 26, 27, 1792, &speed_extra_bits[509]},   /* 齿 */
};

static const dial_glyph_t *glyph(uint8_t code)
{
  uint32_t i;
  for (i = 0; i < DIAL_FONT_NGLYPH; i++)
  {
    if (dial_font_glyphs[i].code == code) { return &dial_font_glyphs[i]; }
  }
  for (i = 0; i < 5U; i++)
  {
    if (speed_extra_glyphs[i].code == code) { return &speed_extra_glyphs[i]; }
  }
  return 0;
}

static void blend_px(uint16_t *fb, int fb_w, int fb_h, int x, int y, int a)
{
  uint16_t p;
  uint16_t out;
  int pr, pg, pb;
  if ((a <= 0) || (x < 0) || (y < 0) || (x >= fb_w) || (y >= fb_h)) { return; }
  if (a > 15) { a = 15; }
  p = fb[(y * fb_w) + x];
  pr = (((p >> 11) & 31) * 255) / 31;
  pg = (((p >> 5) & 63) * 255) / 63;
  pb = ((p & 31) * 255) / 31;
  pr = ((pr * (15 - a)) + (TXT_R * a)) / 15;
  pg = ((pg * (15 - a)) + (TXT_G * a)) / 15;
  pb = ((pb * (15 - a)) + (TXT_B * a)) / 15;
  out = (uint16_t)(((pr * 31 / 255) << 11) | ((pg * 63 / 255) << 5) | (pb * 31 / 255));
  fb[(y * fb_w) + x] = out;
}

static int draw_code(uint16_t *fb, int fb_w, int fb_h, int x, int y, uint8_t code)
{
  const dial_glyph_t *g = glyph(code);
  int ix;
  int iy;
  if (g == 0)
  {
    return 8;
  }
  for (iy = 0; iy < (int)g->h; iy++)
  {
    for (ix = 0; ix < (int)g->w; ix++)
    {
      uint32_t bit_i = (((uint32_t)iy * (uint32_t)g->w) + (uint32_t)ix);
      uint8_t by = g->bits[bit_i / 2U];
      int a = ((bit_i & 1U) == 0U) ? (int)(by & 0x0FU) : (int)(by >> 4);
      if (a != 0)
      {
        blend_px(fb, fb_w, fb_h, x + (int)g->xoff + ix, y + (int)g->yoff + iy, a);
      }
    }
  }
  return (int)((g->adv64 + 32) / 64);
}

static int draw_str(uint16_t *fb, int fb_w, int fb_h, int x, int y, const char *s)
{
  int x0 = x;
  if (s == 0) { return 0; }
  while (*s)
  {
    x += draw_code(fb, fb_w, fb_h, x, y, (uint8_t)*s);
    s++;
  }
  return x - x0;
}

static int measure_str(const char *s)
{
  int x = 0;
  if (s == 0) { return 0; }
  while (*s)
  {
    uint8_t code = (uint8_t)*s++;
    const dial_glyph_t *g = glyph(code);
    if (g != 0) { x += (int)((g->adv64 + 32) / 64); }
    else if (code == 0xD7U) { x += 18; }
    else { x += 12; }
  }
  return x;
}

static void put_u(char *p, uint32_t v, int digits)
{
  int i;
  for (i = digits - 1; i >= 0; i--)
  {
    p[i] = (char)('0' + (char)(v % 10U));
    v /= 10U;
  }
}

static void fmt_ip_frac(char *dst, uint32_t av, uint32_t scale, int frac)
{
  uint32_t ip = av / scale;
  uint32_t fp = av % scale;
  char tmp[12];
  int n = 0;
  int i;
  char *p = dst;
  if (ip == 0U)
  {
    tmp[n++] = '0';
  }
  else
  {
    char rev[12];
    int k = 0;
    while (ip > 0U)
    {
      rev[k++] = (char)('0' + (char)(ip % 10U));
      ip /= 10U;
    }
    while (k > 0) { tmp[n++] = rev[--k]; }
  }
  for (i = 0; i < n; i++) { *p++ = tmp[i]; }
  *p++ = '.';
  put_u(p, fp, frac);
  p += frac;
  *p = 0;
}

static void fmt_signed_frac(char *dst, int32_t e, int scale, int frac)
{
  char *p = dst;
  uint32_t av;
  if (e < 0) { *p++ = '-'; av = (uint32_t)(-e); }
  else { av = (uint32_t)e; }
  fmt_ip_frac(p, av, (uint32_t)scale, frac);
}

void SpeedTrack_Paint(uint16_t *fb, uint16_t fb_w, uint16_t fb_h,
                      int16_t ox, int16_t oy, const speed_state_t *st,
                      const dial_time_t *t)
{
  char l1[48];
  char l2[32];
  char l3[32];
  char l4[32];
  int ncx, ncy, x0, y0, x1, y1, tw, lines;
  const int hx = 12;
  const int hy = 8;
  const int gap = 28;
  const int pad = 8;

  if ((fb == 0) || (st == 0) || (fb_w < 16U) || (fb_h < 16U)) { return; }

  ncx = (int)ox + (int)st->needle.cx;
  ncy = (int)oy + (int)st->needle.cy;
  if ((st->needle.found != 0U) || (st->hub_locked != 0))
  {
    if (st->hub_locked != 0)
    {
      int hr = (int)st->needle.hub_r;
      if (hr < 8) { hr = 8; }
      draw_circle(fb, (int)fb_w, (int)fb_h, ncx, ncy, st->needle.r, LIVE_PATH, 2);
      draw_circle(fb, (int)fb_w, (int)fb_h, ncx, ncy, hr, LIVE_HUB, 3);
      plot(fb, (int)fb_w, (int)fb_h, ncx, ncy, LIVE_HUB, 4);
    }
    else
    {
      uint16_t i;
      for (i = 0U; i < g_tip_n; i++)
      {
        plot(fb, (int)fb_w, (int)fb_h, (int)ox + (int)g_tip_x[i],
             (int)oy + (int)g_tip_y[i], LIVE_LEAD, 2);
      }
    }
    plot(fb, (int)fb_w, (int)fb_h, (int)ox + (int)st->needle.tip_x,
         (int)oy + (int)st->needle.tip_y, LIVE_LEAD, 5);
  }

  l1[0] = 0;
  if (t != 0)
  {
    char *p = l1;
    put_u(p, (uint32_t)t->year, 4); p += 4; *p++ = '-';
    put_u(p, (uint32_t)t->mon, 2); p += 2; *p++ = '-';
    put_u(p, (uint32_t)t->day, 2); p += 2; *p++ = ' ';
    put_u(p, (uint32_t)t->hour, 2); p += 2; *p++ = ':';
    put_u(p, (uint32_t)t->min, 2); p += 2; *p++ = ':';
    put_u(p, (uint32_t)t->sec, 2); p += 2;
    if (st->have_speed != 0U)
    {
      uint32_t ms = st->dt_us / 1000U;
      int nd = (ms >= 1000U) ? 4 : 3;
      *p++ = ' ';
      put_u(p, ms, nd); p += nd;
      *p++ = ' ';
      *p++ = 'm';
      *p++ = 's';
    }
    *p = 0;
  }
  else
  {
    l1[0] = '?';
    l1[1] = 0;
  }

  if (st->have_speed != 0U)
  {
    char num[16];
    char *p;
    const char *s;
    fmt_signed_frac(num, st->flow_filt_e2, 100, 2);
    p = l2; s = num; while (*s) { *p++ = *s++; }
    *p++ = ' '; *p++ = 'L'; *p++ = '/'; *p++ = 'h'; *p = 0;
    /* 秒/圈 = 3600 / (L/h)，乘 100 以便两位小数。 */
    if (st->flow_filt_e2 > 0)
    {
      int32_t per_e2 = (int32_t)(36000000LL / (int64_t)st->flow_filt_e2);
      fmt_signed_frac(num, per_e2, 100, 2);
      p = l3; s = num; while (*s) { *p++ = *s++; }
      *p++ = ' '; *p++ = 's'; *p = 0;
    }
    else
    {
      l3[0] = '-'; l3[1] = '-'; l3[2] = '-'; l3[3] = 0;
    }
    l4[0] = 0;
    lines = 3;
  }
  else
  {
    /* 定位中或已裁窗但还没有速度：字库没有 o/k，用 --- 避免第二行空白。 */
    l2[0] = '-'; l2[1] = '-'; l2[2] = '-'; l2[3] = 0;
    l3[0] = 0; l4[0] = 0;
    lines = 2;
  }

  tw = measure_str(l1);
  {
    int w2 = measure_str(l2);
    int w3 = measure_str(l3);
    int w4 = measure_str(l4);
    if (w2 > tw) { tw = w2; }
    if (w3 > tw) { tw = w3; }
    if (w4 > tw) { tw = w4; }
  }
  x0 = hx - pad;
  y0 = hy - pad;
  x1 = hx + tw + pad;
  y1 = hy + (lines * gap) + pad + 12;  /* p 和 齿 会落到基线以下 */
  fill_rect(fb, (int)fb_w, (int)fb_h, x0, y0, x1, y1, 0xFFFF);
  draw_str(fb, (int)fb_w, (int)fb_h, hx, hy, l1);
  draw_str(fb, (int)fb_w, (int)fb_h, hx, hy + gap, l2);
  if (lines >= 3) { draw_str(fb, (int)fb_w, (int)fb_h, hx, hy + (gap * 2), l3); }
  if (lines >= 4) { draw_str(fb, (int)fb_w, (int)fb_h, hx, hy + (gap * 3), l4); }
}
