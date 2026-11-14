#ifndef DIAL_PATH_DEBUG_H
#define DIAL_PATH_DEBUG_H
#include <stdint.h>
#define DIAL_PATH_PK 64U
/* Diagnostics only. Coordinates/radii Q8, angles milli-degree, SSE Q16.
 * why: 0 kept,1 merged,2 contrast/slot,3 red shadow,4 pointer misaligned,
 * 5 pointer weak,6 no pair,8 feature unavailable.
 * gate veto: bit0 radius,1 points,2 bins,3 inlier ratio,4 RMS,5 improvement. */
typedef struct {
 uint32_t run_seq,candidate,reason,n_vis,tip_sub;
 int32_t place,blob_x,blob_y,hub_x,hub_y;
 uint32_t area,dtm;
 int32_t longest_cx,longest_cy,longest_R;
 uint32_t longest_n,longest_start,longest_veto;
 uint64_t longest_ss;
 int32_t seed_cx,seed_cy,seed_R;
 uint32_t partial_used,seed_n;
 int32_t trace_cx,trace_cy,trace_R;
 uint32_t trace_n,inliers,bins;
 uint64_t trace_ss,seed_ss;
 uint32_t shrink,gate_keep,gate_veto;
 int32_t final_cx,final_cy,final_R;
 uint32_t final_cov,raw_n,feature_n,merged_n,contrast_n;
 uint32_t pair_edges,pair_ang_reject,pair_xy_reject,pair_center_reject,pairs_used,accepted_n;
 uint32_t drop_counts[9];
 int32_t raw_a[64],raw_h[64],merged_a[64],merged_h[64],accepted_a[64],accepted_h[64];
 uint32_t why[64];
} dial_path_record_t;
typedef struct { uint32_t version,count; dial_path_record_t record; } dial_path_latch_t;
extern volatile dial_path_latch_t g_dial_path_success,g_dial_path_failure,g_dial_path_held;
#endif
