/*
 * File: cf_feat_ext_types.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 08-Nov-2021 11:14:21
 */

#ifndef CF_FEAT_EXT_TYPES_H
#define CF_FEAT_EXT_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_Cntrl_type
#define typedef_Cntrl_type

typedef struct {
  float Kpsi;
  float Kdy;
  float Kdx;
  float Vnom;
  float Vnom_min_rtio;
  float Vnom_slw_filt;
  float Vnom_acc_filt;
  signed char sf_deaccel_cnt;
  signed char stopped_consec_cnt;
} Cntrl_type;

#endif                                 /*typedef_Cntrl_type*/

#ifndef typedef_Opt_LLAvd_type
#define typedef_Opt_LLAvd_type

typedef struct {
  float vel_max_cmps;
  unsigned short mr18_dt_msec;
  unsigned short LLAV_radius_min_cm;
  float max_acc_mps2_inv;
  unsigned short tau_theta_msec;
  unsigned short TOF_radius_max_cm;
  unsigned short TOF_radius_min_cm;
  float vel_noise_th_cmps;
  float llav_deadB;
  float kFF_ang;
  float mps2Deg;
  unsigned short rej_radius_mm;
  float rej_k_avoid;
  float rej_avoidance_clamp;
} Opt_LLAvd_type;

#endif                                 /*typedef_Opt_LLAvd_type*/

#ifndef typedef_Opt_type
#define typedef_Opt_type

typedef struct {
  float measHistMinDist_p2;
  float posHistMinStep_p2;
  unsigned short xyPosLen;
  unsigned short xyMeasLen;
  unsigned char nCyc_lowFreq;
  unsigned char nCyc_lowFreq_bp;
  float SF_frwrd;
  float exp_LP;
  unsigned short PF_radius_max;
  unsigned short PF_radius_min;
  unsigned short LF_dist_max;
  unsigned short stopped_dist;
} Opt_type;

#endif                                 /*typedef_Opt_type*/
#endif

/*
 * File trailer for cf_feat_ext_types.h
 *
 * [EOF]
 */
