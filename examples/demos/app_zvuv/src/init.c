/*
 * File: init.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 08-Nov-2021 11:14:21
 */

/* Include Files */
//#include "rt_nonfinite.h"  // removed by ROEE
#include "cf_feat_ext.h"
#include "feat2cmd.h"
#include "init.h"

/* Function Definitions */

/*
 * Arguments    : unsigned short xyPosLen
 *                unsigned short xyMeasLen
 *                Opt_type *Opt
 *                Cntrl_type *Cntrl
 *                Opt_LLAvd_type *Opt_LLAvd
 * Return Type  : void
 */
void init(unsigned short xyPosLen, unsigned short xyMeasLen, Opt_type *Opt,
          Cntrl_type *Cntrl, Opt_LLAvd_type *Opt_LLAvd)
{
  Opt->measHistMinDist_p2 = 9.0F;
  Opt->posHistMinStep_p2 = 1600.0F;
  Opt->xyPosLen = xyPosLen;
  Opt->xyMeasLen = xyMeasLen;
  Opt->nCyc_lowFreq = 2;
  Opt->nCyc_lowFreq_bp = 2;
  Opt->SF_frwrd = 60.0F;
  Opt->exp_LP = 0.96F;
  Opt->PF_radius_max = 100;
  Opt->PF_radius_min = 8;
  Opt->LF_dist_max = 200;
  Opt->stopped_dist = 60;
  Cntrl->Kpsi = 1.0F;
  Cntrl->Kdy = 1.0F;
  Cntrl->Kdx = 1.0F;
  Cntrl->Vnom = 1.0F;
  Cntrl->Vnom_min_rtio = 0.1F;
  Cntrl->Vnom_slw_filt = 0.12F;
  Cntrl->Vnom_acc_filt = 0.05F;
  Cntrl->sf_deaccel_cnt = 10;
  Cntrl->stopped_consec_cnt = 30;
  Opt_LLAvd->vel_max_cmps = 200.0F;
  Opt_LLAvd->mr18_dt_msec = 33;
  Opt_LLAvd->LLAV_radius_min_cm = 8;
  Opt_LLAvd->max_acc_mps2_inv = 0.095785439F;
  Opt_LLAvd->tau_theta_msec = 150;
  Opt_LLAvd->TOF_radius_max_cm = 80;
  Opt_LLAvd->TOF_radius_min_cm = 3;
  Opt_LLAvd->vel_noise_th_cmps = 30.0F;
  Opt_LLAvd->llav_deadB = 7.5F;
  Opt_LLAvd->kFF_ang = 1.5F;
  Opt_LLAvd->mps2Deg = 5.0F;
  Opt_LLAvd->rej_radius_mm = 200;
  Opt_LLAvd->rej_k_avoid = 0.02F;
  Opt_LLAvd->rej_avoidance_clamp = 5.0F;
}

/*
 * File trailer for init.c
 *
 * [EOF]
 */
