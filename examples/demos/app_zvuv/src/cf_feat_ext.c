/*
 * File: cf_feat_ext.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 08-Nov-2021 11:14:21
 */

/* Include Files */
//#include "rt_nonfinite.h"  // removed by ROEE
#include "cf_feat_ext.h"
#include "feat2cmd.h"
#include "init.h"
#include "mtlb_sum.h"
#include "mtlb_power.h"
#include "mtlb_sqrt.h"
#include "mtlb_abs.h"
#include "mtlb_median.h"
#include "repmat.h"
#include "cf_feat_ext_data.h"

/* Variable Definitions */
static boolean_T firstRun_not_empty;
static short xyMeasWorld[200];
static unsigned short xyMeasLastInd;
static short xyPosWorld[30];
static unsigned short xyPosLastInd;
static short xyMeasWorldLastInputs[32];
static unsigned char s2c_i[16];
static float sin_sen[16];
static unsigned char r_180[16];
static unsigned char freq_reduce;
static unsigned char freq_reduce_bp;
static unsigned short numSamp;
static float xyHeadingAvg[2];
static short prevNav[2];
static float rdot_prev[16];
static float rdot_prev_prev[16];
static unsigned int TT_prev;
static unsigned short ranges_prev[16];
static float sin_sen_rev[16];
static float cos_sen_rev[16];
static unsigned short ranges_prev_prev[16];

/* Function Declarations */
static float mtlb_rt_roundf_snf(float u);

/* Function Definitions */

/*
 * Arguments    : float u
 * Return Type  : float
 */
static float mtlb_rt_roundf_snf(float u)
{
  float y;
  if ((float)fabs(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (float)floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = (float)ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * navXY is in cm
 * Arguments    : const short navXY[2]
 *                float Psi
 *                const unsigned short ranges[16]
 *                const Opt_type *Opt
 *                const Opt_LLAvd_type *Opt_LLAvd
 *                unsigned int TT
 *                const float Vin[2]
 *                boolean_T byPass
 *                float modelRANSAC[3]
 *                boolean_T *validData
 *                unsigned char *slowDwn
 *                float LLAvdCmd[2]
 *                float rej_Cmd[2]
 * Return Type  : void
 */
void cf_feat_ext(const short navXY[2], float Psi, const unsigned short ranges[16],
                 const Opt_type *Opt, const Opt_LLAvd_type *Opt_LLAvd, unsigned
                 int TT, const float Vin[2], boolean_T byPass, float
                 modelRANSAC[3], boolean_T *validData, unsigned char *slowDwn,
                 float LLAvdCmd[2], float rej_Cmd[2])
{
  int i;
  unsigned int qY;
  float dt;
  static const float fv0[16] = { 0.0F, 0.382683426F, 0.707106769F, 0.923879504F,
    1.0F, 0.923879504F, 0.707106769F, 0.382683426F, -0.0F, -0.382683426F,
    -0.707106769F, -0.923879504F, -1.0F, -0.923879504F, -0.707106769F,
    -0.382683426F };

  float rdot[16];
  static const unsigned char uv0[16] = { 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U,
    14U, 15U, 16U, 1U, 2U, 3U, 4U };

  signed char UseVel[16];
  float dt_inv;
  float xyHeadingAvg_nrm;
  static const unsigned char uv1[16] = { 9U, 10U, 11U, 12U, 13U, 14U, 15U, 16U,
    1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U };

  int i0;
  float sin_rob;
  float cos_rob;
  static const float fv1[16] = { -0.0F, -0.382683426F, -0.707106769F,
    -0.923879504F, -1.0F, -0.923879504F, -0.707106769F, -0.382683426F, 0.0F,
    0.382683426F, 0.707106769F, 0.923879504F, 1.0F, 0.923879504F, 0.707106769F,
    0.382683426F };

  static const float fv2[16] = { -1.0F, -0.923879504F, -0.707106769F,
    -0.382683426F, 0.0F, 0.382683426F, 0.707106769F, 0.923879504F, 1.0F,
    0.923879504F, 0.707106769F, 0.382683426F, -0.0F, -0.382683426F,
    -0.707106769F, -0.923879504F };

  unsigned char b_i;
  float rej_nrm;
  short i1;
  float xyHeadingAvg_nrm_vec[2];
  short xyMeasWorld_[2];
  unsigned short uv2[3];
  float fv3[16];
  unsigned short r_tmp;
  float b_UseVel[16];
  float sumy2[2];
  short points[200];
  boolean_T cond;
  short tmp;
  boolean_T cond_bp;
  boolean_T isInliers[100];
  float b_sumy2[4];
  short sample[4];
  float c_sumy2;
  float sumxy;
  unsigned char kk;
  int i2;
  float theta0[3];
  unsigned short dist[100];
  boolean_T in_vec[100];
  unsigned char ii;

  /* t */
  if (!firstRun_not_empty) {
    firstRun_not_empty = true;
    for (i = 0; i < 32; i++) {
      xyMeasWorldLastInputs[i] = 300;
    }

    memset(&xyMeasWorld[0], 0, 200U * sizeof(short));
    mtlb_repmat(navXY, xyPosWorld);
    xyMeasLastInd = Opt->xyMeasLen;
    xyPosLastInd = Opt->xyPosLen;

    /*  cos_sen-sin_sen([5:16 1:4]') == 0 */
    memcpy(&sin_sen[0], &fv0[0], sizeof(float) << 4);
    for (i = 0; i < 16; i++) {
      s2c_i[i] = uv0[i];
    }

    freq_reduce = 1;
    freq_reduce_bp = 1;
    numSamp = 0;
    for (i = 0; i < 2; i++) {
      xyHeadingAvg[i] = 0.0F;
      prevNav[i] = navXY[i];
    }

    /* t = int32(zeros(coder.Constant(Opt.xyMeasLen),1)); */
    TT_prev = 0U;
    for (i = 0; i < 16; i++) {
      r_180[i] = uv1[i];
      ranges_prev_prev[i] = 0;
      ranges_prev[i] = 0;
      rdot_prev[i] = 0.0F;
      rdot_prev_prev[i] = 0.0F;
      sin_sen_rev[i] = fv1[i];
      cos_sen_rev[i] = fv2[i];
    }
  }

  for (i = 0; i < 3; i++) {
    modelRANSAC[i] = 0.0F;
  }

  *slowDwn = 0;

  /*  Avoidance */
  qY = TT - TT_prev;
  if (qY > TT) {
    qY = 0U;
  }

  dt = (float)qY;

  /* msec */
  memset(&rdot[0], 0, sizeof(float) << 4);
  for (i = 0; i < 16; i++) {
    UseVel[i] = 0;
  }

  /*  UseRej=false(1,16); */
  /*  rejAmp=single(zeros(16,1)); */
  for (i = 0; i < 2; i++) {
    rej_Cmd[i] = 0.0F;
  }

  if (dt > 0.0001F) {
    dt_inv = 1000.0F / dt;

    /* 1/sec */
  } else {
    dt_inv = 1.0E+6F;
  }

  xyHeadingAvg_nrm = 0.0F;
  for (i = 0; i < 2; i++) {
    i0 = navXY[i] - prevNav[i];
    if (i0 > 32767) {
      i0 = 32767;
    } else {
      if (i0 < -32768) {
        i0 = -32768;
      }
    }

    xyHeadingAvg[i] = xyHeadingAvg[i] * Opt->exp_LP + (float)i0 * (1.0F -
      Opt->exp_LP);
    prevNav[i] = navXY[i];
    xyHeadingAvg_nrm += xyHeadingAvg[i] * xyHeadingAvg[i];
  }

  mtlb_sqrt(&xyHeadingAvg_nrm);
  sin_rob = (float)sin(Psi);
  cos_rob = (float)cos(Psi);

  /*  project range measurements to world: */
  /* %% Avoidance %%% */
  for (b_i = 1; b_i < 17; b_i++) {
    if ((ranges[b_i - 1] > Opt->PF_radius_min) && (ranges[b_i - 1] <
         Opt->PF_radius_max)) {
      rej_nrm = mtlb_rt_roundf_snf((float)ranges[b_i - 1] * (cos_rob *
        sin_sen[s2c_i[b_i - 1] - 1] - sin_rob * sin_sen[b_i - 1]));
      if (rej_nrm < 32768.0F) {
        if (rej_nrm >= -32768.0F) {
          i1 = (short)rej_nrm;
        } else {
          i1 = MIN_int16_T;
        }
      } else if (rej_nrm >= 32768.0F) {
        i1 = MAX_int16_T;
      } else {
        i1 = 0;
      }

      i = i1 + navXY[0];
      if (i > 32767) {
        i = 32767;
      } else {
        if (i < -32768) {
          i = -32768;
        }
      }

      xyMeasWorld_[0] = (short)i;
      rej_nrm = mtlb_rt_roundf_snf((float)ranges[b_i - 1] * (cos_rob *
        sin_sen[b_i - 1] + sin_sen[s2c_i[b_i - 1] - 1] * sin_rob));
      if (rej_nrm < 32768.0F) {
        if (rej_nrm >= -32768.0F) {
          i1 = (short)rej_nrm;
        } else {
          i1 = MIN_int16_T;
        }
      } else if (rej_nrm >= 32768.0F) {
        i1 = MAX_int16_T;
      } else {
        i1 = 0;
      }

      i = i1 + navXY[1];
      if (i > 32767) {
        i = 32767;
      } else {
        if (i < -32768) {
          i = -32768;
        }
      }

      xyMeasWorld_[1] = (short)i;
      for (i = 0; i < 2; i++) {
        i0 = xyMeasWorldLastInputs[(b_i + (i << 4)) - 1] - xyMeasWorld_[i];
        if (i0 > 32767) {
          i0 = 32767;
        } else {
          if (i0 < -32768) {
            i0 = -32768;
          }
        }

        xyHeadingAvg_nrm_vec[i] = (float)i0;
      }

      mtlb_power(xyHeadingAvg_nrm_vec, sumy2);
      for (i = 0; i < 2; i++) {
        xyHeadingAvg_nrm_vec[i] = sumy2[i];
      }

      if (mtlb_sum(xyHeadingAvg_nrm_vec) >= Opt->measHistMinDist_p2) {
        qY = xyMeasLastInd + 1U;
        if (qY > 65535U) {
          qY = 65535U;
        }

        if (xyMeasLastInd == Opt->xyMeasLen) {
          xyMeasLastInd = 1;
        } else {
          xyMeasLastInd = (unsigned short)qY;
        }

        for (i = 0; i < 2; i++) {
          xyMeasWorldLastInputs[(b_i + (i << 4)) - 1] = xyMeasWorld_[i];
          xyMeasWorld[(xyMeasLastInd + 100 * i) - 1] = xyMeasWorld_[i];
        }

        qY = numSamp + 1U;
        if (qY > 65535U) {
          qY = 65535U;
        }

        r_tmp = (unsigned short)qY;
        if (r_tmp < Opt->xyMeasLen) {
          numSamp = r_tmp;
        } else {
          numSamp = Opt->xyMeasLen;
        }
      }
    }

    /*  if (xyHeadingAvg(1)*sin_sen(s2c_i(i))+xyHeadingAvg(2)*sin_sen(i))*Opt.SF_frwrd>ranges(i) */
    /*      disp(num2str(i)) */
    /*  end */
    if ((ranges[b_i - 1] > Opt->PF_radius_min) && (ranges[b_i - 1] <
         Opt->LF_dist_max) && (xyHeadingAvg_nrm * Opt->SF_frwrd > 50.0F)) {
      /* coder.const(50*50) */
      /* xyHeadingAvg_nrm = sqrt(xyHeadingAvg_nrm); */
      for (i = 0; i < 2; i++) {
        xyHeadingAvg_nrm_vec[i] = xyHeadingAvg[i] / xyHeadingAvg_nrm;
      }

      dt = xyHeadingAvg_nrm_vec[0] * (cos_rob * sin_sen[s2c_i[b_i - 1] - 1] -
        sin_rob * sin_sen[b_i - 1]) + xyHeadingAvg_nrm_vec[1] * (cos_rob *
        sin_sen[b_i - 1] + sin_sen[s2c_i[b_i - 1] - 1] * sin_rob);
      if ((dt > 0.97F) && (xyHeadingAvg_nrm * Opt->SF_frwrd * dt > ranges[b_i -
                           1])) {
        *slowDwn = 1;
      }
    }

    /*   check input   */
    if ((ranges[b_i - 1] > Opt_LLAvd->TOF_radius_min_cm) && (ranges[b_i - 1] <
         Opt_LLAvd->TOF_radius_max_cm) && (TT_prev > 0U)) {
      dt = (float)(ranges[b_i - 1] - ranges_prev[b_i - 1]) * dt_inv;

      /*  filter out large velocities */
      if ((float)fabs(dt) > Opt_LLAvd->vel_max_cmps) {
        dt = 0.0F;
      }

      /*  calc. mean rdot */
      rdot[b_i - 1] = ((dt + rdot_prev[b_i - 1]) + rdot_prev_prev[b_i - 1]) *
        0.33333F;
    }

    /*  test if rdot value belongs to collision front */
    if (rdot[b_i - 1] < -Opt_LLAvd->vel_noise_th_cmps) {
      /*  calculate safety radius */
      /* cm  */
      rej_nrm = mtlb_rt_roundf_snf((((float)Opt_LLAvd->LLAV_radius_min_cm + 3.0F
        * Opt_LLAvd->vel_max_cmps * (float)Opt_LLAvd->mr18_dt_msec * 0.001F) -
        rdot[b_i - 1] * (float)Opt_LLAvd->tau_theta_msec * 0.001F) + 0.005F *
        Opt_LLAvd->max_acc_mps2_inv * (rdot[b_i - 1] * rdot[b_i - 1]));
      if (rej_nrm < 65536.0F) {
        if (rej_nrm >= 0.0F) {
          r_tmp = (unsigned short)rej_nrm;
        } else {
          r_tmp = 0;
        }
      } else if (rej_nrm >= 65536.0F) {
        r_tmp = MAX_uint16_T;
      } else {
        r_tmp = 0;
      }

      UseVel[b_i - 1] = (signed char)(ranges[b_i - 1] < r_tmp);
    }

    uv2[0] = ranges_prev_prev[b_i - 1];
    uv2[1] = ranges_prev[b_i - 1];
    uv2[2] = ranges[b_i - 1];
    r_tmp = mtlb_median(uv2);
    if (r_tmp > Opt_LLAvd->TOF_radius_min_cm) {
      dt = (float)(Opt_LLAvd->rej_radius_mm - r_tmp * 10);
      if (dt > 0.0F) {
        rej_Cmd[0] -= Opt_LLAvd->rej_k_avoid * dt * sin_sen[s2c_i[b_i - 1] - 1];
        rej_Cmd[1] -= Opt_LLAvd->rej_k_avoid * dt * sin_sen[b_i - 1];

        /*  rejAmp(i) = Opt_LLAvd.rej_k_avoid*tmp_mm; */
        /*  UseRej(i) = true; */
      }
    }

    /*  prepare for next iteration */
    ranges_prev_prev[b_i - 1] = ranges_prev[b_i - 1];
    ranges_prev[b_i - 1] = ranges[b_i - 1];
    rdot_prev_prev[b_i - 1] = rdot_prev[b_i - 1];
    rdot_prev[b_i - 1] = rdot[b_i - 1];
  }

  TT_prev = TT;

  /* %% Simple rejection (force field): */
  rej_nrm = 0.0F;
  for (i = 0; i < 2; i++) {
    dt = rej_Cmd[i] * Opt_LLAvd->mps2Deg;
    rej_nrm += dt * dt;
    rej_Cmd[i] = dt;
  }

  mtlb_sqrt(&rej_nrm);
  if (rej_nrm < 0.001F) {
    rej_nrm = 0.001F;
  }

  for (i = 0; i < 2; i++) {
    rej_Cmd[i] /= rej_nrm;
  }

  if (rej_nrm > Opt_LLAvd->rej_avoidance_clamp) {
    rej_nrm = Opt_LLAvd->rej_avoidance_clamp;
  }

  for (i = 0; i < 2; i++) {
    rej_Cmd[i] = -rej_Cmd[i] * rej_nrm;
  }

  /*  calculate avoidance cmd */
  mtlb_abs(rdot, fv3);
  for (i = 0; i < 16; i++) {
    b_UseVel[i] = (float)UseVel[i] * fv3[i] * cos_sen_rev[i];
  }

  LLAvdCmd[0] = mtlb_b_sum(b_UseVel) * 0.01F;

  /* VX_body */
  mtlb_abs(rdot, fv3);
  for (i = 0; i < 16; i++) {
    b_UseVel[i] = (float)UseVel[i] * fv3[i] * sin_sen_rev[i];
  }

  LLAvdCmd[1] = mtlb_b_sum(b_UseVel) * 0.01F;

  /* VY_body */
  dt = -Opt_LLAvd->kFF_ang * Opt_LLAvd->mps2Deg;
  dt_inv = 0.0F;
  for (i = 0; i < 2; i++) {
    rej_nrm = dt * LLAvdCmd[i];
    dt_inv += rej_nrm * rej_nrm;
    LLAvdCmd[i] = rej_nrm;
  }

  mtlb_sqrt(&dt_inv);
  if (dt_inv < Opt_LLAvd->llav_deadB) {
    for (i = 0; i < 2; i++) {
      LLAvdCmd[i] = 0.0F;
    }
  }

  /* %% END OF Avoidance %%% */
  /*  disp('---') */
  /*  If current position is too close to last saved position, then dont save current position */
  for (i = 0; i < 2; i++) {
    i0 = xyPosWorld[(xyPosLastInd + 15 * i) - 1] - navXY[i];
    if (i0 > 32767) {
      i0 = 32767;
    } else {
      if (i0 < -32768) {
        i0 = -32768;
      }
    }

    xyHeadingAvg_nrm_vec[i] = (float)i0;
  }

  mtlb_power(xyHeadingAvg_nrm_vec, sumy2);
  for (i = 0; i < 2; i++) {
    xyHeadingAvg_nrm_vec[i] = sumy2[i];
  }

  if (mtlb_sum(xyHeadingAvg_nrm_vec) > Opt->posHistMinStep_p2) {
    qY = xyPosLastInd + 1U;
    if (qY > 65535U) {
      qY = 65535U;
    }

    if (xyPosLastInd == Opt->xyPosLen) {
      xyPosLastInd = 1;
    } else {
      xyPosLastInd = (unsigned short)qY;
    }

    for (i = 0; i < 2; i++) {
      xyPosWorld[(xyPosLastInd + 15 * i) - 1] = navXY[i];
    }
  }

  /*  byPass==true is for enabling 'slowDown' as a service. */
  /*  This is needed when Vnom is not a product of ransac algo. */
  /*  The following change intention: */
  /*  When byPass==false, code is logically equal as before change. */
  if ((freq_reduce >= Opt->nCyc_lowFreq) && (numSamp >= Opt->xyMeasLen)) {
    freq_reduce = 0;
    memcpy(&points[0], &xyMeasWorld[0], 200U * sizeof(short));

    /*  epsilon:      estimated fraction of outliers [0,1] */
    /*  if (coder.target('Matlab')) */
    /*      points = points(sum(points.^2,2)>0,:); */
    /*  end */
    /*  if nargin<3 */
    /*      P = 0.99; */
    /*      sampleSize = ceil(log(1-P)/log(1-(1-epsilon)^3)); */
    /*  end */
    tmp = 0;
    *validData = false;
    memset(&isInliers[0], 0, 100U * sizeof(boolean_T));

    /*  move to rob axis system: */
    for (b_i = 1; b_i <= Opt->xyMeasLen; b_i++) {
      b_sumy2[0] = cos_rob;
      b_sumy2[2] = sin_rob;
      b_sumy2[1] = -sin_rob;
      b_sumy2[3] = cos_rob;
      for (i = 0; i < 2; i++) {
        i0 = points[(b_i + 100 * i) - 1] - navXY[i];
        if (i0 > 32767) {
          i0 = 32767;
        } else {
          if (i0 < -32768) {
            i0 = -32768;
          }
        }

        xyMeasWorld_[i] = (short)i0;
      }

      for (i = 0; i < 2; i++) {
        rej_nrm = 0.0F;
        for (i0 = 0; i0 < 2; i0++) {
          rej_nrm += b_sumy2[i + (i0 << 1)] * (float)xyMeasWorld_[i0];
        }

        rej_nrm = mtlb_rt_roundf_snf(rej_nrm);
        if (rej_nrm < 32768.0F) {
          if (rej_nrm >= -32768.0F) {
            i1 = (short)rej_nrm;
          } else {
            i1 = MIN_int16_T;
          }
        } else if (rej_nrm >= 32768.0F) {
          i1 = MAX_int16_T;
        } else {
          i1 = 0;
        }

        points[(b_i + 100 * i) - 1] = i1;
      }
    }

    b_i = 1;
    while (b_i <= 12) {
      /* for i=1:sampleSize % RANSAC loop */
      for (i = 0; i < 2; i++) {
        sample[i << 1] = points[(rand_couple[(b_i - 1) << 1] + 100 * i) - 1];
        sample[1 + (i << 1)] = points[(rand_couple[1 + ((b_i - 1) << 1)] + 100 *
          i) - 1];
      }

      /*  line est */
      /*  ax + by = 1 */
      /*  a=cos(phi)/rho, b=sin(phi)/rho */
      rej_nrm = 0.0F;
      dt_inv = 0.0F;
      xyHeadingAvg_nrm = 0.0F;
      c_sumy2 = 0.0F;
      sumxy = 0.0F;
      for (kk = 1; kk < 3; kk++) {
        /* for kk=1:L */
        rej_nrm += (float)sample[kk - 1];
        dt_inv += (float)(sample[kk - 1] * sample[kk - 1]);
        xyHeadingAvg_nrm += (float)sample[kk + 1];
        c_sumy2 += (float)(sample[kk + 1] * sample[kk + 1]);
        sumxy += (float)(sample[kk + 1] * sample[kk - 1]);
      }

      /* HH = [sumx2 sumxy; sumxy sumy2]; */
      dt = c_sumy2 * dt_inv - sumxy * sumxy;
      if (dt < 1.0E-6F) {
        /*  line with rho close to zero */
        cond = true;
        dt = 1.0E+6F;
      } else {
        cond = false;
        dt = 1.0F / dt;
      }

      b_sumy2[0] = c_sumy2 * dt;
      b_sumy2[2] = -sumxy * dt;
      b_sumy2[1] = -sumxy * dt;
      b_sumy2[3] = dt_inv * dt;
      xyHeadingAvg_nrm_vec[0] = rej_nrm;
      xyHeadingAvg_nrm_vec[1] = xyHeadingAvg_nrm;
      theta0[2] = 1.0F;
      dt = 0.0F;
      for (i = 0; i < 2; i++) {
        sumy2[i] = 0.0F;
        for (i0 = 0; i0 < 2; i0++) {
          sumy2[i] += b_sumy2[i + (i0 << 1)] * xyHeadingAvg_nrm_vec[i0];
        }

        dt += sumy2[i] * sumy2[i];
        theta0[i] = sumy2[i];
      }

      dt = (float)sqrt(dt);
      for (i = 0; i < 3; i++) {
        theta0[i] /= dt;
      }

      if (cond) {
        b_i++;
      } else {
        memset(&dist[0], 0, 100U * sizeof(unsigned short));
        for (kk = 1; kk < 101; kk++) {
          /* for kk=1:L */
          dt = (float)fabs((theta0[0] * (float)points[kk - 1] + theta0[1] *
                            (float)points[kk + 99]) - theta0[2]);
          rej_nrm = mtlb_rt_roundf_snf(dt);
          if (rej_nrm < 65536.0F) {
            r_tmp = (unsigned short)rej_nrm;
          } else if (rej_nrm >= 65536.0F) {
            r_tmp = MAX_uint16_T;
          } else {
            r_tmp = 0;
          }

          if (dt > 10000.0F) {
            dist[kk - 1] = 10000;
          } else {
            dist[kk - 1] = r_tmp;
          }
        }

        for (i = 0; i < 100; i++) {
          in_vec[i] = (dist[i] < 5);
        }

        kk = 0;
        for (ii = 1; ii < 101; ii++) {
          i = (int)((unsigned int)kk + in_vec[ii - 1]);
          if ((unsigned int)i > 255U) {
            i = 255;
          }

          kk = (unsigned char)i;
        }

        if ((kk > 20) && (kk > tmp)) {
          /*  number of Inliers threshold */
          /*  take sample with biggest number of inliers */
          for (i = 0; i < 100; i++) {
            isInliers[i] = (dist[i] < 5);
          }

          tmp = kk;
          *validData = true;
        }

        b_i++;
      }
    }

    if (*validData) {
      /*  line est */
      /*  ax + by = 1 */
      /*  a=cos(phi)/rho, b=sin(phi)/rho */
      rej_nrm = 0.0F;
      dt_inv = 0.0F;
      xyHeadingAvg_nrm = 0.0F;
      c_sumy2 = 0.0F;
      sumxy = 0.0F;
      for (kk = 1; kk < 101; kk++) {
        /* for kk=1:L */
        if (isInliers[kk - 1]) {
          rej_nrm += (float)points[kk - 1];
          dt_inv += (float)(points[kk - 1] * points[kk - 1]);
          xyHeadingAvg_nrm += (float)points[kk + 99];
          c_sumy2 += (float)(points[kk + 99] * points[kk + 99]);
          sumxy += (float)(points[kk + 99] * points[kk - 1]);
        }
      }

      /* HH = [sumx2 sumxy; sumxy sumy2]; */
      dt = c_sumy2 * dt_inv - sumxy * sumxy;
      if (dt < 1.0E-6F) {
        /*  line with rho close to zero */
        dt = 1.0E+6F;
      } else {
        dt = 1.0F / dt;
      }

      b_sumy2[0] = c_sumy2 * dt;
      b_sumy2[2] = -sumxy * dt;
      b_sumy2[1] = -sumxy * dt;
      b_sumy2[3] = dt_inv * dt;
      xyHeadingAvg_nrm_vec[0] = rej_nrm;
      xyHeadingAvg_nrm_vec[1] = xyHeadingAvg_nrm;
      modelRANSAC[2] = 1.0F;
      dt = 0.0F;
      for (i = 0; i < 2; i++) {
        sumy2[i] = 0.0F;
        for (i0 = 0; i0 < 2; i0++) {
          sumy2[i] += b_sumy2[i + (i0 << 1)] * xyHeadingAvg_nrm_vec[i0];
        }

        dt += sumy2[i] * sumy2[i];
        modelRANSAC[i] = sumy2[i];
      }

      dt = (float)sqrt(dt);
      for (i = 0; i < 3; i++) {
        modelRANSAC[i] /= dt;
      }
    }
  } else {
    i = (int)(freq_reduce + 1U);
    if ((unsigned int)i > 255U) {
      i = 255;
    }

    freq_reduce = (unsigned char)i;
    *validData = false;
  }

  /*  not in byPass mode and valid: */
  if ((!byPass) && (*validData)) {
    cond = true;
  } else {
    cond = false;
  }

  /*  in byPass mode and by-pass counter is triggered */
  if (byPass && (freq_reduce_bp >= Opt->nCyc_lowFreq_bp) && (numSamp >=
       Opt->xyMeasLen)) {
    cond_bp = true;
  } else {
    cond_bp = false;
  }

  if (cond_bp || cond) {
    freq_reduce_bp = 0;
    if (byPass) {
      dt_inv = Vin[0];
      rej_nrm = Vin[1];
    } else if (modelRANSAC[1] > 0.0F) {
      rej_nrm = -modelRANSAC[0];
      dt_inv = modelRANSAC[1];
    } else {
      rej_nrm = modelRANSAC[0];
      dt_inv = -modelRANSAC[1];
    }

    for (b_i = 1; b_i < 17; b_i++) {
      if ((ranges[b_i - 1] > Opt->PF_radius_min) && (ranges[b_i - 1] <
           Opt->stopped_dist) && (sin_sen[s2c_i[b_i - 1] - 1] * dt_inv +
           sin_sen[b_i - 1] * rej_nrm > 0.97F)) {
        *slowDwn = 2;

        /* if  && xyHeadingAvg_nrm<0.3 */
        if (ranges[r_180[b_i - 1] - 1] > Opt->stopped_dist) {
          if (xyPosLastInd == 1) {
            for (i = 0; i < 2; i++) {
              i0 = xyPosWorld[(xyPosLastInd + 15 * i) - 1] - xyPosWorld
                [(Opt->xyPosLen + 15 * i) - 1];
              if (i0 > 32767) {
                i0 = 32767;
              } else {
                if (i0 < -32768) {
                  i0 = -32768;
                }
              }

              xyHeadingAvg_nrm_vec[i] = (float)i0;
            }
          } else {
            i = xyPosLastInd;
            qY = i - 1U;
            if (qY > (unsigned int)i) {
              qY = 0U;
            }

            i = (int)qY;
            for (i0 = 0; i0 < 2; i0++) {
              i2 = xyPosWorld[(xyPosLastInd + 15 * i0) - 1] - xyPosWorld[(i + 15
                * i0) - 1];
              if (i2 > 32767) {
                i2 = 32767;
              } else {
                if (i2 < -32768) {
                  i2 = -32768;
                }
              }

              xyHeadingAvg_nrm_vec[i0] = (float)i2;
            }
          }

          dt = 0.0F;
          for (i = 0; i < 2; i++) {
            dt += xyHeadingAvg_nrm_vec[i] * xyHeadingAvg_nrm_vec[i];
          }

          dt = (float)sqrt(dt);
          if (dt > 0.0F) {
            for (i = 0; i < 2; i++) {
              xyHeadingAvg_nrm_vec[i] /= dt;
            }

            if ((xyHeadingAvg_nrm_vec[0] * cos_rob + xyHeadingAvg_nrm_vec[1] *
                 sin_rob) * dt_inv + (-xyHeadingAvg_nrm_vec[0] * sin_rob +
                 xyHeadingAvg_nrm_vec[1] * cos_rob) * rej_nrm < 0.7F) {
              /*  todo : parametrize */
              *slowDwn = 3;
            }
          }
        }

        /* end */
      }
    }
  } else {
    i = (int)(freq_reduce_bp + 1U);
    if ((unsigned int)i > 255U) {
      i = 255;
    }

    freq_reduce_bp = (unsigned char)i;
  }

  if ((*validData) || cond_bp) {
    *validData = true;
  } else {
    *validData = false;
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void mtlb_firstRun_not_empty_init(void)
{
  firstRun_not_empty = false;
}

/*
 * File trailer for cf_feat_ext.c
 *
 * [EOF]
 */
