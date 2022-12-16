/*
 * File: feat2cmd.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 08-Nov-2021 11:14:21
 */

/* Include Files */
//#include "rt_nonfinite.h"  // removed by ROEE
#include "cf_feat_ext.h"
#include "feat2cmd.h"
#include "init.h"

/* Variable Definitions */
static signed char slowDwnCnt;
static signed char slowDwnPosCnt;
static float slowDwnFilt;

/* Function Definitions */

/*
 * Arguments    : const float modelRANSAC[3]
 *                float D_half
 *                unsigned char slowDwn
 *                const Cntrl_type *Cntrl
 *                float *psi_cmd
 *                float *Vxcmd
 *                float *Vycmd
 *                float *Vxnom
 *                float *Vynom
 *                boolean_T *stopped
 *                float *slowDwnFilt_out
 * Return Type  : void
 */
void feat2cmd(const float modelRANSAC[3], float D_half, unsigned char slowDwn,
              const Cntrl_type *Cntrl, float *psi_cmd, float *Vxcmd, float
              *Vycmd, float *Vxnom, float *Vynom, boolean_T *stopped, float
              *slowDwnFilt_out)
{
  int sf;
  int i3;
  float sin_err;
  float d;
  float cos_err;
  sf = 1;
  *stopped = false;
  i3 = slowDwnCnt + 1;
  if (i3 > 127) {
    i3 = 127;
  }

  if (slowDwn >= 1) {
    slowDwnCnt = (signed char)i3;
  } else {
    slowDwnCnt = 0;
  }

  i3 = slowDwnPosCnt + 1;
  if (i3 > 127) {
    i3 = 127;
  }

  if (slowDwn == 2) {
    slowDwnPosCnt = (signed char)i3;
  } else {
    slowDwnPosCnt = 0;
  }

  if (slowDwnCnt > Cntrl->sf_deaccel_cnt) {
    sf = 3;
  }

  if (slowDwnPosCnt > Cntrl->stopped_consec_cnt) {
    *stopped = true;
  }

  if (slowDwn >= 1) {
    /*  slowDwnFilt = slowDwnFilt*(1-Cntrl.Vnom_slw_filt*sf); */
    slowDwnFilt -= Cntrl->Vnom_slw_filt * (float)sf;
    if (slowDwnFilt < Cntrl->Vnom_min_rtio) {
      slowDwnFilt = Cntrl->Vnom_min_rtio;
    }
  } else {
    slowDwnFilt += Cntrl->Vnom_acc_filt;
    if (slowDwnFilt > 1.0F) {
      slowDwnFilt = 1.0F;
    }
  }

  if (slowDwnFilt < Cntrl->Vnom_min_rtio) {
    slowDwnFilt = Cntrl->Vnom_min_rtio;
  }

  if (modelRANSAC[1] > 0.0F) {
    sin_err = modelRANSAC[0];
    d = D_half - modelRANSAC[2];
    cos_err = modelRANSAC[1];
  } else {
    sin_err = -modelRANSAC[0];
    d = -D_half + modelRANSAC[2];
    cos_err = -modelRANSAC[1];
  }

  *Vxcmd = -Cntrl->Kdx * d * sin_err;
  *Vycmd = -Cntrl->Kdy * d * cos_err;
  if (slowDwn == 3) {
    *Vxnom = -Cntrl->Vnom * cos_err * slowDwnFilt;
    *Vynom = Cntrl->Vnom * sin_err * slowDwnFilt;
    *psi_cmd = Cntrl->Kpsi * sin_err;
  } else {
    *psi_cmd = -Cntrl->Kpsi * sin_err;
    *Vxnom = Cntrl->Vnom * cos_err * slowDwnFilt;
    *Vynom = -Cntrl->Vnom * sin_err * slowDwnFilt;
  }

  *slowDwnFilt_out = slowDwnFilt;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void mtlb_feat2cmd_init(void)
{
  slowDwnFilt = 1.0F;
  slowDwnCnt = 0;
  slowDwnPosCnt = 0;
}

/*
 * File trailer for feat2cmd.c
 *
 * [EOF]
 */
