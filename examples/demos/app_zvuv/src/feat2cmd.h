/*
 * File: feat2cmd.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 08-Nov-2021 11:14:21
 */

#ifndef FEAT2CMD_H
#define FEAT2CMD_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "cf_feat_ext_types.h"

/* Function Declarations */
#ifdef __cplusplus

extern "C" {

#endif

  extern void feat2cmd(const float modelRANSAC[3], float D_half, unsigned char
                       slowDwn, const Cntrl_type *Cntrl, float *psi_cmd, float
                       *Vxcmd, float *Vycmd, float *Vxnom, float *Vynom,
                       boolean_T *stopped, float *slowDwnFilt_out);
  extern void mtlb_feat2cmd_init(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for feat2cmd.h
 *
 * [EOF]
 */
