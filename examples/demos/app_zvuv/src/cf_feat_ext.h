/*
 * File: cf_feat_ext.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 08-Nov-2021 11:14:21
 */

#ifndef CF_FEAT_EXT_H
#define CF_FEAT_EXT_H

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

  extern void cf_feat_ext(const short navXY[2], float Psi, const unsigned short
    ranges[16], const Opt_type *Opt, const Opt_LLAvd_type *Opt_LLAvd, unsigned
    int TT, const float Vin[2], boolean_T byPass, float modelRANSAC[3],
    boolean_T *validData, unsigned char *slowDwn, float LLAvdCmd[2], float
    rej_Cmd[2]);
  extern void mtlb_firstRun_not_empty_init(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for cf_feat_ext.h
 *
 * [EOF]
 */
