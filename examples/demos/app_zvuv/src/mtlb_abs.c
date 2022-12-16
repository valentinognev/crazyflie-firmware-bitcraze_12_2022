/*
 * File: abs.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 08-Nov-2021 11:14:21
 */

/* Include Files */
//#include "rt_nonfinite.h"  // removed by ROEE
#include "cf_feat_ext.h"
#include "feat2cmd.h"
#include "init.h"
#include "mtlb_abs.h"

/* Function Definitions */

/*
 * Arguments    : const float x[16]
 *                float y[16]
 * Return Type  : void
 */
void mtlb_abs(const float x[16], float y[16])
{
  int k;
  for (k = 0; k < 16; k++) {
    y[k] = (float)fabs(x[k]);
  }
}

/*
 * File trailer for abs.c
 *
 * [EOF]
 */
