/*
 * File: power.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 08-Nov-2021 11:14:21
 */

/* Include Files */
//#include "rt_nonfinite.h"  // removed by ROEE
#include "cf_feat_ext.h"
#include "feat2cmd.h"
#include "init.h"
#include "mtlb_power.h"

/* Function Definitions */

/*
 * Arguments    : const float a[2]
 *                float y[2]
 * Return Type  : void
 */
void mtlb_power(const float a[2], float y[2])
{
  int k;
  for (k = 0; k < 2; k++) {
    y[k] = a[k] * a[k];
  }
}

/*
 * File trailer for power.c
 *
 * [EOF]
 */
