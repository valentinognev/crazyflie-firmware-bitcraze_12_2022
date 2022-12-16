/*
 * File: sum.c
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

/* Function Definitions */

/*
 * Arguments    : const float x[16]
 * Return Type  : float
 */
float mtlb_b_sum(const float x[16])
{
  float y;
  int k;
  y = x[0];
  for (k = 0; k < 15; k++) {
    y += x[k + 1];
  }

  return y;
}

/*
 * Arguments    : const float x[2]
 * Return Type  : float
 */
float mtlb_sum(const float x[2])
{
  return x[0] + x[1];
}

/*
 * File trailer for sum.c
 *
 * [EOF]
 */
