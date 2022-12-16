/*
 * File: median.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 08-Nov-2021 11:14:21
 */

/* Include Files */
//#include "rt_nonfinite.h"  // removed by ROEE
#include "cf_feat_ext.h"
#include "feat2cmd.h"
#include "init.h"
#include "mtlb_median.h"

/* Function Definitions */

/*
 * Arguments    : const unsigned short x[3]
 * Return Type  : unsigned short
 */
unsigned short mtlb_median(const unsigned short x[3])
{
  int j2;
  if (x[0] < x[1]) {
    if (x[1] < x[2]) {
      j2 = 1;
    } else if (x[0] < x[2]) {
      j2 = 2;
    } else {
      j2 = 0;
    }
  } else if (x[0] < x[2]) {
    j2 = 0;
  } else if (x[1] < x[2]) {
    j2 = 2;
  } else {
    j2 = 1;
  }

  return x[j2];
}

/*
 * File trailer for median.c
 *
 * [EOF]
 */
