/*
 * File: myRansac.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 08-Nov-2021 11:14:21
 */

/* Include Files */
//#include "rt_nonfinite.h"  // removed by ROEE
#include "cf_feat_ext.h"
#include "feat2cmd.h"
#include "init.h"
#include "myRansac.h"
#include "cf_feat_ext_data.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void mtlb_myRansac_init(void)
{
  int i4;
  static const unsigned char uv3[60] = { 17U, 84U, 9U, 27U, 72U, 58U, 75U, 17U,
    54U, 49U, 4U, 68U, 80U, 30U, 91U, 75U, 11U, 38U, 13U, 96U, 85U, 90U, 46U,
    61U, 18U, 16U, 37U, 46U, 5U, 34U, 21U, 80U, 30U, 94U, 92U, 8U, 3U, 78U, 49U,
    69U, 82U, 36U, 68U, 33U, 7U, 53U, 41U, 24U, 86U, 39U, 38U, 40U, 36U, 93U,
    20U, 35U, 88U, 51U, 6U, 32U };

  for (i4 = 0; i4 < 60; i4++) {
    rand_couple[i4] = uv3[i4];
  }
}

/*
 * File trailer for myRansac.c
 *
 * [EOF]
 */
