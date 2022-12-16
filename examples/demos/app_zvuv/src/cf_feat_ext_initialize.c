/*
 * File: cf_feat_ext_initialize.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 08-Nov-2021 11:14:21
 */

/* Include Files */
//#include "rt_nonfinite.h"  // removed by ROEE
#include "cf_feat_ext.h"
#include "feat2cmd.h"
#include "init.h"
#include "cf_feat_ext_initialize.h"
#include "myRansac.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void cf_feat_ext_initialize(void)
{
  //rt_InitInfAndNaN(8U); // removed by ROEE
  mtlb_firstRun_not_empty_init();
  mtlb_myRansac_init();
  mtlb_feat2cmd_init();
}

/*
 * File trailer for cf_feat_ext_initialize.c
 *
 * [EOF]
 */
