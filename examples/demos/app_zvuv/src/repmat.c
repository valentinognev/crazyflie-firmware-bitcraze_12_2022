/*
 * File: repmat.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 08-Nov-2021 11:14:21
 */

/* Include Files */
//#include "rt_nonfinite.h"  // removed by ROEE
#include "cf_feat_ext.h"
#include "feat2cmd.h"
#include "init.h"
#include "repmat.h"

/* Function Definitions */

/*
 * Arguments    : const short a[2]
 *                short b[30]
 * Return Type  : void
 */
void mtlb_repmat(const short a[2], short b[30])
{
  int jcol;
  int ibmat;
  int itilerow;
  for (jcol = 0; jcol < 2; jcol++) {
    ibmat = jcol * 15;
    for (itilerow = 0; itilerow < 15; itilerow++) {
      b[ibmat + itilerow] = a[jcol];
    }
  }
}

/*
 * File trailer for repmat.c
 *
 * [EOF]
 */
