/* Produced by CVXGEN, 2019-02-11 08:42:11 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "solver.h"
double eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 216; i++)
    gap += work.z[i]*work.s[i];
  return gap;
}
void set_defaults(void) {
  settings.resid_tol = 1e-8;  //6
  settings.eps = 1e-6; //4
  settings.max_iters = 2500;
  settings.refine_steps = 4;  //1
  settings.s_init = 1;
  settings.z_init = 1;
  settings.debug = 0;
  settings.verbose = 1;
  settings.verbose_refinement = 0;
  settings.better_start = 1;
  settings.kkt_reg = 1e-7;
}
void setup_pointers(void) {
  work.y = work.x + 255;
  work.s = work.x + 261;
  work.z = work.x + 477;
  vars.jerk_0 = work.x + 0;
  vars.jerk_1 = work.x + 1;
  vars.jerk_2 = work.x + 2;
  vars.jerk_3 = work.x + 3;
  vars.jerk_4 = work.x + 4;
  vars.jerk_5 = work.x + 5;
  vars.jerk_6 = work.x + 6;
  vars.jerk_7 = work.x + 7;
  vars.jerk_8 = work.x + 8;
  vars.jerk_9 = work.x + 9;
  vars.jerk_10 = work.x + 10;
  vars.jerk_11 = work.x + 11;
  vars.jerk_12 = work.x + 12;
  vars.jerk_13 = work.x + 13;
  vars.jerk_14 = work.x + 14;
  vars.jerk_15 = work.x + 15;
  vars.jerk_16 = work.x + 16;
  vars.jerk_17 = work.x + 17;
  vars.jerk_18 = work.x + 18;
  vars.jerk_19 = work.x + 19;
  vars.jerk_20 = work.x + 20;
  vars.jerk_21 = work.x + 21;
  vars.jerk_22 = work.x + 22;
  vars.jerk_23 = work.x + 23;
  vars.jerk_24 = work.x + 24;
  vars.jerk_25 = work.x + 25;
  vars.jerk_26 = work.x + 26;
  vars.jerk_27 = work.x + 27;
  vars.jerk_28 = work.x + 28;
  vars.jerk_29 = work.x + 29;
  vars.jerk_30 = work.x + 30;
  vars.jerk_31 = work.x + 31;
  vars.jerk_32 = work.x + 32;
  vars.jerk_33 = work.x + 33;
  vars.jerk_34 = work.x + 34;
  vars.jerk_35 = work.x + 35;
  vars.x_0 = work.x + 144;
  vars.x_1 = work.x + 147;
  vars.x_2 = work.x + 150;
  vars.x_3 = work.x + 153;
  vars.x_4 = work.x + 156;
  vars.x_5 = work.x + 159;
  vars.x_6 = work.x + 162;
  vars.x_7 = work.x + 165;
  vars.x_8 = work.x + 168;
  vars.x_9 = work.x + 171;
  vars.x_10 = work.x + 174;
  vars.x_11 = work.x + 177;
  vars.x_12 = work.x + 180;
  vars.x_13 = work.x + 183;
  vars.x_14 = work.x + 186;
  vars.x_15 = work.x + 189;
  vars.x_16 = work.x + 192;
  vars.x_17 = work.x + 195;
  vars.x_18 = work.x + 198;
  vars.x_19 = work.x + 201;
  vars.x_20 = work.x + 204;
  vars.x_21 = work.x + 207;
  vars.x_22 = work.x + 210;
  vars.x_23 = work.x + 213;
  vars.x_24 = work.x + 216;
  vars.x_25 = work.x + 219;
  vars.x_26 = work.x + 222;
  vars.x_27 = work.x + 225;
  vars.x_28 = work.x + 228;
  vars.x_29 = work.x + 231;
  vars.x_30 = work.x + 234;
  vars.x_31 = work.x + 237;
  vars.x_32 = work.x + 240;
  vars.x_33 = work.x + 243;
  vars.x_34 = work.x + 246;
  vars.x_35 = work.x + 249;
  vars.x_36 = work.x + 252;
}
void setup_indexed_optvars(void) {
  /* In CVXGEN, you can say */
  /*   variables */
  /*     x[i] (5), i=2..4 */
  /*   end */
  /* This function sets up x[3] to be a pointer to x_3, which is a length-5 */
  /* vector of doubles. */
  /* If you access variables that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  vars.jerk[0] = vars.jerk_0;
  vars.jerk[1] = vars.jerk_1;
  vars.jerk[2] = vars.jerk_2;
  vars.jerk[3] = vars.jerk_3;
  vars.jerk[4] = vars.jerk_4;
  vars.jerk[5] = vars.jerk_5;
  vars.jerk[6] = vars.jerk_6;
  vars.jerk[7] = vars.jerk_7;
  vars.jerk[8] = vars.jerk_8;
  vars.jerk[9] = vars.jerk_9;
  vars.jerk[10] = vars.jerk_10;
  vars.jerk[11] = vars.jerk_11;
  vars.jerk[12] = vars.jerk_12;
  vars.jerk[13] = vars.jerk_13;
  vars.jerk[14] = vars.jerk_14;
  vars.jerk[15] = vars.jerk_15;
  vars.jerk[16] = vars.jerk_16;
  vars.jerk[17] = vars.jerk_17;
  vars.jerk[18] = vars.jerk_18;
  vars.jerk[19] = vars.jerk_19;
  vars.jerk[20] = vars.jerk_20;
  vars.jerk[21] = vars.jerk_21;
  vars.jerk[22] = vars.jerk_22;
  vars.jerk[23] = vars.jerk_23;
  vars.jerk[24] = vars.jerk_24;
  vars.jerk[25] = vars.jerk_25;
  vars.jerk[26] = vars.jerk_26;
  vars.jerk[27] = vars.jerk_27;
  vars.jerk[28] = vars.jerk_28;
  vars.jerk[29] = vars.jerk_29;
  vars.jerk[30] = vars.jerk_30;
  vars.jerk[31] = vars.jerk_31;
  vars.jerk[32] = vars.jerk_32;
  vars.jerk[33] = vars.jerk_33;
  vars.jerk[34] = vars.jerk_34;
  vars.jerk[35] = vars.jerk_35;
  vars.x[1] = vars.x_1;
  vars.x[0] = vars.x_0;
  vars.x[2] = vars.x_2;
  vars.x[3] = vars.x_3;
  vars.x[4] = vars.x_4;
  vars.x[5] = vars.x_5;
  vars.x[6] = vars.x_6;
  vars.x[7] = vars.x_7;
  vars.x[8] = vars.x_8;
  vars.x[9] = vars.x_9;
  vars.x[10] = vars.x_10;
  vars.x[11] = vars.x_11;
  vars.x[12] = vars.x_12;
  vars.x[13] = vars.x_13;
  vars.x[14] = vars.x_14;
  vars.x[15] = vars.x_15;
  vars.x[16] = vars.x_16;
  vars.x[17] = vars.x_17;
  vars.x[18] = vars.x_18;
  vars.x[19] = vars.x_19;
  vars.x[20] = vars.x_20;
  vars.x[21] = vars.x_21;
  vars.x[22] = vars.x_22;
  vars.x[23] = vars.x_23;
  vars.x[24] = vars.x_24;
  vars.x[25] = vars.x_25;
  vars.x[26] = vars.x_26;
  vars.x[27] = vars.x_27;
  vars.x[28] = vars.x_28;
  vars.x[29] = vars.x_29;
  vars.x[30] = vars.x_30;
  vars.x[31] = vars.x_31;
  vars.x[32] = vars.x_32;
  vars.x[33] = vars.x_33;
  vars.x[34] = vars.x_34;
  vars.x[35] = vars.x_35;
  vars.x[36] = vars.x_36;
}
void setup_indexing(void) {
  setup_pointers();
  setup_indexed_optvars();
}
void set_start(void) {
  int i;
  for (i = 0; i < 255; i++)
    work.x[i] = 0;
  for (i = 0; i < 6; i++)
    work.y[i] = 0;
  for (i = 0; i < 216; i++)
    work.s[i] = (work.h[i] > 0) ? work.h[i] : settings.s_init;
  for (i = 0; i < 216; i++)
    work.z[i] = settings.z_init;
}
double eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in work.rhs. */
  multbyP(work.rhs, work.x);
  objv = 0;
  for (i = 0; i < 255; i++)
    objv += work.x[i]*work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 255; i++)
    objv += work.q[i]*work.x[i];
  objv += 0;
  return objv;
}
void fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = work.rhs;
  r2 = work.rhs + 255;
  r3 = work.rhs + 471;
  r4 = work.rhs + 687;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  multbymAT(r1, work.y);
  multbymGT(work.buffer, work.z);
  for (i = 0; i < 255; i++)
    r1[i] += work.buffer[i];
  multbyP(work.buffer, work.x);
  for (i = 0; i < 255; i++)
    r1[i] -= work.buffer[i] + work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 216; i++)
    r2[i] = -work.z[i];
  /* r3 = -Gx - s + h. */
  multbymG(r3, work.x);
  for (i = 0; i < 216; i++)
    r3[i] += -work.s[i] + work.h[i];
  /* r4 = -Ax + b. */
  multbymA(r4, work.x);
  for (i = 0; i < 6; i++)
    r4[i] += work.b[i];
}
void fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = work.rhs + 255;
  ds_aff = work.lhs_aff + 255;
  dz_aff = work.lhs_aff + 471;
  mu = 0;
  for (i = 0; i < 216; i++)
    mu += work.s[i]*work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 216; i++)
    if (ds_aff[i] < minval*work.s[i])
      minval = ds_aff[i]/work.s[i];
  for (i = 0; i < 216; i++)
    if (dz_aff[i] < minval*work.z[i])
      minval = dz_aff[i]/work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 216; i++)
    sigma += (work.s[i] + alpha*ds_aff[i])*
      (work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.004629629629629629;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 255; i++)
    work.rhs[i] = 0;
  for (i = 471; i < 693; i++)
    work.rhs[i] = 0;
  for (i = 0; i < 216; i++)
    r2[i] = work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void refine(double *target, double *var) {
  int i, j;
  double *residual = work.buffer;
  double norm2;
  double *new_var = work.buffer2;
  for (j = 0; j < settings.refine_steps; j++) {
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 693; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    ldl_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 693; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 693; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
    if (j == 0)
      printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
    else
      printf("After refinement we get squared norm %.6g.\n", norm2);
  }
#endif
}
double calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  multbymG(work.buffer, work.x);
  /* Add -s + h. */
  for (i = 0; i < 216; i++)
    work.buffer[i] += -work.s[i] + work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 216; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];
  return norm2_squared;
}
double calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  multbymA(work.buffer, work.x);
  /* Add +b. */
  for (i = 0; i < 6; i++)
    work.buffer[i] += work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 6; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];
  return norm2_squared;
}
void better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 216; i++)
    work.s_inv_z[i] = 1;
  fill_KKT();
  ldl_factor();
  fillrhs_start();
  /* Borrow work.lhs_aff for the solution. */
  ldl_solve(work.rhs, work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = work.lhs_aff;
  s = work.lhs_aff + 255;
  z = work.lhs_aff + 471;
  y = work.lhs_aff + 687;
  /* Just set x and y as is. */
  for (i = 0; i < 255; i++)
    work.x[i] = x[i];
  for (i = 0; i < 6; i++)
    work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 216; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 216; i++)
      work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 216; i++)
      work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 216; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 216; i++)
      work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 216; i++)
      work.z[i] = z[i] + alpha;
  }
}
void fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = work.rhs;
  r2 = work.rhs + 255;
  r3 = work.rhs + 471;
  r4 = work.rhs + 687;
  for (i = 0; i < 255; i++)
    r1[i] = -work.q[i];
  for (i = 0; i < 216; i++)
    r2[i] = 0;
  for (i = 0; i < 216; i++)
    r3[i] = work.h[i];
  for (i = 0; i < 6; i++)
    r4[i] = work.b[i];
}
long solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  work.converged = 0;
  setup_pointers();
  pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  fillq();
  fillh();
  fillb();
  if (settings.better_start)
    better_start();
  else
    set_start();
  for (iter = 0; iter < settings.max_iters; iter++) {
    for (i = 0; i < 216; i++) {
      work.s_inv[i] = 1.0 / work.s[i];
      work.s_inv_z[i] = work.s_inv[i]*work.z[i];
    }
    work.block_33[0] = 0;
    fill_KKT();
    ldl_factor();
    /* Affine scaling directions. */
    fillrhs_aff();
    ldl_solve(work.rhs, work.lhs_aff);
    refine(work.rhs, work.lhs_aff);
    /* Centering plus corrector directions. */
    fillrhs_cc();
    ldl_solve(work.rhs, work.lhs_cc);
    refine(work.rhs, work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 693; i++)
      work.lhs_aff[i] += work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = work.lhs_aff;
    ds = work.lhs_aff + 255;
    dz = work.lhs_aff + 471;
    dy = work.lhs_aff + 687;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 216; i++)
      if (ds[i] < minval*work.s[i])
        minval = ds[i]/work.s[i];
    for (i = 0; i < 216; i++)
      if (dz[i] < minval*work.z[i])
        minval = dz[i]/work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 255; i++)
      work.x[i] += alpha*dx[i];
    for (i = 0; i < 216; i++)
      work.s[i] += alpha*ds[i];
    for (i = 0; i < 216; i++)
      work.z[i] += alpha*dz[i];
    for (i = 0; i < 6; i++)
      work.y[i] += alpha*dy[i];
    work.gap = eval_gap();
    work.eq_resid_squared = calc_eq_resid_squared();
    work.ineq_resid_squared = calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (settings.verbose) {
      work.optval = eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, work.optval, work.gap, sqrt(work.eq_resid_squared),
          sqrt(work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (work.gap < settings.eps)
        && (work.eq_resid_squared <= settings.resid_tol*settings.resid_tol)
        && (work.ineq_resid_squared <= settings.resid_tol*settings.resid_tol)
       ) {
      work.converged = 1;
      work.optval = eval_objv();
      return iter+1;
    }
  }
  return iter;
}
