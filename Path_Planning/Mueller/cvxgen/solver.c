/* Produced by CVXGEN, 2019-02-04 10:09:24 -0500.  */
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
  for (i = 0; i < 194; i++)
    gap += work.z[i]*work.s[i];
  return gap;
}
void set_defaults(void) {
  settings.resid_tol = 1e-6;
  settings.eps = 1e-4;
  settings.max_iters = 25;
  settings.refine_steps = 1;
  settings.s_init = 1;
  settings.z_init = 1;
  settings.debug = 0;
  settings.verbose = 1;
  settings.verbose_refinement = 0;
  settings.better_start = 1;
  settings.kkt_reg = 1e-7;
}
void setup_pointers(void) {
  work.y = work.x + 195;
  work.s = work.x + 345;
  work.z = work.x + 539;
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
  vars.jerk_36 = work.x + 36;
  vars.jerk_37 = work.x + 37;
  vars.jerk_38 = work.x + 38;
  vars.jerk_39 = work.x + 39;
  vars.jerk_40 = work.x + 40;
  vars.jerk_41 = work.x + 41;
  vars.jerk_42 = work.x + 42;
  vars.jerk_43 = work.x + 43;
  vars.jerk_44 = work.x + 44;
  vars.jerk_45 = work.x + 45;
  vars.jerk_46 = work.x + 46;
  vars.jerk_47 = work.x + 47;
  vars.z_0 = work.x + 48;
  vars.z_1 = work.x + 51;
  vars.z_2 = work.x + 54;
  vars.z_3 = work.x + 57;
  vars.z_4 = work.x + 60;
  vars.z_5 = work.x + 63;
  vars.z_6 = work.x + 66;
  vars.z_7 = work.x + 69;
  vars.z_8 = work.x + 72;
  vars.z_9 = work.x + 75;
  vars.z_10 = work.x + 78;
  vars.z_11 = work.x + 81;
  vars.z_12 = work.x + 84;
  vars.z_13 = work.x + 87;
  vars.z_14 = work.x + 90;
  vars.z_15 = work.x + 93;
  vars.z_16 = work.x + 96;
  vars.z_17 = work.x + 99;
  vars.z_18 = work.x + 102;
  vars.z_19 = work.x + 105;
  vars.z_20 = work.x + 108;
  vars.z_21 = work.x + 111;
  vars.z_22 = work.x + 114;
  vars.z_23 = work.x + 117;
  vars.z_24 = work.x + 120;
  vars.z_25 = work.x + 123;
  vars.z_26 = work.x + 126;
  vars.z_27 = work.x + 129;
  vars.z_28 = work.x + 132;
  vars.z_29 = work.x + 135;
  vars.z_30 = work.x + 138;
  vars.z_31 = work.x + 141;
  vars.z_32 = work.x + 144;
  vars.z_33 = work.x + 147;
  vars.z_34 = work.x + 150;
  vars.z_35 = work.x + 153;
  vars.z_36 = work.x + 156;
  vars.z_37 = work.x + 159;
  vars.z_38 = work.x + 162;
  vars.z_39 = work.x + 165;
  vars.z_40 = work.x + 168;
  vars.z_41 = work.x + 171;
  vars.z_42 = work.x + 174;
  vars.z_43 = work.x + 177;
  vars.z_44 = work.x + 180;
  vars.z_45 = work.x + 183;
  vars.z_46 = work.x + 186;
  vars.z_47 = work.x + 189;
  vars.z_48 = work.x + 192;
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
  vars.jerk[36] = vars.jerk_36;
  vars.jerk[37] = vars.jerk_37;
  vars.jerk[38] = vars.jerk_38;
  vars.jerk[39] = vars.jerk_39;
  vars.jerk[40] = vars.jerk_40;
  vars.jerk[41] = vars.jerk_41;
  vars.jerk[42] = vars.jerk_42;
  vars.jerk[43] = vars.jerk_43;
  vars.jerk[44] = vars.jerk_44;
  vars.jerk[45] = vars.jerk_45;
  vars.jerk[46] = vars.jerk_46;
  vars.jerk[47] = vars.jerk_47;
  vars.z[1] = vars.z_1;
  vars.z[0] = vars.z_0;
  vars.z[2] = vars.z_2;
  vars.z[3] = vars.z_3;
  vars.z[4] = vars.z_4;
  vars.z[5] = vars.z_5;
  vars.z[6] = vars.z_6;
  vars.z[7] = vars.z_7;
  vars.z[8] = vars.z_8;
  vars.z[9] = vars.z_9;
  vars.z[10] = vars.z_10;
  vars.z[11] = vars.z_11;
  vars.z[12] = vars.z_12;
  vars.z[13] = vars.z_13;
  vars.z[14] = vars.z_14;
  vars.z[15] = vars.z_15;
  vars.z[16] = vars.z_16;
  vars.z[17] = vars.z_17;
  vars.z[18] = vars.z_18;
  vars.z[19] = vars.z_19;
  vars.z[20] = vars.z_20;
  vars.z[21] = vars.z_21;
  vars.z[22] = vars.z_22;
  vars.z[23] = vars.z_23;
  vars.z[24] = vars.z_24;
  vars.z[25] = vars.z_25;
  vars.z[26] = vars.z_26;
  vars.z[27] = vars.z_27;
  vars.z[28] = vars.z_28;
  vars.z[29] = vars.z_29;
  vars.z[30] = vars.z_30;
  vars.z[31] = vars.z_31;
  vars.z[32] = vars.z_32;
  vars.z[33] = vars.z_33;
  vars.z[34] = vars.z_34;
  vars.z[35] = vars.z_35;
  vars.z[36] = vars.z_36;
  vars.z[37] = vars.z_37;
  vars.z[38] = vars.z_38;
  vars.z[39] = vars.z_39;
  vars.z[40] = vars.z_40;
  vars.z[41] = vars.z_41;
  vars.z[42] = vars.z_42;
  vars.z[43] = vars.z_43;
  vars.z[44] = vars.z_44;
  vars.z[45] = vars.z_45;
  vars.z[46] = vars.z_46;
  vars.z[47] = vars.z_47;
  vars.z[48] = vars.z_48;
}
void setup_indexing(void) {
  setup_pointers();
  setup_indexed_optvars();
}
void set_start(void) {
  int i;
  for (i = 0; i < 195; i++)
    work.x[i] = 0;
  for (i = 0; i < 150; i++)
    work.y[i] = 0;
  for (i = 0; i < 194; i++)
    work.s[i] = (work.h[i] > 0) ? work.h[i] : settings.s_init;
  for (i = 0; i < 194; i++)
    work.z[i] = settings.z_init;
}
double eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in work.rhs. */
  multbyP(work.rhs, work.x);
  objv = 0;
  for (i = 0; i < 195; i++)
    objv += work.x[i]*work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 195; i++)
    objv += work.q[i]*work.x[i];
  objv += 0;
  return objv;
}
void fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = work.rhs;
  r2 = work.rhs + 195;
  r3 = work.rhs + 389;
  r4 = work.rhs + 583;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  multbymAT(r1, work.y);
  multbymGT(work.buffer, work.z);
  for (i = 0; i < 195; i++)
    r1[i] += work.buffer[i];
  multbyP(work.buffer, work.x);
  for (i = 0; i < 195; i++)
    r1[i] -= work.buffer[i] + work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 194; i++)
    r2[i] = -work.z[i];
  /* r3 = -Gx - s + h. */
  multbymG(r3, work.x);
  for (i = 0; i < 194; i++)
    r3[i] += -work.s[i] + work.h[i];
  /* r4 = -Ax + b. */
  multbymA(r4, work.x);
  for (i = 0; i < 150; i++)
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
  r2 = work.rhs + 195;
  ds_aff = work.lhs_aff + 195;
  dz_aff = work.lhs_aff + 389;
  mu = 0;
  for (i = 0; i < 194; i++)
    mu += work.s[i]*work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 194; i++)
    if (ds_aff[i] < minval*work.s[i])
      minval = ds_aff[i]/work.s[i];
  for (i = 0; i < 194; i++)
    if (dz_aff[i] < minval*work.z[i])
      minval = dz_aff[i]/work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 194; i++)
    sigma += (work.s[i] + alpha*ds_aff[i])*
      (work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.005154639175257732;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 195; i++)
    work.rhs[i] = 0;
  for (i = 389; i < 733; i++)
    work.rhs[i] = 0;
  for (i = 0; i < 194; i++)
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
    for (i = 0; i < 733; i++) {
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
    for (i = 0; i < 733; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 733; i++) {
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
  for (i = 0; i < 194; i++)
    work.buffer[i] += -work.s[i] + work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 194; i++)
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
  for (i = 0; i < 150; i++)
    work.buffer[i] += work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 150; i++)
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
  for (i = 0; i < 194; i++)
    work.s_inv_z[i] = 1;
  fill_KKT();
  ldl_factor();
  fillrhs_start();
  /* Borrow work.lhs_aff for the solution. */
  ldl_solve(work.rhs, work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = work.lhs_aff;
  s = work.lhs_aff + 195;
  z = work.lhs_aff + 389;
  y = work.lhs_aff + 583;
  /* Just set x and y as is. */
  for (i = 0; i < 195; i++)
    work.x[i] = x[i];
  for (i = 0; i < 150; i++)
    work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 194; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 194; i++)
      work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 194; i++)
      work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 194; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 194; i++)
      work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 194; i++)
      work.z[i] = z[i] + alpha;
  }
}
void fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = work.rhs;
  r2 = work.rhs + 195;
  r3 = work.rhs + 389;
  r4 = work.rhs + 583;
  for (i = 0; i < 195; i++)
    r1[i] = -work.q[i];
  for (i = 0; i < 194; i++)
    r2[i] = 0;
  for (i = 0; i < 194; i++)
    r3[i] = work.h[i];
  for (i = 0; i < 150; i++)
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
    for (i = 0; i < 194; i++) {
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
    for (i = 0; i < 733; i++)
      work.lhs_aff[i] += work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = work.lhs_aff;
    ds = work.lhs_aff + 195;
    dz = work.lhs_aff + 389;
    dy = work.lhs_aff + 583;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 194; i++)
      if (ds[i] < minval*work.s[i])
        minval = ds[i]/work.s[i];
    for (i = 0; i < 194; i++)
      if (dz[i] < minval*work.z[i])
        minval = dz[i]/work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 195; i++)
      work.x[i] += alpha*dx[i];
    for (i = 0; i < 194; i++)
      work.s[i] += alpha*ds[i];
    for (i = 0; i < 194; i++)
      work.z[i] += alpha*dz[i];
    for (i = 0; i < 150; i++)
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
