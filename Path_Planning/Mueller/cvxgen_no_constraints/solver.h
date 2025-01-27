/* Produced by CVXGEN, 2019-02-18 06:55:49 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double A[9];
  double B[3];
  double initial[3];
  double final[3];
} Params;
typedef struct Vars_t {
  double *jerk_0; /* 1 rows. */
  double *jerk_1; /* 1 rows. */
  double *jerk_2; /* 1 rows. */
  double *jerk_3; /* 1 rows. */
  double *jerk_4; /* 1 rows. */
  double *jerk_5; /* 1 rows. */
  double *jerk_6; /* 1 rows. */
  double *jerk_7; /* 1 rows. */
  double *jerk_8; /* 1 rows. */
  double *jerk_9; /* 1 rows. */
  double *jerk_10; /* 1 rows. */
  double *jerk_11; /* 1 rows. */
  double *jerk_12; /* 1 rows. */
  double *jerk_13; /* 1 rows. */
  double *jerk_14; /* 1 rows. */
  double *jerk_15; /* 1 rows. */
  double *jerk_16; /* 1 rows. */
  double *jerk_17; /* 1 rows. */
  double *jerk_18; /* 1 rows. */
  double *jerk_19; /* 1 rows. */
  double *jerk_20; /* 1 rows. */
  double *jerk_21; /* 1 rows. */
  double *jerk_22; /* 1 rows. */
  double *jerk_23; /* 1 rows. */
  double *jerk_24; /* 1 rows. */
  double *jerk_25; /* 1 rows. */
  double *jerk_26; /* 1 rows. */
  double *jerk_27; /* 1 rows. */
  double *jerk_28; /* 1 rows. */
  double *jerk_29; /* 1 rows. */
  double *jerk_30; /* 1 rows. */
  double *jerk_31; /* 1 rows. */
  double *jerk_32; /* 1 rows. */
  double *jerk_33; /* 1 rows. */
  double *jerk_34; /* 1 rows. */
  double *jerk_35; /* 1 rows. */
  double *jerk_36; /* 1 rows. */
  double *jerk_37; /* 1 rows. */
  double *jerk_38; /* 1 rows. */
  double *jerk_39; /* 1 rows. */
  double *jerk_40; /* 1 rows. */
  double *x_1; /* 3 rows. */
  double *x_0; /* 3 rows. */
  double *x_2; /* 3 rows. */
  double *x_3; /* 3 rows. */
  double *x_4; /* 3 rows. */
  double *x_5; /* 3 rows. */
  double *x_6; /* 3 rows. */
  double *x_7; /* 3 rows. */
  double *x_8; /* 3 rows. */
  double *x_9; /* 3 rows. */
  double *x_10; /* 3 rows. */
  double *x_11; /* 3 rows. */
  double *x_12; /* 3 rows. */
  double *x_13; /* 3 rows. */
  double *x_14; /* 3 rows. */
  double *x_15; /* 3 rows. */
  double *x_16; /* 3 rows. */
  double *x_17; /* 3 rows. */
  double *x_18; /* 3 rows. */
  double *x_19; /* 3 rows. */
  double *x_20; /* 3 rows. */
  double *x_21; /* 3 rows. */
  double *x_22; /* 3 rows. */
  double *x_23; /* 3 rows. */
  double *x_24; /* 3 rows. */
  double *x_25; /* 3 rows. */
  double *x_26; /* 3 rows. */
  double *x_27; /* 3 rows. */
  double *x_28; /* 3 rows. */
  double *x_29; /* 3 rows. */
  double *x_30; /* 3 rows. */
  double *x_31; /* 3 rows. */
  double *x_32; /* 3 rows. */
  double *x_33; /* 3 rows. */
  double *x_34; /* 3 rows. */
  double *x_35; /* 3 rows. */
  double *x_36; /* 3 rows. */
  double *x_37; /* 3 rows. */
  double *x_38; /* 3 rows. */
  double *x_39; /* 3 rows. */
  double *x_40; /* 3 rows. */
  double *x_41; /* 3 rows. */
  double *jerk[41];
  double *x[42];
} Vars;
typedef struct Workspace_t {
  double *h;
  double *s_inv;
  double *s_inv_z;
  double b[129];
  double q[167];
  double rhs[296];
  double x[296];
  double *s;
  double *z;
  double *y;
  double lhs_aff[296];
  double lhs_cc[296];
  double buffer[296];
  double buffer2[296];
  double KKT[662];
  double L[1104];
  double d[296];
  double v[296];
  double d_inv[296];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
//int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
