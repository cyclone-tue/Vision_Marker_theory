/* Produced by CVXGEN, 2018-11-20 15:26:18 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.A[0] = 0.20319161029830202;
  params.A[1] = 0.8325912904724193;
  params.A[2] = -0.8363810443482227;
  params.A[3] = 0.04331042079065206;
  params.A[4] = 1.5717878173906188;
  params.A[5] = 1.5851723557337523;
  params.A[6] = -1.497658758144655;
  params.A[7] = -1.171028487447253;
  params.A[8] = -1.7941311867966805;
  params.B[0] = -0.23676062539745413;
  params.B[1] = -1.8804951564857322;
  params.B[2] = -0.17266710242115568;
  params.min_vel[0] = 0.596576190459043;
  params.selectVelocity[0] = -0.8860508694080989;
  params.selectVelocity[1] = 0.7050196079205251;
  params.selectVelocity[2] = 0.3634512696654033;
  params.max_vel[0] = -1.9040724704913385;
  params.min_acc[0] = 0.23541635196352795;
  params.selectAcceleration[0] = -0.9629902123701384;
  params.selectAcceleration[1] = -0.3395952119597214;
  params.selectAcceleration[2] = -0.865899672914725;
  params.max_acc[0] = 0.7725516732519853;
  params.min_jerk[0] = -0.23818512931704205;
  params.max_jerk[0] = -1.372529046100147;
  params.initial[0] = 0.17859607212737894;
  params.initial[1] = 1.1212590580454682;
  params.initial[2] = -0.774545870495281;
  params.final[0] = -1.1121684642712744;
  params.final[1] = -0.44811496977740495;
  params.final[2] = 1.7455345994417217;
}
