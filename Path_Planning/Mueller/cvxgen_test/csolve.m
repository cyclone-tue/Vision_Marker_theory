% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(square(jerk_0) + square(jerk_1) + square(jerk_2) + square(jerk_3) + square(jerk_4) + square(jerk_5) + square(jerk_6) + square(jerk_7) + square(jerk_8) + square(jerk_9) + square(jerk_10) + square(jerk_11) + square(jerk_12) + square(jerk_13) + square(jerk_14) + square(jerk_15) + square(jerk_16) + square(jerk_17) + square(jerk_18) + square(jerk_19) + square(jerk_20) + square(jerk_21) + square(jerk_22) + square(jerk_23) + square(jerk_24) + square(jerk_25) + square(jerk_26) + square(jerk_27) + square(jerk_28) + square(jerk_29) + square(jerk_30) + square(jerk_31) + square(jerk_32) + square(jerk_33) + square(jerk_34) + square(jerk_35) + square(jerk_36) + square(jerk_37) + square(jerk_38) + square(jerk_39) + square(jerk_40) + square(jerk_41) + square(jerk_42) + square(jerk_43) + square(jerk_44) + square(jerk_45) + square(jerk_46) + square(jerk_47) + square(jerk_48) + square(jerk_49) + square(jerk_50))
%   subject to
%     z_1 == A*z_0 + B*jerk_0
%     z_2 == A*z_1 + B*jerk_1
%     z_3 == A*z_2 + B*jerk_2
%     z_4 == A*z_3 + B*jerk_3
%     z_5 == A*z_4 + B*jerk_4
%     z_6 == A*z_5 + B*jerk_5
%     z_7 == A*z_6 + B*jerk_6
%     z_8 == A*z_7 + B*jerk_7
%     z_9 == A*z_8 + B*jerk_8
%     z_10 == A*z_9 + B*jerk_9
%     z_11 == A*z_10 + B*jerk_10
%     z_12 == A*z_11 + B*jerk_11
%     z_13 == A*z_12 + B*jerk_12
%     z_14 == A*z_13 + B*jerk_13
%     z_15 == A*z_14 + B*jerk_14
%     z_16 == A*z_15 + B*jerk_15
%     z_17 == A*z_16 + B*jerk_16
%     z_18 == A*z_17 + B*jerk_17
%     z_19 == A*z_18 + B*jerk_18
%     z_20 == A*z_19 + B*jerk_19
%     z_21 == A*z_20 + B*jerk_20
%     z_22 == A*z_21 + B*jerk_21
%     z_23 == A*z_22 + B*jerk_22
%     z_24 == A*z_23 + B*jerk_23
%     z_25 == A*z_24 + B*jerk_24
%     z_26 == A*z_25 + B*jerk_25
%     z_27 == A*z_26 + B*jerk_26
%     z_28 == A*z_27 + B*jerk_27
%     z_29 == A*z_28 + B*jerk_28
%     z_30 == A*z_29 + B*jerk_29
%     z_31 == A*z_30 + B*jerk_30
%     z_32 == A*z_31 + B*jerk_31
%     z_33 == A*z_32 + B*jerk_32
%     z_34 == A*z_33 + B*jerk_33
%     z_35 == A*z_34 + B*jerk_34
%     z_36 == A*z_35 + B*jerk_35
%     z_37 == A*z_36 + B*jerk_36
%     z_38 == A*z_37 + B*jerk_37
%     z_39 == A*z_38 + B*jerk_38
%     z_40 == A*z_39 + B*jerk_39
%     z_41 == A*z_40 + B*jerk_40
%     z_42 == A*z_41 + B*jerk_41
%     z_43 == A*z_42 + B*jerk_42
%     z_44 == A*z_43 + B*jerk_43
%     z_45 == A*z_44 + B*jerk_44
%     z_46 == A*z_45 + B*jerk_45
%     z_47 == A*z_46 + B*jerk_46
%     z_48 == A*z_47 + B*jerk_47
%     z_49 == A*z_48 + B*jerk_48
%     z_50 == A*z_49 + B*jerk_49
%     min_vel <= selectVelocity*z_0
%     min_vel <= selectVelocity*z_1
%     min_vel <= selectVelocity*z_2
%     min_vel <= selectVelocity*z_3
%     min_vel <= selectVelocity*z_4
%     min_vel <= selectVelocity*z_5
%     min_vel <= selectVelocity*z_6
%     min_vel <= selectVelocity*z_7
%     min_vel <= selectVelocity*z_8
%     min_vel <= selectVelocity*z_9
%     min_vel <= selectVelocity*z_10
%     min_vel <= selectVelocity*z_11
%     min_vel <= selectVelocity*z_12
%     min_vel <= selectVelocity*z_13
%     min_vel <= selectVelocity*z_14
%     min_vel <= selectVelocity*z_15
%     min_vel <= selectVelocity*z_16
%     min_vel <= selectVelocity*z_17
%     min_vel <= selectVelocity*z_18
%     min_vel <= selectVelocity*z_19
%     min_vel <= selectVelocity*z_20
%     min_vel <= selectVelocity*z_21
%     min_vel <= selectVelocity*z_22
%     min_vel <= selectVelocity*z_23
%     min_vel <= selectVelocity*z_24
%     min_vel <= selectVelocity*z_25
%     min_vel <= selectVelocity*z_26
%     min_vel <= selectVelocity*z_27
%     min_vel <= selectVelocity*z_28
%     min_vel <= selectVelocity*z_29
%     min_vel <= selectVelocity*z_30
%     min_vel <= selectVelocity*z_31
%     min_vel <= selectVelocity*z_32
%     min_vel <= selectVelocity*z_33
%     min_vel <= selectVelocity*z_34
%     min_vel <= selectVelocity*z_35
%     min_vel <= selectVelocity*z_36
%     min_vel <= selectVelocity*z_37
%     min_vel <= selectVelocity*z_38
%     min_vel <= selectVelocity*z_39
%     min_vel <= selectVelocity*z_40
%     min_vel <= selectVelocity*z_41
%     min_vel <= selectVelocity*z_42
%     min_vel <= selectVelocity*z_43
%     min_vel <= selectVelocity*z_44
%     min_vel <= selectVelocity*z_45
%     min_vel <= selectVelocity*z_46
%     min_vel <= selectVelocity*z_47
%     min_vel <= selectVelocity*z_48
%     min_vel <= selectVelocity*z_49
%     min_vel <= selectVelocity*z_50
%     selectVelocity*z_0 <= max_vel
%     selectVelocity*z_1 <= max_vel
%     selectVelocity*z_2 <= max_vel
%     selectVelocity*z_3 <= max_vel
%     selectVelocity*z_4 <= max_vel
%     selectVelocity*z_5 <= max_vel
%     selectVelocity*z_6 <= max_vel
%     selectVelocity*z_7 <= max_vel
%     selectVelocity*z_8 <= max_vel
%     selectVelocity*z_9 <= max_vel
%     selectVelocity*z_10 <= max_vel
%     selectVelocity*z_11 <= max_vel
%     selectVelocity*z_12 <= max_vel
%     selectVelocity*z_13 <= max_vel
%     selectVelocity*z_14 <= max_vel
%     selectVelocity*z_15 <= max_vel
%     selectVelocity*z_16 <= max_vel
%     selectVelocity*z_17 <= max_vel
%     selectVelocity*z_18 <= max_vel
%     selectVelocity*z_19 <= max_vel
%     selectVelocity*z_20 <= max_vel
%     selectVelocity*z_21 <= max_vel
%     selectVelocity*z_22 <= max_vel
%     selectVelocity*z_23 <= max_vel
%     selectVelocity*z_24 <= max_vel
%     selectVelocity*z_25 <= max_vel
%     selectVelocity*z_26 <= max_vel
%     selectVelocity*z_27 <= max_vel
%     selectVelocity*z_28 <= max_vel
%     selectVelocity*z_29 <= max_vel
%     selectVelocity*z_30 <= max_vel
%     selectVelocity*z_31 <= max_vel
%     selectVelocity*z_32 <= max_vel
%     selectVelocity*z_33 <= max_vel
%     selectVelocity*z_34 <= max_vel
%     selectVelocity*z_35 <= max_vel
%     selectVelocity*z_36 <= max_vel
%     selectVelocity*z_37 <= max_vel
%     selectVelocity*z_38 <= max_vel
%     selectVelocity*z_39 <= max_vel
%     selectVelocity*z_40 <= max_vel
%     selectVelocity*z_41 <= max_vel
%     selectVelocity*z_42 <= max_vel
%     selectVelocity*z_43 <= max_vel
%     selectVelocity*z_44 <= max_vel
%     selectVelocity*z_45 <= max_vel
%     selectVelocity*z_46 <= max_vel
%     selectVelocity*z_47 <= max_vel
%     selectVelocity*z_48 <= max_vel
%     selectVelocity*z_49 <= max_vel
%     selectVelocity*z_50 <= max_vel
%     min_acc <= selectAcceleration*z_0
%     min_acc <= selectAcceleration*z_1
%     min_acc <= selectAcceleration*z_2
%     min_acc <= selectAcceleration*z_3
%     min_acc <= selectAcceleration*z_4
%     min_acc <= selectAcceleration*z_5
%     min_acc <= selectAcceleration*z_6
%     min_acc <= selectAcceleration*z_7
%     min_acc <= selectAcceleration*z_8
%     min_acc <= selectAcceleration*z_9
%     min_acc <= selectAcceleration*z_10
%     min_acc <= selectAcceleration*z_11
%     min_acc <= selectAcceleration*z_12
%     min_acc <= selectAcceleration*z_13
%     min_acc <= selectAcceleration*z_14
%     min_acc <= selectAcceleration*z_15
%     min_acc <= selectAcceleration*z_16
%     min_acc <= selectAcceleration*z_17
%     min_acc <= selectAcceleration*z_18
%     min_acc <= selectAcceleration*z_19
%     min_acc <= selectAcceleration*z_20
%     min_acc <= selectAcceleration*z_21
%     min_acc <= selectAcceleration*z_22
%     min_acc <= selectAcceleration*z_23
%     min_acc <= selectAcceleration*z_24
%     min_acc <= selectAcceleration*z_25
%     min_acc <= selectAcceleration*z_26
%     min_acc <= selectAcceleration*z_27
%     min_acc <= selectAcceleration*z_28
%     min_acc <= selectAcceleration*z_29
%     min_acc <= selectAcceleration*z_30
%     min_acc <= selectAcceleration*z_31
%     min_acc <= selectAcceleration*z_32
%     min_acc <= selectAcceleration*z_33
%     min_acc <= selectAcceleration*z_34
%     min_acc <= selectAcceleration*z_35
%     min_acc <= selectAcceleration*z_36
%     min_acc <= selectAcceleration*z_37
%     min_acc <= selectAcceleration*z_38
%     min_acc <= selectAcceleration*z_39
%     min_acc <= selectAcceleration*z_40
%     min_acc <= selectAcceleration*z_41
%     min_acc <= selectAcceleration*z_42
%     min_acc <= selectAcceleration*z_43
%     min_acc <= selectAcceleration*z_44
%     min_acc <= selectAcceleration*z_45
%     min_acc <= selectAcceleration*z_46
%     min_acc <= selectAcceleration*z_47
%     min_acc <= selectAcceleration*z_48
%     min_acc <= selectAcceleration*z_49
%     min_acc <= selectAcceleration*z_50
%     selectAcceleration*z_0 <= max_acc
%     selectAcceleration*z_1 <= max_acc
%     selectAcceleration*z_2 <= max_acc
%     selectAcceleration*z_3 <= max_acc
%     selectAcceleration*z_4 <= max_acc
%     selectAcceleration*z_5 <= max_acc
%     selectAcceleration*z_6 <= max_acc
%     selectAcceleration*z_7 <= max_acc
%     selectAcceleration*z_8 <= max_acc
%     selectAcceleration*z_9 <= max_acc
%     selectAcceleration*z_10 <= max_acc
%     selectAcceleration*z_11 <= max_acc
%     selectAcceleration*z_12 <= max_acc
%     selectAcceleration*z_13 <= max_acc
%     selectAcceleration*z_14 <= max_acc
%     selectAcceleration*z_15 <= max_acc
%     selectAcceleration*z_16 <= max_acc
%     selectAcceleration*z_17 <= max_acc
%     selectAcceleration*z_18 <= max_acc
%     selectAcceleration*z_19 <= max_acc
%     selectAcceleration*z_20 <= max_acc
%     selectAcceleration*z_21 <= max_acc
%     selectAcceleration*z_22 <= max_acc
%     selectAcceleration*z_23 <= max_acc
%     selectAcceleration*z_24 <= max_acc
%     selectAcceleration*z_25 <= max_acc
%     selectAcceleration*z_26 <= max_acc
%     selectAcceleration*z_27 <= max_acc
%     selectAcceleration*z_28 <= max_acc
%     selectAcceleration*z_29 <= max_acc
%     selectAcceleration*z_30 <= max_acc
%     selectAcceleration*z_31 <= max_acc
%     selectAcceleration*z_32 <= max_acc
%     selectAcceleration*z_33 <= max_acc
%     selectAcceleration*z_34 <= max_acc
%     selectAcceleration*z_35 <= max_acc
%     selectAcceleration*z_36 <= max_acc
%     selectAcceleration*z_37 <= max_acc
%     selectAcceleration*z_38 <= max_acc
%     selectAcceleration*z_39 <= max_acc
%     selectAcceleration*z_40 <= max_acc
%     selectAcceleration*z_41 <= max_acc
%     selectAcceleration*z_42 <= max_acc
%     selectAcceleration*z_43 <= max_acc
%     selectAcceleration*z_44 <= max_acc
%     selectAcceleration*z_45 <= max_acc
%     selectAcceleration*z_46 <= max_acc
%     selectAcceleration*z_47 <= max_acc
%     selectAcceleration*z_48 <= max_acc
%     selectAcceleration*z_49 <= max_acc
%     selectAcceleration*z_50 <= max_acc
%     min_jerk <= jerk_0
%     min_jerk <= jerk_1
%     min_jerk <= jerk_2
%     min_jerk <= jerk_3
%     min_jerk <= jerk_4
%     min_jerk <= jerk_5
%     min_jerk <= jerk_6
%     min_jerk <= jerk_7
%     min_jerk <= jerk_8
%     min_jerk <= jerk_9
%     min_jerk <= jerk_10
%     min_jerk <= jerk_11
%     min_jerk <= jerk_12
%     min_jerk <= jerk_13
%     min_jerk <= jerk_14
%     min_jerk <= jerk_15
%     min_jerk <= jerk_16
%     min_jerk <= jerk_17
%     min_jerk <= jerk_18
%     min_jerk <= jerk_19
%     min_jerk <= jerk_20
%     min_jerk <= jerk_21
%     min_jerk <= jerk_22
%     min_jerk <= jerk_23
%     min_jerk <= jerk_24
%     min_jerk <= jerk_25
%     min_jerk <= jerk_26
%     min_jerk <= jerk_27
%     min_jerk <= jerk_28
%     min_jerk <= jerk_29
%     min_jerk <= jerk_30
%     min_jerk <= jerk_31
%     min_jerk <= jerk_32
%     min_jerk <= jerk_33
%     min_jerk <= jerk_34
%     min_jerk <= jerk_35
%     min_jerk <= jerk_36
%     min_jerk <= jerk_37
%     min_jerk <= jerk_38
%     min_jerk <= jerk_39
%     min_jerk <= jerk_40
%     min_jerk <= jerk_41
%     min_jerk <= jerk_42
%     min_jerk <= jerk_43
%     min_jerk <= jerk_44
%     min_jerk <= jerk_45
%     min_jerk <= jerk_46
%     min_jerk <= jerk_47
%     min_jerk <= jerk_48
%     min_jerk <= jerk_49
%     min_jerk <= jerk_50
%     jerk_0 <= max_jerk
%     jerk_1 <= max_jerk
%     jerk_2 <= max_jerk
%     jerk_3 <= max_jerk
%     jerk_4 <= max_jerk
%     jerk_5 <= max_jerk
%     jerk_6 <= max_jerk
%     jerk_7 <= max_jerk
%     jerk_8 <= max_jerk
%     jerk_9 <= max_jerk
%     jerk_10 <= max_jerk
%     jerk_11 <= max_jerk
%     jerk_12 <= max_jerk
%     jerk_13 <= max_jerk
%     jerk_14 <= max_jerk
%     jerk_15 <= max_jerk
%     jerk_16 <= max_jerk
%     jerk_17 <= max_jerk
%     jerk_18 <= max_jerk
%     jerk_19 <= max_jerk
%     jerk_20 <= max_jerk
%     jerk_21 <= max_jerk
%     jerk_22 <= max_jerk
%     jerk_23 <= max_jerk
%     jerk_24 <= max_jerk
%     jerk_25 <= max_jerk
%     jerk_26 <= max_jerk
%     jerk_27 <= max_jerk
%     jerk_28 <= max_jerk
%     jerk_29 <= max_jerk
%     jerk_30 <= max_jerk
%     jerk_31 <= max_jerk
%     jerk_32 <= max_jerk
%     jerk_33 <= max_jerk
%     jerk_34 <= max_jerk
%     jerk_35 <= max_jerk
%     jerk_36 <= max_jerk
%     jerk_37 <= max_jerk
%     jerk_38 <= max_jerk
%     jerk_39 <= max_jerk
%     jerk_40 <= max_jerk
%     jerk_41 <= max_jerk
%     jerk_42 <= max_jerk
%     jerk_43 <= max_jerk
%     jerk_44 <= max_jerk
%     jerk_45 <= max_jerk
%     jerk_46 <= max_jerk
%     jerk_47 <= max_jerk
%     jerk_48 <= max_jerk
%     jerk_49 <= max_jerk
%     jerk_50 <= max_jerk
%     z_0 == initial
%     z_50 == final
%
% with variables
%   jerk_0   1 x 1
%   jerk_1   1 x 1
%   jerk_2   1 x 1
%   jerk_3   1 x 1
%   jerk_4   1 x 1
%   jerk_5   1 x 1
%   jerk_6   1 x 1
%   jerk_7   1 x 1
%   jerk_8   1 x 1
%   jerk_9   1 x 1
%  jerk_10   1 x 1
%  jerk_11   1 x 1
%  jerk_12   1 x 1
%  jerk_13   1 x 1
%  jerk_14   1 x 1
%  jerk_15   1 x 1
%  jerk_16   1 x 1
%  jerk_17   1 x 1
%  jerk_18   1 x 1
%  jerk_19   1 x 1
%  jerk_20   1 x 1
%  jerk_21   1 x 1
%  jerk_22   1 x 1
%  jerk_23   1 x 1
%  jerk_24   1 x 1
%  jerk_25   1 x 1
%  jerk_26   1 x 1
%  jerk_27   1 x 1
%  jerk_28   1 x 1
%  jerk_29   1 x 1
%  jerk_30   1 x 1
%  jerk_31   1 x 1
%  jerk_32   1 x 1
%  jerk_33   1 x 1
%  jerk_34   1 x 1
%  jerk_35   1 x 1
%  jerk_36   1 x 1
%  jerk_37   1 x 1
%  jerk_38   1 x 1
%  jerk_39   1 x 1
%  jerk_40   1 x 1
%  jerk_41   1 x 1
%  jerk_42   1 x 1
%  jerk_43   1 x 1
%  jerk_44   1 x 1
%  jerk_45   1 x 1
%  jerk_46   1 x 1
%  jerk_47   1 x 1
%  jerk_48   1 x 1
%  jerk_49   1 x 1
%  jerk_50   1 x 1
%      z_0   3 x 1
%      z_1   3 x 1
%      z_2   3 x 1
%      z_3   3 x 1
%      z_4   3 x 1
%      z_5   3 x 1
%      z_6   3 x 1
%      z_7   3 x 1
%      z_8   3 x 1
%      z_9   3 x 1
%     z_10   3 x 1
%     z_11   3 x 1
%     z_12   3 x 1
%     z_13   3 x 1
%     z_14   3 x 1
%     z_15   3 x 1
%     z_16   3 x 1
%     z_17   3 x 1
%     z_18   3 x 1
%     z_19   3 x 1
%     z_20   3 x 1
%     z_21   3 x 1
%     z_22   3 x 1
%     z_23   3 x 1
%     z_24   3 x 1
%     z_25   3 x 1
%     z_26   3 x 1
%     z_27   3 x 1
%     z_28   3 x 1
%     z_29   3 x 1
%     z_30   3 x 1
%     z_31   3 x 1
%     z_32   3 x 1
%     z_33   3 x 1
%     z_34   3 x 1
%     z_35   3 x 1
%     z_36   3 x 1
%     z_37   3 x 1
%     z_38   3 x 1
%     z_39   3 x 1
%     z_40   3 x 1
%     z_41   3 x 1
%     z_42   3 x 1
%     z_43   3 x 1
%     z_44   3 x 1
%     z_45   3 x 1
%     z_46   3 x 1
%     z_47   3 x 1
%     z_48   3 x 1
%     z_49   3 x 1
%     z_50   3 x 1
%
% and parameters
%        A   3 x 3
%        B   3 x 1
%    final   3 x 1
%  initial   3 x 1
%  max_acc   1 x 1
% max_jerk   1 x 1
%  max_vel   1 x 1
%  min_acc   1 x 1
% min_jerk   1 x 1
%  min_vel   1 x 1
% selectAcceleration   1 x 3
% selectVelocity   1 x 3
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.selectVelocity, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2019-01-17 08:59:28 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
