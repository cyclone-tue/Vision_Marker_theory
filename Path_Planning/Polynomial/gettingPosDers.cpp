
#include "path_planner.h"




/*
Let a vector "constraints" be given by [p(t=0); pd(t=0); pdd(t=0); p(t=time); pd(t=time); pdd(t=time)].
Let p(t) = taylor(reverse(a), t), for a vector "a" with elements [a0, ... a5].
The output matrix satisfies M*a = constraints.
*/
MatrixXd path_planner::getMatForDersToConstraints(double time){

    // constructing upper part of matrix
    MatrixXd upperPart(3, 6);
    upperPart << 0,0,0,0,0,1, 0,0,0,0,1,0, 0,0,0,1,0,0;

    // constructing lower part of matrix
    double t5 = pow(time,5);
    double t4 = pow(time,4);
    double t3 = pow(time,3);
    double t2 = pow(time,2);
    double t1 = pow(time,1);
    MatrixXd lowerPart(3, 6);
    lowerPart << 1./120*t5,1./24*t4,1./6*t3,1./2*t2,t1,1, 1./24*t4,1./6*t3,1./2*t2,t1,1,0, 1./6*t3,1./2*t2,t1,1,0,0;

    // constructing final matrix
    MatrixXd mat(6,6);
    mat.block<3,6>(0,0) = upperPart;
    mat.block<3,6>(3,0) = lowerPart;

    return mat;
}


/*
The constraints vector contains [p(t=0); pd(t=0); pdd(t=0); p(t=endT); pd(t=endT); pdd(t=endT)].
It holds that p(t) = a0*t^5 + a1*t^4 + ... a5*t^0, for a vector "a" with elements [a0, ... a5].
The output matrix satisfies the following:  // this seems incorrect. only first row of described matrix is returned.
For each row i with elements [e0, ... e5] of the output matrix, it is satisfied that p_i = e0*t^5 + e1*t^4 + ... e5*t^0.
Here p_i is the i'th derivative p. The matrix has 4 rows.
*/
VectorXd path_planner::constraintsToDers(VectorXd constraints, double t) {

    MatrixXd mat(6,6);
    mat = getMatForDersToConstraints(t);			// There exists a matrix M(endTime), satisfying M*ders = constraints.
    VectorXd ders(6);

    ders = mat.fullPivLu().solve(constraints);
    VectorXd ders2 = ders.reverse();


    return ders2;
}

/*
MatrixXd path_planner::dersToTimeDependenceOfDim(MatrixXd a) {

    MatrixXd mat(4, 6);
    mat << 1/120*a(0),1/24*a(1),1/6*a(2),1/2*a(3),a(4),a(5),  0,1/24*a(0),1/6*a(1),1/2*a(2),a(3),a(4),  0,0,1/6*a(0),1/2*a(1),a(2),a(3),  0,0,0,1/2*a(0),a(1),a(2);

    return mat;
}*/


/*
The input format should be as follows:
// constraints = [[x0, xd0, xdd0]; [y0, ... ]; [z0, ... ]; [x1, xd1, xdd1]; [y1, ... ]; [z1, ... ]]
The output is [[x;xd;xdd; ... xddddd], [y;yd; ...], [z;zd; ...]] at time = 0.
*/
MatrixXd path_planner::getDersXYZ(MatrixXd constraints, double time) {

    // dersXYZ = [[x;xd;xdd; ... xddddd], [y;yd; ...], [z;zd; ...]]

    MatrixXd dersXYZ(6, 3);

    for (int j = 0; j <= 2; j++) {			//iterate over parts(x, y, z) of states
        VectorXd ders(6);
        VectorXd constraintsOneDim(6);      // [x0, xd0, xdd0, x1, xd1, xdd1]
        constraintsOneDim << constraints(j,0), constraints(j,1), constraints(j,2), constraints(j+3,0), constraints(j+3,1), constraints(j+3,2);

        ders = constraintsToDers(constraintsOneDim, time);

        for(int i = 0; i < 6; i++){
            dersXYZ(i, j) = ders(i);
        }
    }

    return dersXYZ;
}



double path_planner::maxJerk(MatrixXd dersXYZ, double time){

    // dersXYZ = [[x;xd;xdd; ... xddddd], [y;yd; ...], [z;zd; ...]]; derivatives at t=0

    double tX = time/2;
    double tY = time/2;
    double tZ = time/2;

    VectorXd dersx(4);
    VectorXd dersy(4);
    VectorXd dersz(4);
    dersx = dersXYZ.block<3,1>(3,0);
    dersy = dersXYZ.block<3,1>(3,1);
    dersz = dersXYZ.block<3,1>(3,2);

    double accX = taylor(dersx, tX);
    double accY = taylor(dersy, tY);
    double accZ = taylor(dersz, tZ);

    double maxAcc = sqrt(pow(accX,2) + pow(accY,2) + pow(accZ,2));

    return maxAcc;
}



/*
// dersXYZ = [[x;xd;xdd; ... xddddd], [y;yd; ...], [z;zd; ...]]; derivatives at t=0
*/
double path_planner::getPathDers(MatrixXd& constraints, MatrixXd& dersXYZ) {
    // constraints = [[x0, xd0, xdd0]; [y0, ... ]; [z0, ... ]; [x1, xd1, xdd1]; [y1, ... ]; [z1, ... ]]

    double maxAcc = 1;

    double t_min = 0;
    double t_max = 10;
    double t;

    do {                        // make sure t_max yields trajectory with valid thrusts
        t_max = t_max * 2;

        dersXYZ = getDersXYZ(constraints, t_max);



    } while (maxJerk(dersXYZ, t_max) >= maxAcc);

    cout << "valid" << endl;

    while (t_max - t_min > 0.1) {
        t = (t_max + t_min) / 2;
        dersXYZ = getDersXYZ(constraints, t);

        if (maxJerk(dersXYZ, t) <= maxAcc) {
            t_max = t;
        } else {
            t_min = t;
        }
    }

    dersXYZ = getDersXYZ(constraints, t_max);

    return t_max;
}





