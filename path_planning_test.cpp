//#include "opencv2/aruco.hpp"
//#include "opencv2/highgui.hpp"
#include <iostream>
//#include <opencv/cv.hpp>			//contains CommandLineParser, aruco
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
//#include <opencv2/core/eigen.hpp>

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::DiagonalMatrix;

//using namespace cv;
//using namespace std;
//using namespace Eigen;

VectorXd coef_vec2(6);

namespace {
    const char* keys =
            "{cal |    | File to load calibration data from}"
            "{cam | 0  | camera input to use}";
    const double markerSize = 0.07; // Marker side length in meters
    const int markerId = 23;
    const double d_after = 1;
    const double v_after = 1;
    const double d_before = 2;
    const double v_in = 1;
    //const Vec3d hoop_offset = Vec3d(0,0,0);//Offset variable in world space from center of marker to center of hoop.
}


/*

Let a vector "constraints" be given by [p(t=0); pd(t=0); pdd(t=0); p(t=time); pd(t=time); pdd(t=time)].
Let p(t) = a0*t^5 + a1*t^4 + ... a5*t^0, for a vector "a" with elements [a0, ... a5].

The output matrix satisfies M*a = constraints.

*/
MatrixXd getMatForDerToConstraints(double time){

	// constructing upper part of matrix
	
	MatrixXd upperPart(3, 6);
	upperPart << 0,0,0,0,0,1, 0,0,0,0,1,0, 0,0,0,2,0,0;
	
	// constructing lower part of matrix
	//time = 3;		// uncomment this line to see result of old version of code.
	VectorXd powersOfTime(6);
	powersOfTime << pow(time,5), pow(time,4), pow(time,3), pow(time,2), pow(time,1), 1;
	DiagonalMatrix<double, 6> powersOfTimeDiag = powersOfTime.asDiagonal();
	
	MatrixXd lowerPart(3, 6);
	lowerPart << 1,1,1,1,1,1, 5,4,3,2,1,0, 20,12,6,2,0,0;
	lowerPart = lowerPart*powersOfTimeDiag;
	
	// constructing final matrix
	MatrixXd mat(6,6);
    mat.block<3,6>(0,0) = upperPart;
	mat.block<3,6>(3,0) = lowerPart;
	
	return mat;
}


/*

The constraints vector contains [p(t=0); pd(t=0); pdd(t=0); p(t=1); pd(t=1); pdd(t=1)].
It holds that p(t) = a0*t^5 + a1*t^4 + ... a5*t^0, for a vector "a" with elements [a0, ... a5].

The output matrix satisfies the following:
For each row i with elements [e0, ... e5] of the output matrix, it is satisfied that p_i = e0*t^5 + e1*t^4 + ... e5*t^0.
Here p_i is the i'th derivative p. The matrix has 4 rows.

*/
MatrixXd constraintsToDerivatives(VectorXd constraints, double t) {
	
	MatrixXd M = getMatForDerToConstraints(t);		// There exists a matrix M(endTime), satisfying M*a = constraints.
	
    VectorXd a = M.fullPivLu().solve(constraints);

    MatrixXd mat_out(4, 6);
    mat_out << a(0),a(1),a(2),a(3),a(4),a(5),  0,5*a(0),4*a(1),3*a(2),2*a(3),a(4),  0,0,20*a(0),12*a(1),6*a(2),2*a(3),  0,0,0,60*a(0),24*a(1),6*a(2);
	
    return mat_out;
}


/*

The input matrix should satisfy the following:
For each row i with elements [a0, ... a5] of the input matrix, there is a corresponding variable p_i, for which p_i = a0*t^5 + a1*t^4 + ... a5*t^0.
Also, p_(i+1) is the derivative of p_i. The matrix has 4 rows.

The output is a list of states, each of the form [p_0, p_1, p_2, p_3], at the times [endTime/numOfStates, ... , endTime].

*/
MatrixXd expansionToStateList(double numOfStates, MatrixXd mat, double endTime) {
	
    MatrixXd states(4, 50);		// states = [p, pd, pdd, pddd] for several times; state(t = 0) is not included in states						

    double dt = endTime/numOfStates;

	
    for (double t = dt; t <= endTime+0.001; t += dt) {
        VectorXd powersOfTime(6);
        powersOfTime << pow(t,5), pow(t,4), pow(t,3), pow(t,2), pow(t,1), 1;	// powers of time

        MatrixXd state(4, 1);
        state = mat * powersOfTime;									// intermediate state calculation

        int stateIndex = (int) round(numOfStates*t/endTime) - 1;
        for (int j = 0; j <= 3; ++j) {								// populate the nth column of states with state(t)
            states(j, stateIndex) = state(j);
        }
    }
	
    return states;
}


/*

The input format should be as follows:
constraints = [[x0; xd0; xdd0; x1; xd1; xdd1], [y0; ... ], [z0; ... ]] 

The output is a list of states, each of the form state = [x,xd,xdd,xddd, y,yd, ... z,zd, ... ], at times [time/numOfStates, ... , time].
The begin and end constraints are satisfied by this path.

*/
MatrixXd getPath(double numOfStates, MatrixXd constraints, double time) {
	
    MatrixXd states(12,(int) numOfStates);			// state = [x,xd,xdd,xddd, y,yd, ... z,zd, ... ]

    for (int j = 0; j <= 2; j++) {			//iterate over parts(x, y, z) of states
        MatrixXd derMat(4,6);
        derMat = constraintsToDerivatives(constraints.col(j), time);

        states.block<4,50>(j*4, 0) = expansionToStateList(numOfStates, derMat, time);
    }

    return states;	
}






// argument count, argument vector
int main(int argc, char* argv[]){
	MatrixXd constraints(6,3);
	constraints << 0,0,0, 0,0,0, 0,0,0, 2,1,-0.5, 0,0,0, 0,0,0;
	
	MatrixXd path(12, 50);
	path = getPath(50, constraints, 3);		// mind the format of the returned states.
	for(int i=0; i<50; i++){
		std::cout << path(0,i) << "\t\t" << path(4,i) << "\t\t" << path(8,i) << "\n";
	}
	return 0;
}





// run code using g++ -I/usr/include/eigen3 test.cpp














