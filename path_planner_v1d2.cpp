
#include <iostream>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::DiagonalMatrix;

using namespace std;
using namespace Eigen;

VectorXd coef_vec2(6);

namespace {
    const double d_after = 1;
    const double v_after = 1;
    const double d_before = -2;
    const double v_in = 1;
	
    const Vec3d hoop_offset = Vec3d(0,0,0);//Offset variable in world space from center of marker to center of hoop.
}



/*

Let a vector "constraints" be given by [p(t=0); pd(t=0); pdd(t=0); p(t=time); pd(t=time); pdd(t=time)].
Let p(t) = a0*t^5 + a1*t^4 + ... a5*t^0, for a vector "a" with elements [a0, ... a5].

The output matrix satisfies M*a = constraints.

*/
MatrixXd getMatForDersToConstraints(double time){

	// constructing upper part of matrix
	MatrixXd upperPart(3, 6);
	upperPart << 0,0,0,0,0,1, 0,0,0,0,1,0, 0,0,0,2,0,0;
	
	// constructing lower part of matrix
	double t5 = pow(time,5);
	double t4 = pow(time,4);
	double t3 = pow(time,3);
	double t2 = pow(time,2);
	double t1 = pow(time,1);
	MatrixXd lowerPart(3, 6);
	lowerPart << 1/120*t5,1/24*t4,1/6*t3,1/2*t2,t1,1, 1/24*t4,1/6*t3,1/2*t2,t1,1,0, 1/6*t3,1/2*t2,t1,1,0,0; 
	
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
MatrixXd constraintsToDers(VectorXd constraints, double t) {
	
	MatrixXd mat = getMatForDersToConstraints(t);			// There exists a matrix M(endTime), satisfying M*ders = constraints.
    VectorXd ders = mat.fullPivLu().solve(constraints);

	return ders;
}


MatrixXd dersToTimeDependenceOfDim(MatrixXd a) {
	
	MatrixXd mat(4, 6);
    mat << 1/120*a(0),1/24*a(1),1/6*a(2),1/2*a(3),a(4),a(5),  0,1/24*a(0),1/6*a(1),1/2*a(2),a(3),a(4),  0,0,1/6*a(0),1/2*a(1),a(2),a(3),  0,0,0,1/2*a(0),a(1),a(2);
	
	return mat
}



/*

The input format should be as follows:
constraints = [[x0; xd0; xdd0; x1; xd1; xdd1], [y0; ... ], [z0; ... ]] 

The output is [[x;xd;xdd; ... xddddd], [y;yd; ...], [z;zd; ...]] at time = 0.

*/
MatrixXd getDersXYZ(MatrixXd constraints, double time) {
	// dersXYZ = [[x;xd;xdd; ... xddddd], [y;yd; ...], [z;zd; ...]]
	
	MatrixXd dersXYZ(6, 3);

    for (int j = 0; j <= 2; j++) {			//iterate over parts(x, y, z) of states
        MatrixXd ders(4,6);
        ders = constraintsToDers(constraints.col(j), time);
        dersXYZ.col(j) = ders;
    }

    return dersXYZ;	
}


/*

The output is a list of states, at times [time/points, ... , time].

*/
MatrixXd dersToPath(MatrixXd ders, VectorXd yaw, int points, double time) {
	// ders = [x;xd;xdd;xddd; y;yd; ... z;zd; ... ]
	// state in path = [x, y, z, Vx, Vy, Vz, phi, theta, psi, p, q, r]
	
	length = len(ders);
	MatrixXd(12, length) path;
	
	path.block<1,length>(0,0) = ders.block<1,length>(0,0);		// set x
	path.block<1,length>(1,0) = ders.block<1,length>(4,0);		// set y
	path.block<1,length>(2,0) = ders.block<1,length>(8,0);		// set z 
	
	path.block<1,length>(3,0) = ders.block<1,length>(1,0);		// set Vx
	path.block<1,length>(4,0) = ders.block<1,length>(5,0);		// set Vy
	path.block<1,length>(5,0) = ders.block<1,length>(9,0);		// set Vz
	
	
	return path;
}



double maxAcceleration(MatrixXd dersXYZ){
	
	tX = 
	tY = 
	tZ = 
	
	accX = 
	accY = 
	accZ = 
	
	maxAcc = sqrt(pow(accX,2) + pow(accY,2) + pow(accZ,2));
	
	return maxAcc
}

/*


*/
MatrixXd getPathSegment(MatrixXd constraints, int points){
	// constraints = [[x0; xd0; xdd0; x1; xd1; xdd1], [y0; ... ], [z0; ... ]] 
	
	MatrixXd dersXYZ(6, 3);
	MatrixXd path(12, points);
	
	int t_min = 0;		
	int t_max = 10;	
	int t;
	double maxAcc = 1;
	
	do{						// make sure t_max yields trajectory with valid thrusts
		t_max = t_max*2;
		dersXYZ = getDersXYZ(constraints, t_max);
	}
	while(maxAcceleration(dersXYZ) >= maxAcc)
			
	while(t_max - t_min > 0.1){
		t = (t_max + t_min)/2;
		dersXYZ = getDersXYZ(constraints, t);
		
		if(maxAcceleration(dersXYZ) <= maxAcc){
			t_max = t;
		} else{
			t_min = t;
		}
	}
		
	dersXYZ = getDersXYZ(constraints, t_max);
	yaw = yaw_math(p_before_hoop(1,1),p_before_hoop(1,2),final(1,1),final(1,2),yaw0,t_max); //need the method
	path = dersToPath(dersXYZ, yaw, points, t_max)
		    
	return path
}


/*

All states must be given in the world frame.

The world frame must satisfy:
The positive direction of the z-axis of the world frame is parrallel to the gravitational accelleration.
The world frame stands still with respect to the earth.

*/
MatrixXd getPath(MatrixXd currentState, MatrixXd stateBeforeHoop, MatrixXd stateAfterHoop) {
	// currentState = [x; y; z; Vx; Vy; Vz; phi; theta; psi; p; q; r]
	// stateBeforeHoop = [[x; y; z], [Vx; Vy; Vz], [xdd; ydd; zdd]]
	// stateAfterHoop = [[x; y; z], [Vx; Vy; Vz], [xdd; ydd; zdd]]
	
	
	// state = [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]] 

    MatrixXd allConstraints(6, 6);
    allConstraints.block<3, 3>(0, 0) = currentState;		// constraints of first trajectory part (currentState -> stateBeforeHoop)
	allConstraints.block<3, 3>(3, 0) = stateBeforeHoop;
    allConstraints.block<3, 3>(0, 3) = stateBeforeHoop;		// constraints of second trajectory part (stateBeforeHoop -> stateAfterHoop)
    allConstraints.block<3, 3>(3, 3) = stateAfterHoop;

    int pointsPerPathPart = 50;
    MatrixXd constraints(6,3);		// constraints = [[x0; xd0; xdd0; x1; xd1; xdd1], [y0; ... ], [z0; ... ]] 
    int waypoints = 1;				// number of input waypoints to this function, excluding the currentState and endState(stateAfterHoop in current implementation).
    double yaw = 0;

    MatrixXd pathPart(12, pointsPerPathPart);
    MatrixXd path(12, (waypoints + 1)*pointsPerPathPart);

    for (int i = 0; i <= waypoints; i++) {							// iterate over path intervals. 
		constraints = allConstraints.block<6,3>(0, i*3) 			// get constraints for this path interval
		pathPart = getPathSegment(constraints, pointsPerPathPart);
        path.block<12, pointsPerPathPart>(0, i*pointsPerPathPart) = pathPart;
    }

    return path;
}



/* 

Let the hoop coordinate frame be as follows:
The x-axis goes through the main symmetry axis of the hoop, positive direction given by the passage direction.
The y-axis is horizontal in the earth-frame, positive direction to the right, from the incoming drone point of view.
The z-axis is as in a righthanded coordinate frame.

The positive direction of the z-axis of the world frame is parrallel to the gravitational accelleration.

All used frames stand still with respect to the earth.

*/
void runPathPlanner(VectorXd currentState, Vector3d hoopTransVec, Matrix3d hoopRotMat, MatrixXd output){
	// currentState = state of drone in world frame.
    // hoopTransVec = hoop position in world frame.
	// hoopRotMat = rotation from hoop frame to world frame.
	
    MatrixXd R = MatrixXd::Zero(3, 3);
    hoopRotMat.copyTo(R);
	Vec3d hoop_pos;
    hoopTransVec.copyTo(hoop_pos);				
	
	
	// state relative to hoop before hoop
    Vector3d dist_in(d_before, 0, 0);				// hoop frame
    Vector3d vel_in(v_in, 0, 0);											
	dist_in = hoop_pos + R*dist_in;					// world frame
    vel_in = R*vel_in;
	
	// set state before hoop
	MatrixXd beforeHoop = Zero(3, 3);
    beforeHoop.col(0) = dist_in;
    beforeHoop.col(1) = vel_in;
	
	
	// state relative to hoop after hoop
    Vector3d dist_out(d_after, 0, 0);				// hoop frame
    Vector3d vel_out(v_after, 0, 0);
    dist_out = hoop_pos + R*dist_out;				// world frame
    vel_out = R*vel_out;
	
	// set state after hoop
	MatrixXd afterHoop = Zero(3, 3);	
    afterHoop.col(0) = dist_out;
    afterHoop.col(1) = vel_out;

	
    output = getPath(currentState, beforeHoop, afterHoop);		// coordinates in world frame.
    
	//cout << "Pathplanner executed succesfully" << endl;
	
	return;
}





// argument count, argument vector
int main(int argc, char* argv[]){
	
	
	MatrixXd path(50, 12);
    runPathPlanner(currentState, hoopTransVec, hoopRotMat, path);

    return 0;
}



















