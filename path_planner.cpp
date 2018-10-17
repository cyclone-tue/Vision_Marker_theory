#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <opencv/cv.hpp>			//contains CommandLineParser, aruco
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::DiagonalMatrix;

using namespace cv;
using namespace std;
using namespace Eigen;

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
    const Vec3d hoop_offset = Vec3d(0,0,0);//Offset variable in world space from center of marker to center of hoop.
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
	//time = 1;		// uncomment this line to see result of old version of code.
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

        states.block<4,numOfStates>(j*4, 0) = expansionToStateList(numOfStates, derMat, time);
    }

    return states;	
}



MatrixXd getFastestPath(MatrixXd currentState, MatrixXd stateBeforeHoop, MatrixXd stateAfterHoop, MatrixXd hoop_pos) {

	// state = [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]] 

    MatrixXd allConstraints(6, 6);
    allConstraints.block<3, 3>(0, 0) = currentState;		// constraints of first trajectory part (currentState -> stateBeforeHoop)
	allConstraints.block<3, 3>(3, 0) = stateBeforeHoop;
    allConstraints.block<3, 3>(0, 3) = stateBeforeHoop;		// constraints of second trajectory part (stateBeforeHoop -> stateAfterHoop)
    allConstraints.block<3, 3>(3, 3) = stateAfterHoop;

    double pointsPerTrajPart = 50;
    MatrixXd constraints(6,3);		// constraints = [[x0; xd0; xdd0; x1; xd1; xdd1], [y0; ... ], [z0; ... ]] 
    int waypoints = 1;				// number of input waypoints to this function, excluding the currentState and endState(stateAfterHoop in current implementation).
    double yaw = 0;

    MatrixXd trajPart(12, (int)pointsPerTrajPart);
    MatrixXd trajectory(12, (int)((waypoints + 1)*pointsPerTrajectory));

    for (int i = 0; i <= waypoints; i++) {		// iterate over trajectory intervals. 

		constraints = allConstraints.block<6,3>(0, i*3) 	// get constraints for this trajectory interval
        
		int t_min = 0;
		int t_max = 10;	
		do{						// make sure t_max yields trajectory with valid thrusts
			t_max = t_max*2;
			trajPart = getPath(pointsPerTrajPart, constraints, t_max);
			// yaw = yaw_math(p_before_hoop(1,1),p_before_hoop(1,2),final(1,1),final(1,2),yaw0,t); need the method
		}
		while(!hasValidThrusts(trajPart))
			
		while(t_max - t_min > 0.1){
			t = (t_max + t_min)/2;
			trajPart = getPath(pointsPerTrajPart, constraints, t);
			// yaw = yaw_math(p_before_hoop(1,1),p_before_hoop(1,2),final(1,1),final(1,2),yaw0,t); need the method
			if(hasValidThrusts(trajPart)){
				t_max = t;
			} else{
				t_min = t;
			}
		}
		
		trajPart = getPath(pointsPerTrajPart, constraints, t_max); 
        // yaw = yaw_math(p_before_hoop(1,1),p_before_hoop(1,2),final(1,1),final(1,2),yaw0,t); need the method

        trajectory.block<12, pointsPerTrajPart>(0, (int)(i * pointsPerTrajPart)) = trajPart;
    }

    return trajectory;
}



void runPathPlanner(InputArray hoopTransVec, InputArray hoopRotMat, OutputArray output){
    
	
    Mat R = Mat::zeros(3, 3, CV_64FC1);			// rotation from world(inertial) frame to body frame
    hoopRotMat.copyTo(R);
	Vec3d hoop_pos;								// hoop translation in body frame
    hoopTransVec.copyTo(hoop_pos);		
	
	
	// currentState in body coordinates
	Mat currentState = Mat::zeros(3, 1, CV_64FC1);		
	
	
	// state relative to hoop before hoop
    Mat dist_in_worldframe = Mat::zeros(3, 1, CV_64FC1);		// world(inertial) frame	
    Mat vel_in_worldframe = Mat::zeros(3, 1, CV_64FC1);
	dist_in_worldframe.at<double>(1,0) = d_before;				
    vel_in_worldframe.at<double>(1,0) = v_in;
	
	Mat dist_in = Mat::zeros(3, 1, CV_64FC1);					// body frame
	Mat vel_in = Mat::zeros(3, 1, CV_64FC1);
	dist_in = R * dist_in_worldframe;
    vel_in = R * vel_in_worldframe;
	
	// set state before hoop
	Mat beforeHoop = Mat::zeros(2, 3, CV_64F);
    beforeHoop.col(0) = hoop_pos + dist_in;
    beforeHoop.col(1) = vel_in;
	
	
	
	// state relative to hoop after hoop
    Mat dist_out_worldframe = Mat::zeros(3, 1, CV_64FC1);		// world(inertial) frame
    Mat vel_out_worldframe = Mat::zeros(3, 1, CV_64FC1);
	dist_out_worldframe.at<double>(1,0) = d_after;
    vel_out_worldframe.at<double>(1,0) = v_after;
	
	Mat dist_out = Mat::zeros(3, 1, CV_64FC1);					// body frame
	Mat vel_out = Mat::zeros(3, 1, CV_64FC1);
    dist_out = R * dist_out_worldframe;
    vel_out = R * vel_out_worldframe;
	
	// set state after hoop
	Mat afterHoop = Mat::zeros(3, 3, CV_64F);	
    afterHoop.col(0) = hoop_pos - dist_out;
    afterHoop.col(1) = vel_out;
	

    //cout << "Initialized pathplanner variables" << endl;

    
	
    MatrixXd currentStateEigen, beforeHoopEigen, afterHoopEigen, hoop_posEigen;			// transform cv matrices to MatrixXd
    cv2eigen(currentState, currentStateEigen);
    cv2eigen(beforeHoop, beforeHoopEigen);
    cv2eigen(afterHoop, afterHoopEigen);
    cv2eigen(hoop_pos, hoop_posEigen);
	
    MatrixXd path = getFastestPath(currentStateEigen, beforeHoopEigen, afterHoopEigen, hoop_posEigen);		// coordinates in camera frame
    
	Mat r;
    eigen2cv(path, r);
    r.copyTo(output);

	//cout << "Pathplanner executed succesfully" << endl;
}





// argument count, argument vector
int main(int argc, char* argv[]){
	
	//parser stuff
    CommandLineParser parser = CommandLineParser(argc, argv, keys);
    //parser.about("Run marker detection code.");
    if(argc < 2){					//check number of arguments
        parser.printMessage();		//print some standard information of parser (its arguments)
        return 0;
    }
	
	
	VideoCapture cap = getCameraVar(parser)

    
    Mat cameraMatrix, distCoef;		// some matrices , distortion coefficients
	String filename = parser.get<String>("cal");	//filename for calibration values
	getCameraParameters(filename, cameraMatrix, distCoef); 
	
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);		//make dictionary for what??

	
	
	
	
	

    while(cap.grab()){
        Mat image, imageCopy;
		
        cap.retrieve(image);
        //image.copyTo(imageCopy);	//why the hell

        vector<int> ids;						// ID's of detected markers
        vector<vector<Point2f>> corners;		// corners of markers
        aruco::detectMarkers(image, dictionary, corners, ids);		//write to corners, ids

        //At least one marker detected
        if(!ids.empty()){
            aruco::drawDetectedMarkers(image, corners, ids);		//draw marker
            vector<Vec3d> rvecs, tvecs;		// rotation and translation of marker in body frame
            aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoef, rvecs, tvecs);
			
            for(int i=0; i<ids.size(); i++){
                if(ids[i] == markerId){
                    
					Mat rotMat;								// body to world frame
                    Rodrigues(rvecs[i], rotMat);			// Calculate rotation matrix for marker

                    Mat pos = rotMat.t()*Mat(tvecs[i]); 	// Calculate marker position in world space
                    pos = pos + Mat(hoop_offset); 			// Add offset in world space to get center of hoop.
                    

                    Mat pixelsTranslated = rotMat * pos;	// what does this do? you rotate the marker position in world space with rotMat?
                    Vec3d pixels;
                    pixelsTranslated.copyTo(pixels);
                    tvecs[i] = pixels;
					
                    double sy = sqrt(pow(rotMat.at<double>(0,0), 2) + pow(rotMat.at<double>(1,0), 2));		//what is sy? sin(theta)?
                    bool singular = sy < 1e-6;
					
                    double rollTemp, pitchTemp, yawTemp;		//temp is what
                    if(!singular){
                        rollTemp = atan2(rotMat.at<double>(2,1), rotMat.at<double>(2,2));
                        pitchTemp = atan2(-rotMat.at<double>(2,0), sy);
                        yawTemp = atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
                    }else{
                        rollTemp = atan2(rotMat.at<double>(2,1), rotMat.at<double>(2,2));
                        pitchTemp = 0;
                        yawTemp = atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
                    }

                    double yaw = -pitchTemp;		//thats not nice
                    double roll = -yawTemp;
                    double pitch = M_PI - rollTemp;
                    if(pitch > M_PI){
                        pitch -= 2*M_PI;
                    }

                    double x = pos.at<double>(0,0);
                    double y = pos.at<double>(1,0);
                    double z = pos.at<double>(2,0);



                    //cout << "x " << x << ", y : " << y << ", z :" << z <<  ", yaw: " << yaw/M_PI*180 << ", pitch: " << pitch/M_PI*180 << ", roll: " << roll/M_PI*180 << endl;
                    aruco::drawAxis(imageCopy, cameraMatrix, distCoef, rvecs[i],tvecs[i], markerSize);
                    Mat path;
                    runPathPlanner(pos, rotMat, path);

                    vector<Point3d> pathPoints;
                    for(int i = 0; i < path.rows; i+=10){
                        Mat row = path.row(i);
                        pathPoints.push_back(Point3d(row.at<double>(0,0), row.at<double>(1,0), row.at<double>(2,0)));
                    }
                    //cout << "Created path points." << endl;
                    vector<Point2d> imagePoints;
                    projectPoints(pathPoints, Vec3d(0,0,0), Vec3d(0,0,0), cameraMatrix, distCoef, imagePoints);
                    for (int j = 0; j < imagePoints.size() - 1; j++) {
                        line(imageCopy, imagePoints[j], imagePoints[j + 1], Scalar(255/imagePoints.size() * j,0,255/imagePoints.size()*(imagePoints.size() - j)), 3);
                    }
                    //cout << "Drawn lines" << endl;
                    //cout << path << endl;
                }

            }

        }
        imshow("out", imageCopy);
        char key = (char) waitKey(1);
        if (key == 27) break;

    }

    cap.release();

}



















