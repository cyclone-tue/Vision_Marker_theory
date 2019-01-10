#ifndef MARKER_VISION_PATH_PLANNER_H
#define MARKER_VISION_PATH_PLANNER_H


#include <iostream>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include "../../constants.h"

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::DiagonalMatrix;

using namespace std;
using namespace Eigen;


/*

a state of the drone is [x,y,z, Vx,Vy,Vz, roll,pitch,yaw, p,q,r].


*/

class path_planner{
    public:
        // in path_planner.cpp:
        static int run(VectorXd currentState, VectorXd currentTorque, Vector3d hoopTransVec, Matrix3d hoopRotMat, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques);


    private:
        // in path_planner.cpp:
        static void getPathSegment(MatrixXd& constraints, double startYaw, Vector3d& pos_to_look_at, bool look_at_pos, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques);
        static int getPath(MatrixXd currentState, MatrixXd stateBeforeHoop, MatrixXd stateAfterHoop, double currentYaw, Vector3d hoop_pos, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques);
        static Matrix3d getCurrentDers(VectorXd currentState, VectorXd currentTorque);
        static Matrix3d getBeforeHoop(Vector3d hoop_pos, Matrix3d R);
        static Matrix3d getAfterHoop(Vector3d hoop_pos, Matrix3d R);

        // in gettingPosDers.cpp:
        static MatrixXd getMatForDersToConstraints(double time);
        static VectorXd constraintsToDers(VectorXd constraints, double t);
        static MatrixXd dersToTimeDependenceOfDim(MatrixXd a);
        static MatrixXd getDersXYZ(MatrixXd constraints, double time);
        static double maxJerk(MatrixXd dersXYZ, double time);
        static double getPathDers(MatrixXd& constraints, MatrixXd& dersXYZ);

        // in posToStatePath.cpp:
        static void dersToPath(MatrixXd ders, double endTime, double startYaw, Vector3d pos_to_look_at, bool look_at_pos, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques);
        static VectorXd dersXYZtoYaw(MatrixXd dersXYZ);
        static double getYawToHoop(Vector3d pos, Vector3d pos_to_look_at);

        // in math_tools.cpp:
        static double taylor(VectorXd& ders, double time);
        static int factorial(int n);
    };


#endif //MARKER_VISION_PATH_PLANNER_H

