#ifndef MARKER_VISION_PATH_PLANNER_H
#define MARKER_VISION_PATH_PLANNER_H

#include <Eigen/Dense>
#include <iostream>
#include "../../constants.h"

#include "../../V_PP.h"

extern "C" {
    #include "cvxgen/solver.h"
}

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::DiagonalMatrix;

using namespace Eigen;
using namespace std;




/*

a state of the drone is [x,y,z, Vx,Vy,Vz, roll,pitch,yaw, p,q,r].


*/

class path_planner{
    public:
        int getPathSize();
        void init();
        void release();

        /*
        Let the hoop coordinate frame be as follows:
        The x-axis goes through the main symmetry axis of the hoop, positive direction given by the passage direction.
        The y-axis is horizontal in the earth-frame, positive direction to the right, from the incoming drone point of view.
        The z-axis is as in a righthanded coordinate frame.

        The positive direction of the z-axis of the world frame is parallel to the gravitational acceleration.
        All used frames stand still with respect to the earth.

        The arguments are the following:
        currentState    : state of drone in world frame.
        currentTorque   : [thrust, torqueX, torqueY, torqueZ].
        hoopTransVec    : hoop position in world frame.
        hoopRotMat      : rotation which transforms from hoop frame coordinates to world frame coordinates.
        &path           : each row is a state of the path.
        &timeDiffs      : each i'th element is the time difference between the i-1'th and i'th state in &path.
                        : the first element is the time difference between the current time and the time of the first state in &path.
        &torques        : each row is the [thrust, torqueX, torqueY, torqueZ] corresponding to the same row in &path.

        return value    : number of states in &path.
        */
        static int run(VectorXd currentState, VectorXd currentTorque, Vector3d hoopTransVec, Matrix3d hoopRotMat, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques);
    private:
        static void jerkToPath(VectorXd beginState, MatrixXd pos, MatrixXd jerk, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques);
        static void getEndStatePosDers(VectorXd currentState, Vector3d hoopTransVec, Matrix3d hoopRotMat, Vector3d& endX, Vector3d& endY, Vector3d& endZ);
        static void load_default_data(Vector3d beginState, Vector3d endState);
        static MatrixXd getConstraints(VectorXd currentState, VectorXd currentTorque, Vector3d hoopTransVec, Matrix3d hoopRotMat);
        static Matrix3d get_before_ders(Vector3d hoopTransVec, Matrix3d hoopRotMat);
        static Matrix3d get_after_ders(Vector3d hoopTransVec, Matrix3d hoopRotMat);
        static Matrix3d stateToPosDers(VectorXd currentState, Vector4d currentTorque);
        static void getPathSegment(VectorXd currentState, MatrixXd constraints, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques);
        static MatrixXd concatenate(MatrixXd& one, MatrixXd& two);
        static VectorXd concatenateVec(VectorXd& one, VectorXd& two);
    };
/*
jiroe MatrixXd arrayToEigen(double* array, int rows, int columns);
*/


#endif //MARKER_VISION_PATH_PLANNER_H
