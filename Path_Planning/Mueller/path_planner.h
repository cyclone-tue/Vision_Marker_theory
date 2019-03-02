#ifndef MARKER_VISION_PATH_PLANNER_H
#define MARKER_VISION_PATH_PLANNER_H

#include <Eigen/Dense>
#include <iostream>
#include "../../constants.h"


//#include "../../V_PP.h"

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
        static void init();
        static void release();


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
        static bool run(VectorXd& currentState, Vector4d& currentTorque, Vector3d& hoopTransVec, Matrix3d& hoopRotMat, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques);
    private:
        static MatrixXd concatenate(MatrixXd& one, MatrixXd& two);
        static VectorXd concatenateVec(VectorXd& one, VectorXd& two);
        static bool getPathSegment(double time, VectorXd& currentState, MatrixXd& constraints, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques);
        static bool getSegment1D(double time, Vector3d& begin, Vector3d& end, VectorXd& pos, VectorXd& vel, VectorXd& acc, VectorXd& jerk);
        static MatrixXd getConstraints(VectorXd& currentState, Vector4d& currentTorque, Vector3d& hoopTransVec, Matrix3d& hoopRotMat);
        static Matrix3d get_ders_hoop_to_world(double dist, double vel, Vector3d& hoopTransVec, Matrix3d& hoopRotMat);
        static Matrix3d stateToPosDers(VectorXd& currentState, Vector4d& currentTorque);
        static bool jerkToPath(double time, VectorXd& beginState, MatrixXd& pos, MatrixXd& vel, MatrixXd& acc, MatrixXd& jerk, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques);
        static bool validTorques(MatrixXd& torques);
        static Vector4d torquesToThrusts(Vector4d torques);
        static void load_data(double time, Vector3d beginState, Vector3d endState);
};

/*
jiroe MatrixXd arrayToEigen(double* array, int rows, int columns);
*/


#endif //MARKER_VISION_PATH_PLANNER_H
