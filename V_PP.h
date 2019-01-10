#ifndef MARKER_VISION_V_PP_H
#define MARKER_VISION_V_PP_H


#include "Vision/vision.h"
#include "Path_Planning/path_planner.h"

#include <iostream>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>

//PATH_PLANNER()
//VISION()

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::DiagonalMatrix;

using namespace std;
using namespace Eigen;



int runFrame(bool visualize, VectorXd currentState, VectorXd currentTorque, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques);
int main();
void runVisualize(VectorXd& currentState, MatrixXd& path, bool displayPath);

extern "C" {
    /*
    The positive direction of the z-axis of the world frame is parallel to the gravitational acceleration.
    All used frames stand still with respect to the earth.

    The arguments are the following:
    currentState    : state of drone in world frame.
    currentTorque   : [thrust, torqueX, torqueY, torqueZ].
    *pathLength     : number of states in the returned path.
                    : initial value is not used.

    return value    : pointer to array of doubles, being the elements([row0,row1, .. row_n]) of the following matrix:
                    : The matrix is a concatenation of the following matrices, each with *pathLength rows.
                    : path      : each row is a state in world space of the path.
                    : timeDiffs : each i'th element is the time difference between the i-1'th and i'th state in path.
                                : the first element is the time difference between the current time and the time of the first state in path.
                    : torques   : each row is the [thrust, torqueX, torqueY, torqueZ] corresponding to the same row in path.
    */
    double *output_to_py(double* currentStateArray, double* currentTorqueArray, int *pathLength, bool visualize);
    void setup(const char* camera_calibration_file);
    void cleanup();
}




#endif //MARKER_VISION_V_PP_H
