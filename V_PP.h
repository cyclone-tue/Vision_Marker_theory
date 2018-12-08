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

const double g = 9.81;




int runFrame(bool visualize, VectorXd currentState, VectorXd currentTorque, MatrixXd path, VectorXd timeDiffs, MatrixXd torques);
int main();

extern "C" {
    double *output_to_py(VectorXd currentState, VectorXd currentTorque, int *pathLength, bool visualize);
    void setup();
}




#endif //MARKER_VISION_V_PP_H
