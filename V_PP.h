#ifndef MARKER_VISION_V_PP_H
#define MARKER_VISION_V_PP_H

#define _USE_MATH_DEFINES

#include "Vision/vision.h"
#include "Path_Planning/path_planner.h"
#include "general.h"

#include <iostream>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <fstream>
#include <ctime>
#include <memory>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include "logging.h"

//#include "spdlog/sinks/file_sink.h" // support for basic file logging


using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::DiagonalMatrix;

using namespace std;
using namespace Eigen;

typedef Matrix<double, 12, 1> Vector12d;
typedef Matrix<double, Dynamic, 12> MatrixX12;
typedef Matrix<double, Dynamic, 3> MatrixX3;


Trajectory runFrame(Vector12d& currentState, Vector4d& currentTorque, bool visualize);
VectorXd arrayToEigen(double* array, int length);
int main();

extern "C" {
    double *output_to_py(double* currentStateArray, double* currentTorqueArray, int *pathLength, bool visualize);
    void setup(const char* camera_calibration_file);
    void cleanup();
}

// in logging.c :

void runVisualize(Vector12d& currentState, Trajectory traj, Vector3d& hoopTransVec, Matrix3d& hoopRotMat, bool displayPath);


#endif //MARKER_VISION_V_PP_H
