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


bool runFrame(VectorXd& currentState, Vector4d& currentTorque, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques);
VectorXd arrayToEigen(double* array, int length);
int main();

extern "C" {
    bool output_to_py(double* output, double* currentStateArray, double* currentTorqueArray, int *pathLength, bool visualize);
    void setup(const char* camera_calibration_file);
    void cleanup();
    double *output_simple_path(double* currentStateArray, double* currentTorqueArray, int* pathLength, bool visualize); //Output a path consisting of 2 points. One before the hoop one after.
}

// in logging.c :

void runVisualize(VectorXd& currentState, MatrixXd& path, bool displayPath);
void showPathInteractive(MatrixXd& path, VectorXd& timeDiffs, Vector3d hoopTransVec, Matrix3d hoopRotMat);


#endif //MARKER_VISION_V_PP_H
