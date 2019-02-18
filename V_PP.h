#ifndef MARKER_VISION_V_PP_H
#define MARKER_VISION_V_PP_H


#include "Vision/vision.h"
#include "Path_Planning/path_planner.h"

#include <iostream>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <fstream>
#include <ctime>


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
    double *output_to_py(double* currentStateArray, double* currentTorqueArray, int *pathLength, bool visualize);
    void setup(const char* camera_calibration_file);
    void cleanup();
}


// in logging.c :
/*void runVisualize(VectorXd& currentState, MatrixXd& path, bool displayPath);
void writeDebug(string info, bool display, string destination = "log");
void writeDebug(MatrixXd info, bool display, string destination = "log");
void writeDebug(string info, string destination = "log", bool display = false);
void writeDebug(MatrixXd info, string destination = "log", bool display = false);*/



#endif //MARKER_VISION_V_PP_H
