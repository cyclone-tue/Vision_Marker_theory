#ifndef MARKER_VISION_DETECTMARKER_H
#define MARKER_VISION_DETECTMARKER_H

#include <iostream>
#include <opencv/cv.hpp>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;

using namespace cv;
using namespace std;
using namespace Eigen;

bool readCameraParameters(String filename, OutputArray cameraMatrix, OutputArray distCoefficients);
MatrixXd equations_f(MatrixXd M_used, VectorXd cond_vec, int j);
MatrixXd statef(double coef, MatrixXd M_full, double t);
MatrixXd mainm(double iteration, double waypoints, double i, double coef, MatrixXd cond_final, double t);
MatrixXd Dimention3(MatrixXd init, MatrixXd p_before_hoop, MatrixXd final);
void runPathPlanner(InputArray hoopTransVec, InputArray hoopRotMat, OutputArray output);
bool runFrame(bool visualize, OutputArray path);


extern "C" {
    double* output_to_py(bool* foundPath);
    void setupVariables(int camera, const char* calibrationFile);
}
#endif //MARKER_VISION_DETECTMARKER_H
