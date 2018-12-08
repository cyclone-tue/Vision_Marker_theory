#ifndef MARKER_VISION_PATH_PLANNER_H
#define MARKER_VISION_PATH_PLANNER_H

#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::DiagonalMatrix;

using namespace Eigen;
using namespace std;


class path_planner{
public:
  int getPathSize();
  void init();
  void release();
  static int run(VectorXd currentState, VectorXd currentTorque, Vector3d hoopTransVec, Matrix3d hoopRotMat, MatrixXd path, MatrixXd timeDiffs, MatrixXd torques);
  static void stateToPosDers(VectorXd currentState, Vector4d currentTorque, Vector3d beginX, Vector3d beginY, Vector3d beginZ);
  static void jerkToPath(VectorXd beginState, MatrixXd pos, MatrixXd jerk, double timeInterval, double number_of_points, MatrixXd path, VectorXd timeDiffs, MatrixXd torques);
  static void getEndStatePosDers(VectorXd currentState, Vector3d hoopTransVec, Matrix3d hoopRotMat, Vector3d endX, Vector3d endY, Vector3d endZ);
  static void load_default_data(Vector3d beginState, Vector3d endState);
};
/*
jiroe MatrixXd arrayToEigen(double* array, int rows, int columns);
*/


#endif //MARKER_VISION_PATH_PLANNER_H
