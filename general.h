//
// Created by arnoud on 5-3-19.
//

#ifndef MARKER_VISION_GENERAL_H
#define MARKER_VISION_GENERAL_H



#include <Eigen/Dense>
using namespace Eigen;

Vector4d torquesToThrusts(Vector4d torques);
Matrix3d anglesToRotMatXYZ(double roll, double pitch, double yaw);






#endif //MARKER_VISION_GENERAL_H
