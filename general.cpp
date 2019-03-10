//
// Created by arnoud on 5-3-19.
//

#include "general.h"

/*
 * let the input be [Thrust, torqueX, torqueY, torqueZ]
 */
Vector4d torquesToThrusts(Vector4d torques){
    // solving torques = M*thrusts
    double c_q = 2.42e-7;
    double c_t = 8.048e-6;
    double d = 0.12;
    double b = c_q/c_t;
    Matrix4d diag;
    diag << 1,0,0,0, 0,d,0,0, 0,0,d,0, 0,0,0,b;
    Matrix4d signs;
    signs << 1,1,1,1, 1,1,-1,-1, 1,-1,-1,1, -1,1,-1,1;

    Matrix4d M = diag*signs;
    Vector4d thrusts = M.colPivHouseholderQr().solve(torques);
    return thrusts;
}


Matrix3d anglesToRotMatXYZ(double roll, double pitch, double yaw){
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = yawAngle*pitchAngle*rollAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    return rotationMatrix;
}





