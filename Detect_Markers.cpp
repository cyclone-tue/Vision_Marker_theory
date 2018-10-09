#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <opencv/cv.hpp>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;

using namespace cv;
using namespace std;
using namespace Eigen;

VectorXd coef_vec2(6);

namespace {
    const char* keys =
            "{cal |    | File to load calibration data from}"
            "{cam | 0  | camera input to use}";
    const double markerSize = 0.07; // Marker side length in meters
    const int markerId = 23;
    const double d_after = 1;
    const double v_after = 1;
    const double d_before = 2;
    const double v_in = 1;
    const Vec3d hoop_offset = Vec3d(0,0,0);//Offset variable in world space from center of marker to center of hoop.

}

static bool readCameraParameters(String filename, OutputArray cameraMatrix, OutputArray distCoefficients){
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()){
        fs.release();
        return false;
    }

    Mat localCamera, localDist;
    fs["camera_matrix"] >> localCamera;
    fs["distortion_coefficients"] >> localDist;

    localCamera.copyTo(cameraMatrix);
    localDist.copyTo(distCoefficients);

    //cout << cameraMatrix << endl;
    //cout << distCoefficients << endl;
    fs.release();
    return true;
}

MatrixXd equations_f(MatrixXd M_used, VectorXd cond_vec, int j) {
    MatrixXd M_u(6, 6);
    M_u = M_used;
    VectorXd cond_v(6);
    cond_v = cond_vec;

    //std::cout << "Here is the matrix A:\n" << M_u << std::endl;
    //std::cout << "Here is the vector b:\n" << cond_v << std::endl;
    VectorXd x = M_u.fullPivLu().solve(cond_v);
    //std::cout << "The solution is:\n" << x << std::endl;

    coef_vec2 = x;
    MatrixXd M_full(4, 6);
    M_full << x(0),x(1),x(2),x(3),x(4),x(5),  0,5*x(0),4*x(1),3*x(2),2*x(3),x(4),  0,0,20*x(0),12*x(1),6*x(2),2*x(3),  0,0,0,60*x(0),24*x(1),6*x(2);
    return M_full;
}



MatrixXd statef(double coef, MatrixXd M_full, double t) {

    MatrixXd statef(4, 50);

    MatrixXd M_fullM(4, 6);
    M_fullM = M_full;
    //std::cout << M_fullM << std::endl;

    coef = coef / t;

    for (double i = 1.0 / coef; (int)round(i*coef) <= 50; i += (1.0 / coef)) {
        VectorXd m(6);
        m << pow(i,5),pow(i,4),pow(i,3),pow(i,2),pow(i,1),1;
        //std::cout << m << std::endl;


        int n = (int) round(i*coef);
        //std::cout << n << std::endl;
        MatrixXd result(4, 1);
        result = M_fullM * m;
        //std::cout << result << std::endl;

        //populate the nth column with the result of the matrix mult of M_full and t_instant
        for (int j = 0; j < 4; ++j) {
            statef(j,n-1) = result(j);
        }
    }
    return statef;
}

MatrixXd mainm(double iteration, double waypoints, double i, double coef, MatrixXd cond_final, double t) {

    MatrixXd state(12,(int) coef);

    MatrixXd cond_vec = cond_final;
    int last = 1;
    int l1 = 3;
    int l2 = 3;
    MatrixXd m_used(6,6);
    m_used << 0,0,0,0,0,1, 0,0,0,0,1,0, 0,0,0,2,0,0, 1,1,1,1,1,1, 5,4,3,2,1,0, 20,12,6,2,0,0;

    MatrixXd coef_vec_val(6,3);

    for (int j = 1; j <= 3; j++) {
        MatrixXd M_full(4,6);
        M_full = equations_f(m_used, cond_vec.col(j-1), j);

        state.block<4,50>((j-1)*4,0) = statef(coef,M_full,t);
        coef_vec_val.block<6,1>(0,j-1) = coef_vec2;
    }

    return state;
}


// This method is for path planning
MatrixXd Dimention3(MatrixXd init, MatrixXd p_before_hoop, MatrixXd final, MatrixXd hoop_pos) {

    //the point before the hoop where we still see the hoop
    MatrixXd p33(3,3);
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            p33(i,j) = p_before_hoop(i,j);
        }
    }
    p33(2,0) = 0;
    p33(2,1) = 0;
    p33(2,2) = 0;

    MatrixXd final_vec(6, 6);
    final_vec.block<3, 3>(0, 0) = init;
    final_vec.block<3, 3>(0, 3) = p33;
    final_vec.block<3, 3>(3, 0) = p33;
    final_vec.block<3, 3>(3, 3) = final;

    double coef = 50;
    int iteration = 0;
    int t = 0;
    MatrixXd cond_final(6,3);
    int counter = 0;
    int t_max = 0;
    int x = 5;
    int T_max = 35;
    int waypoints = 0;
    double yaw = 0;

    MatrixXd state(12, (int)coef);
    state.block<3, 1>(0, 49) = init.block<3, 1>(0, 0);
    state.block<3, 1>(4, 49) = init.block<3, 1>(0, 1);
    state.block<3, 1>(8, 49) = init.block<3, 1>(0, 2);

    MatrixXd trajectory(12, (int)((waypoints + 2)*coef));

    for (int i = 1; i <= waypoints + 2; i++) {

        if (iteration >= waypoints) {

            cond_final.block<3, 1>(0, 0) = state.block<3, 1>(0, 49);
            cond_final.block<3, 1>(0, 1) = state.block<3, 1>(4, 49);
            cond_final.block<3, 1>(0, 2) = state.block<3, 1>(8, 49);
            cond_final.block<3, 3>(3, 0) = final_vec.block<3, 3>(3, (3 * (i - 1)));

        }
        t = 1;

        state = mainm(iteration, waypoints, i, coef, cond_final,t);
        //        yaw = yaw_math(p_before_hoop(1,1),p_before_hoop(1,2),final(1,1),final(1,2),yaw0,t); need the method


        trajectory.block<12, 50>(0, (int)(iteration * 50)) = state;

        iteration++;
    }



    return trajectory;
}



void runPathPlanner(InputArray hoopTransVec, InputArray hoopRotMat, OutputArray output){
    Mat init = Mat::zeros(3, 1, CV_64FC1);
    Mat R = Mat::zeros(3, 3, CV_64FC1);
    Mat dist_corr_in = Mat::zeros(3, 1, CV_64FC1);
    Mat vel_corr_in = Mat::zeros(3, 1, CV_64FC1);
    Mat dist_corr_fin = Mat::zeros(3, 1, CV_64FC1);
    Mat vel_corr_fin = Mat::zeros(3, 1, CV_64FC1);
    hoopRotMat.copyTo(R);

    Mat distanceBeforeHoop = Mat::zeros(3, 1, CV_64FC1);
    Mat velocityBeforeHoop = Mat::zeros(3, 1, CV_64FC1);
    Mat distanceAfterHoop = Mat::zeros(3, 1, CV_64FC1);
    Mat velocityAfterHoop = Mat::zeros(3, 1, CV_64FC1);
    Vec3d hoop_pos;
    hoopTransVec.copyTo(hoop_pos);

    distanceAfterHoop.at<double>(1,0) = d_after;
    velocityAfterHoop.at<double>(1,0) = v_after;
    distanceBeforeHoop.at<double>(1,0) = d_before;
    velocityBeforeHoop.at<double>(1,0) = v_in;

    dist_corr_in = R * distanceBeforeHoop;
    dist_corr_fin = R * distanceAfterHoop;
    vel_corr_in = R * velocityBeforeHoop;
    vel_corr_fin = R * velocityAfterHoop;

    Mat pointBeforeHoop = hoop_pos + dist_corr_in;
    Mat beforeHoop = Mat::zeros(2, 3, CV_64F);
    beforeHoop.col(0) = pointBeforeHoop;
    beforeHoop.col(1) = vel_corr_in;

    Mat pointFinal = Mat::zeros(1,3, CV_64F);
    pointFinal = hoop_pos - dist_corr_fin;
    Mat afterHoop = Mat::zeros(3, 3, CV_64F);
    afterHoop.col(0) = pointFinal;
    afterHoop.col(1) = vel_corr_fin;

    //cout << "Initialized pathplanner variables" << endl;

    Mat r;
    MatrixXd initEigen, beforeHoopEigen, afterHoopEigen, hoop_posEigen;
    cv2eigen(init, initEigen);
    cv2eigen(beforeHoop, beforeHoopEigen);
    cv2eigen(afterHoop, afterHoopEigen);
    cv2eigen(hoop_pos, hoop_posEigen);
    MatrixXd result = Dimention3(initEigen, beforeHoopEigen, afterHoopEigen, hoop_posEigen);
    //cout << "Pathplanner executed succesfully" << endl;
    eigen2cv(result, r);
    r.copyTo(output);

}



int main(int argc, char* argv[]){
    CommandLineParser parser = CommandLineParser(argc, argv, keys);
    parser.about("Run marker detection code.");

    if(argc < 2){
        parser.printMessage();
        return 0;
    }

    int cam = parser.get<int>("cam");
    String filename = parser.get<String>("cal");
    VideoCapture cap;
    cap.open(cam);

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);

    Mat cameraMatrix, distCoef;

    if(!readCameraParameters(filename, cameraMatrix, distCoef)){
        cout << "Could not load camera calibration file: " << filename << endl;
    }else{
        cout << cameraMatrix << endl;
        cout << distCoef << endl;
    }

    while(cap.grab()){
        Mat image, imageCopy;
        cap.retrieve(image);
        image.copyTo(imageCopy);

        vector<int> ids;
        vector<vector<Point2f>> corners;
        aruco::detectMarkers(image, dictionary, corners, ids);

        //At least one marker detected
        if(!ids.empty()){
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            vector<Vec3d> rvecs, tvecs;
            aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoef, rvecs, tvecs);
            for(int i=0; i<ids.size(); i++){
                if(ids[i] == markerId){
                    Mat rotMat;

                    Rodrigues(rvecs[i], rotMat);//Calculate rotation matrix for marker

                    Mat pos = rotMat.t()*Mat(tvecs[i]); //Calculate marker position in world space
                    pos = pos + Mat(hoop_offset); //Add offset in world space to get center of hoop.
                    //cout << pos <<endl;

                    Mat pixelsTranslated = rotMat * pos;
                    Vec3d pixels;
                    pixelsTranslated.copyTo(pixels);
                    tvecs[i] = pixels;
                    double sy = sqrt(pow(rotMat.at<double>(0,0),2) + pow(rotMat.at<double>(1,0), 2));
                    bool singular = sy < 1e-6;
                    double rollTemp, pitchTemp, yawTemp;
                    if(!singular){
                        rollTemp = atan2(rotMat.at<double>(2,1), rotMat.at<double>(2,2));
                        pitchTemp = atan2(-rotMat.at<double>(2,0), sy);
                        yawTemp = atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
                    }else{
                        rollTemp = atan2(rotMat.at<double>(2,1), rotMat.at<double>(2,2));
                        pitchTemp = 0;
                        yawTemp = atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
                    }

                    double yaw = -pitchTemp;
                    double roll = -yawTemp;
                    double pitch = M_PI - rollTemp;
                    if(pitch > M_PI){
                        pitch -= 2*M_PI;
                    }

                    double x = pos.at<double>(0,0);
                    double y = pos.at<double>(1,0);
                    double z = pos.at<double>(2,0);



                    //cout << "x " << x << ", y : " << y << ", z :" << z <<  ", yaw: " << yaw/M_PI*180 << ", pitch: " << pitch/M_PI*180 << ", roll: " << roll/M_PI*180 << endl;
                    aruco::drawAxis(imageCopy, cameraMatrix, distCoef, rvecs[i],tvecs[i], markerSize);
                    Mat path;
                    runPathPlanner(pos, rotMat, path);

                    vector<Point3d> pathPoints;
                    for(int i = 0; i < path.rows; i+=10){
                        Mat row = path.row(i);
                        pathPoints.push_back(Point3d(row.at<double>(0,0), row.at<double>(1,0), row.at<double>(2,0)));
                    }
                    //cout << "Created path points." << endl;
                    vector<Point2d> imagePoints;
                    projectPoints(pathPoints, Vec3d(0,0,0), Vec3d(0,0,0), cameraMatrix, distCoef, imagePoints);
                    for (int j = 0; j < imagePoints.size() - 1; j++) {
                        line(imageCopy, imagePoints[j], imagePoints[j + 1], Scalar(255/imagePoints.size() * j,0,255/imagePoints.size()*(imagePoints.size() - j)), 3);
                    }
                    //cout << "Drawn lines" << endl;
                    //cout << path << endl;
                }

            }

        }
        imshow("out", imageCopy);
        char key = (char) waitKey(1);
        if (key == 27) break;

    }

    cap.release();

}

