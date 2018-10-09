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

MatrixXd equations_f(InputArray M_used, InputArray cond_vec, int j) {
    Mat M_used_Matrix(6,6, CV_64F);
    M_used.copyTo(M_used_Matrix);
    MatrixXd M_u;
    cv2eigen(M_used_Matrix, M_u);

    Mat cond_vecMatrix(6,1, CV_64F);
    cond_vec.copyTo(cond_vecMatrix);
    MatrixXd cond_v;
    cv2eigen(cond_vecMatrix, cond_v);

    //Map<MatrixXd> cond_v((double *)cond_vecMatrix.data);

    //std::cout << "Here is the matrix A:\n" << M_u << std::endl;
    //std::cout << "Here is the vector b:\n" << cond_v << std::endl;
    VectorXd x = M_u.fullPivLu().solve(cond_v);
    //std::cout << "The solution is:\n" << x << std::endl;


    MatrixXd M_full(4, 6);
    M_full << x(0),x(1),x(2),x(3),x(4),x(5),  0,5*x(0),4*x(1),3*x(2),2*x(3),x(4),  0,0,20*x(0),12*x(1),6*x(2),2*x(3),  0,0,0,60*x(0),24*x(1),6*x(2);
    return M_full;
}

void mainm(double iteration, double waypoints, double i, double coef, InputArray cond_finalArray, double t, OutputArray output) {

    Mat cond_final;
    cond_finalArray.copyTo(cond_final);

    Mat state = Mat::zeros(12,(int) coef, CV_64F);

    Mat cond_vec;
    cond_final.copyTo(cond_vec);
    int last = 1;
    int l1 = 3;
    int l2 = 3;
    //Matrix initial data
    float  data[6*6] = {0,0,0,0,0,1, 0,0,0,0,1,0, 0,0,0,2,0,0, 1,1,1,1,1,1, 5,4,3,2,1,0, 20,12,6,2,0,0};
    Mat m_used = Mat(6,6, CV_64F, &data);

    //m_used << 0,0,0,0,0,1, 0,0,0,0,1,0, 0,0,0,2,0,0, 1,1,1,1,1,1, 5,4,3,2,1,0, 20,12,6,2,0,0;

    Mat coef_vec_val = Mat::zeros(6,3, CV_64FC4);

    for (int j = 1; j <= 3; j++) {
        Mat M_full = Mat(4,6, CV_64F);
        MatrixXd test = equations_f(m_used, cond_vec.col(j-1), j);

        Mat stateField;
        //statef(coef, M_full, t, stateField);
        stateField.copyTo(state(Rect(Point((j-1)*4, 0), Size(4,50))));//Copy stateField to ((j-1)*4,0) with size (4,50)
        Mat coef_vec2 = Mat(6,1, CV_64F);
    }

    state.copyTo(output);
}

void Dimention3(InputArray initArray, InputArray p_before_hoop, InputArray finalArray, InputArray hoop_posArray, OutputArray result) {

    Mat init;
    Mat final;
    Mat hoop_pos;
    Mat beforeHoop;
    initArray.copyTo(init);
    p_before_hoop.copyTo(beforeHoop);
    finalArray.copyTo(final);
    hoop_posArray.copyTo(hoop_pos);

    //the point before the hoop where we still see the hoop
    Mat p33 = Mat::zeros(3,3,CV_64F);
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            p33.at<double>(i,j) = beforeHoop.at<double>(i,j);
        }
    }
    /*p33(2,0) = 0;
    p33(2,1) = 0;
    p33(2,2) = 0;*/

    Mat final_vec = Mat::zeros(6, 6, CV_64F);
    init.copyTo(final_vec(Rect(Point(0,0), init.size())));//Copy init to 0,0
    p33.copyTo(final_vec(Rect(Point(0,3), p33.size())));//Copy p33 to 0,3
    p33.copyTo(final_vec(Rect(Point(3,0), p33.size())));//Copy p33 to 3,0
    final.copyTo(final_vec(Rect(Point(3,0), final.size())));//Copy final to 3,3

    double coef = 50;
    int iteration = 0;
    int t = 0;
    Mat cond_final = Mat::zeros(6,3, CV_64F);
    int counter = 0;
    int t_max = 0;
    int x = 5;
    int T_max = 35;
    int waypoints = 0;
    double yaw = 0;

    Mat state = Mat::zeros(12, (int)coef, CV_64F);
    init(Rect(Point(0,0), Size(3,1))).copyTo(state(Rect(Point(0,49), Size(3,1))));//Copy block of size (3,1) from (0,0) from init to state (0,49) of size (3,1)
    init(Rect(Point(0,1), Size(3,1))).copyTo(state(Rect(Point(4,49), Size(3,1))));//Copy block of size (3,1) from (0,1) from init to state (4,49) of size (3,1)
    init(Rect(Point(0,2), Size(3,1))).copyTo(state(Rect(Point(8,49), Size(3,1))));//Copy block of size (3,1) from (0,2) from init to state (8,49) of size (3,1)

    Mat trajectory = Mat::zeros(12, (int)((waypoints + 2)*coef), CV_64F);

    for (int i = 1; i <= waypoints + 2; i++) {

        if (iteration >= waypoints) {

            state(Rect(Point(0,49), Size(3,1))).copyTo(cond_final(Rect(Point(0,0), Size(3,1))));
            state(Rect(Point(4,49), Size(3,1))).copyTo(cond_final(Rect(Point(0,1), Size(3,1))));
            state(Rect(Point(8,49), Size(3,1))).copyTo(cond_final(Rect(Point(0,2), Size(3,1))));
            final_vec(Rect(Point(3, (3* (i-1))), Size(3,3))).copyTo(cond_final(Rect(Point(3,0), Size(3,3))));

        }
        t = 1;

        mainm(iteration, waypoints, i, coef, cond_final,t, state);
        //        yaw = yaw_math(p_before_hoop(1,1),p_before_hoop(1,2),final(1,1),final(1,2),yaw0,t); need the method


        state.copyTo(trajectory(Rect(Point(0, (int)(iteration * 50)), Size(12,50))));

        iteration++;
    }


    trajectory.copyTo(result);
}

void runPathPlanner(InputArray hoopTransVec, InputArray hoopRotMat, InputOutputArray output, InputArray img){
    Mat init = Mat::zeros(3, 1, CV_64FC1);
    Mat R = Mat::zeros(3, 3, CV_64FC1);
    Mat dist_corr_in = Mat::zeros(3, 1, CV_64FC1);
    Mat vel_corr_in = Mat::zeros(3, 1, CV_64FC1);
    Mat dist_corr_fin = Mat::zeros(3, 1, CV_64FC1);
    Mat vel_corr_fin = Mat::zeros(3, 1, CV_64FC1);

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

    Mat pointBeforeHoop = hoop_pos - dist_corr_in;
    Mat beforeHoop = Mat::zeros(2, 3, CV_64F);
    beforeHoop.col(0) = pointBeforeHoop;
    beforeHoop.col(1) = vel_corr_in;

    Mat pointFinal = Mat::zeros(1,3, CV_64F);
    pointFinal = hoop_pos + dist_corr_fin;
    Mat afterHoop = Mat::zeros(3, 3, CV_64F);
    afterHoop.col(0) = pointFinal;
    afterHoop.col(1) = vel_corr_fin;

    Mat r;
    MatrixXd test;
    cv2eigen(init, test);
    //Dimention3(init, beforeHoop, afterHoop, hoop_pos, r);
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
                    Vec3d offset = Vec3d(0,-0.37,0);

                    Mat rotMat;

                    Rodrigues(rvecs[i], rotMat);

                    Mat camOffset = rotMat*Mat(offset);
                    Vec3d pixelOffset;

                    camOffset.col(0).copyTo(pixelOffset);
                    tvecs[i] += pixelOffset;
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

                    double x = tvecs[i][0];
                    double y = tvecs[i][1];
                    double z = tvecs[i][2];



                    //double dist = rotMatInversed.dot(tvecs[0] * -1);
                    //cout << "Roll: " << rollTemp << endl;
                    //cout << "Pitch: " << pitchTemp << endl;
                    //cout << "Yaw: "  << yawTemp << endl;
                    cout << "x " << x << ", y : " << y << ", z :" << z <<  ", yaw: " << yaw/M_PI*180 << ", pitch: " << pitch/M_PI*180 << ", roll: " << roll/M_PI*180 << endl;
                    aruco::drawAxis(imageCopy, cameraMatrix, distCoef, rvecs[i],tvecs[i], markerSize);
                    Mat path;
                    runPathPlanner(tvecs[i], rotMat, path, imageCopy);
                }

            }

        }
        imshow("out", imageCopy);
        char key = (char) waitKey(1);
        if (key == 27) break;

    }

    cap.release();

}

