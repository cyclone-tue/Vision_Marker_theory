#include "DetectMarker.h"
#include <ctime>

VectorXd coef_vec2(6);
VideoCapture cap;
Mat cameraMatrix, distCoef;
Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
VideoWriter debugWriter;


namespace {
    const char* keys =
            "{cal |    | File to load calibration data from}"
            "{cam | 0  | camera input to use}";
    const double markerSize = 0.141; // Marker side length in meters
    const int markerId = 26;
    const double d_after = 0.5;
    const double v_after = 0.1;
    const double d_before = 0.5;
    const double v_in = 0.1;
    const Vec3d hoop_offset = Vec3d(0,-0.37,0);//Offset variable in world space from center of marker to center of hoop.

}

bool readCameraParameters(String filename, OutputArray cameraMatrix, OutputArray distCoefficients){
    cout << "Opening file: " << filename << endl;
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()){
        fs.release();
        cout << "Could not open file: " << filename << endl;
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
MatrixXd Dimention3(MatrixXd init, MatrixXd p_before_hoop, MatrixXd final) {

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
    Mat init = Mat::zeros(1, 3, CV_64FC1);
    Mat R = Mat::zeros(3, 3, CV_64FC1);
    Mat dist_corr_in = Mat::zeros(3, 1, CV_64FC1);
    Mat vel_corr_in = Mat::zeros(3, 1, CV_64FC1);
    Mat dist_corr_fin = Mat::zeros(3, 1, CV_64FC1);
    Mat vel_corr_fin = Mat::zeros(3, 1, CV_64FC1);
    hoopRotMat.copyTo(R);
    //cout << "Rotation matrix: " << R << endl;
    Mat entryPointData = Mat::zeros(3,2, CV_64FC1);//Two columns. One for position, one for speed.
    Mat exitPointData = Mat::zeros(3,2, CV_64FC1);//Two columns. One for position, one for speed.
    Mat hoop_pos = Mat::zeros(3,2, CV_64FC1);
    //Mat hoop_pos_data = hoop_pos(Rect(Point2f(0,0), Size(1,3)));//Mat::zeros uses row, cols notation. Size uses width, height notation.
    //cout << "Hoop_ps_data: " << hoop_pos_data << endl;
    hoopTransVec.copyTo(hoop_pos.col(0));
    cout<< "HoopPos: " << hoop_pos << endl;

    entryPointData.at<double>(2,0) = d_before;
    entryPointData.at<double>(2,1) = -v_in;
    exitPointData.at<double>(2,0) = -d_after;
    exitPointData.at<double>(2,1) = -v_after;

    cout << "Entrypoint data: " << entryPointData << endl;
    cout << "Exit point data: " << exitPointData << endl;

    Mat entryCorrection = R * entryPointData;
    Mat exitCorrection = R * exitPointData;

    Mat beforeHoop = hoop_pos + entryCorrection;
    beforeHoop = beforeHoop.t();

    Mat afterData = hoop_pos + exitCorrection;
    Mat afterHoop;
    hconcat(afterData, Mat::zeros(3,1, CV_64FC1), afterHoop);
    //Mat afterHoop = Mat::zeros(3,3,CV_64FC1);
    //Mat afterHoop_data = afterHoop(Rect(Point2f(0,0), Size(3,2)));
    //afterData.copyTo(afterHoop_data);
    afterHoop = afterHoop.t();

    //cout << "Initialized pathplanner variables" << endl;

    //cout << "before hoop point: " << beforeHoop << endl;
    //cout << "after hoop point: " << afterData << endl;

    cout << "Init: " << init << endl << "beforeHoop: " << beforeHoop << endl << "afterHoop: " << afterHoop << endl;// <<  "hoop_pos: " << hoop_pos << endl;

    Mat r;
    MatrixXd initEigen, beforeHoopEigen, afterHoopEigen, hoop_posEigen;
    cv2eigen(init, initEigen);
    cv2eigen(beforeHoop, beforeHoopEigen);
    cv2eigen(afterHoop, afterHoopEigen);
    MatrixXd result = Dimention3(initEigen, beforeHoopEigen, afterHoopEigen);
    //cout << "Pathplanner executed succesfully" << endl;
    eigen2cv(result, r);
    r.copyTo(output);
    //cout << "Path: " << r.rows << "x" << r.cols << endl;
}

bool runFrame(bool visualize, OutputArray path) {
    bool foundMarker = false;
    Mat image, imageCopy;
    cap >> image;
    image.copyTo(imageCopy);

    vector<int> ids;
    vector<vector<Point2f> > corners;
    aruco::detectMarkers(image, dictionary, corners, ids);

    //At least one marker detected
    if (!ids.empty()) {
        aruco::drawDetectedMarkers(imageCopy, corners, ids);
        vector<Vec3d> rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoef, rvecs, tvecs);
        for (int i = 0; i < ids.size(); i++) {
            if (ids[i] == markerId) {
                Mat rotMat;

                Rodrigues(rvecs[i], rotMat);//Calculate rotation matrix for marker

                Mat pos = rotMat.t() * Mat(tvecs[i]); //Calculate marker position in world space
                pos = pos + Mat(hoop_offset); //Add offset in world space to get center of hoop.
                //cout << pos << endl;

                Mat pixelsTranslated = rotMat * pos;
                Vec3d pixels;
                pixelsTranslated.copyTo(pixels);
                tvecs[i] = pixels;
                double sy = sqrt(pow(rotMat.at<double>(0, 0), 2) + pow(rotMat.at<double>(1, 0), 2));
                bool singular = sy < 1e-6;
                double rollTemp, pitchTemp, yawTemp;
                if (!singular) {
                    rollTemp = atan2(rotMat.at<double>(2, 1), rotMat.at<double>(2, 2));
                    pitchTemp = atan2(-rotMat.at<double>(2, 0), sy);
                    yawTemp = atan2(rotMat.at<double>(1, 0), rotMat.at<double>(0, 0));
                } else {
                    rollTemp = atan2(rotMat.at<double>(2, 1), rotMat.at<double>(2, 2));
                    pitchTemp = 0;
                    yawTemp = atan2(rotMat.at<double>(1, 0), rotMat.at<double>(0, 0));
                }

                double yaw = -pitchTemp;
                double roll = -yawTemp;
                double pitch = M_PI - rollTemp;
                if (pitch > M_PI) {
                    pitch -= 2 * M_PI;
                }

                double x = pos.at<double>(0, 0);
                double y = pos.at<double>(1, 0);
                double z = pos.at<double>(2, 0);



                //cout << "x " << x << ", y : " << y << ", z :" << z <<  ", yaw: " << yaw/M_PI*180 << ", pitch: " << pitch/M_PI*180 << ", roll: " << roll/M_PI*180 << endl;
                if(visualize){
                    aruco::drawAxis(imageCopy, cameraMatrix, distCoef, rvecs[i], tvecs[i], markerSize);
                }

                Mat pathData;
                Mat input = Mat(tvecs[i]);
                //cout << input << endl;
                runPathPlanner(input, rotMat, pathData);


                if(visualize){
                    vector<Point3d> pathPoints;
                    for (int i = 0; i < pathData.cols; i++) {
                        Mat col = pathData.col(i);
                        pathPoints.push_back(Point3d(col.at<double>(0, 0), col.at<double>(4, 0), col.at<double>(8, 0)));
                    }
                    //cout << "Path points: " << pathPoints << endl;
                    //cout << "Created path points." << endl;
                    vector<Point2d> imagePoints;
                    projectPoints(pathPoints, Vec3d(0, 0, 0), Vec3d(0, 0, 0), cameraMatrix, distCoef, imagePoints);
                    for (int j = 0; j < imagePoints.size() - 1; j++) {
                        //cout << imagePoints[j] << endl;
                        line(imageCopy, imagePoints[j], imagePoints[j + 1], Scalar(int(255 / imagePoints.size() * j), 0,
                                                                                   int(255 / imagePoints.size() *
                                                                                       (imagePoints.size() - j))), 3);
                    }
                }
                pathData.copyTo(path);
                foundMarker = true;
                //cout << "Drawn lines" << endl;
                //cout << path << endl;
            }

        }

    }
    if(visualize){
        imshow("out", imageCopy);
    }
    if(debugWriter.isOpened()){
        debugWriter.write(imageCopy);
    }
    return foundMarker;
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


    setupVariables(cam, filename.c_str());

    while(true){
        Mat path;
        if(runFrame(true, path)){
            for(int j = 0; j < 100; j+=10){
                //cout << "Printing points from " << j << " to " << (j + 9) << endl;
                for(int i = j; i < j + 10; i++){
                    //cout << "x: " << path.at<double>(0,i) << ", y: " << path.at<double>(4,i) << ", z: " << path.at<double>(8,i) << endl;
                }
            }
        }
        char key = (char) waitKey(1);
        if (key == 27) break;
    }

    cleanup();
}

double* MatrixToArray(MatrixXd m) {
    static double db_array[100][12];
//    cout << "Path is: " << endl;
//    cout <<  m  << endl;
    Map<MatrixXd>(&db_array[0][0], m.rows(), m.cols()) = m;
    return &db_array[0][0];
}

double* output_to_py(bool* foundPath, bool visualize){
    static double* db_p;
    Mat path;
    MatrixXd pathEigen;
    *foundPath = runFrame(visualize, path);
    if(visualize) waitKey(1);
    if(*foundPath){
        path = path.t();
    } else{
        path = Mat::zeros(100, 12, CV_64FC1);
    }
    cv2eigen(path, pathEigen);
    db_p = MatrixToArray(pathEigen);
    return db_p;
}
void setupVariables(int camera, const char* calibrationFile){
    String filename = String(calibrationFile);
    cout << "Opening camera" << endl;
    cap = VideoCapture();
    cap.open(camera);
    cout << "Reading camera parameters" << endl;
    cout << "Calibration file is" << filename << endl;
    if(!readCameraParameters(filename, cameraMatrix, distCoef)){
        cout << "Could not load camera calibration file: " << calibrationFile << endl;
    }else{
        cout << cameraMatrix << endl;
        cout << distCoef << endl;
    }
    namedWindow("out", WINDOW_KEEPRATIO);
    resizeWindow("out", 300,300);

    Mat testFrame;
    cap >> testFrame;

    time_t now = time(0);
    tm* localTime = localtime(&now);
    int codec = VideoWriter::fourcc('H','2','6','4');
    ostringstream converter;
    converter << "Field_test-" << localTime->tm_mday << "-" << localTime->tm_mon << "-" << localTime->tm_year << "_" << localTime->tm_hour << ":" << localTime->tm_min << ":" << localTime->tm_sec << ".mp4";
    String outFilename;
    outFilename = converter.str();
    //String outFilename = "Field_test-"  + to_string(localTime->tm_mday) + "-" + to_string(localTime->tm_mon) + "-" + to_string(localTime->tm_year) + "_" + to_string(localTime->tm_hour) + ":" + to_string(localTime->tm_min) + ":" + to_string(localTime->tm_sec) + ".mp4";
    cout << "Writing to video file: " << outFilename.c_str() << endl;
    debugWriter = VideoWriter(outFilename.c_str(), codec, 25.0, Size(testFrame.cols, testFrame.rows), true);
    if(!debugWriter.isOpened()){
        cout << "Could not open video writer!" << endl;
    }
}

void cleanup(){
    cap.release();

    debugWriter.release();
}

