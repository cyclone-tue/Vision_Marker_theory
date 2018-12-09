#include "DetectMarker.h"
#include <ctime>

VideoCapture cap;
//Videowriter debugStream;
Mat cameraMatrix;
Mat distCoef;
Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
VideoWriter debugWriter;

Mat vision::debugFrame;// = Mat::zeros(cv::Size(1,49), CV_64FC1);

//VectorXd coef_vec2(6);




namespace {
    const char* keys =
            "{cal |    | File to load calibration data from}"
            "{cam | 0  | camera input to use}";


    const Vec3d hoop_offset = Vec3d(0,0.37,0);//Offset variable in world space from center of marker to center of hoop.

    //used for detectboard
    const vector<Point2f> polarCoordinates{
            Point2f((1.057-0.141)/2,M_PI_2),
            Point2f((1.036-0.141)/2,0),
            Point2f((1.057-0.141)/2,-M_PI_2),
            Point2f((1.036-0.141)/2,M_PI)
    };
    const vector<int> boardIds{26,27,25,24};
    const float markerLength = 0.141; // Marker side length in meters
    const Ptr<Board> board = CircleBoard::create(polarCoordinates, markerLength, dictionary, boardIds);

}






bool vision::readCameraParameters(String filename, OutputArray cameraMat, OutputArray distCoefficients){ // not OutputArray
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

    localCamera.copyTo(cameraMat);
    localDist.copyTo(distCoefficients);

    fs.release();
    return true;
}


bool vision::run(Vector3d& hoopTransVec, Matrix3d& hoopRotMat, bool visualize) {        // visualize should be removed in future version.

    bool foundMarker = false;
    Mat image, imageCopy;
    cap >> image;
    //cap.read(image);
    image.copyTo(imageCopy);
    Vector3d translation;
    Matrix3d rotation;

    //resize(imageCopy, imageCopy, Size(64, 36), 0, 0, INTER_CUBIC);

    vector<int> ids;
    vector<vector<Point2f> > corners;
    aruco::detectMarkers(image, dictionary, corners, ids);

    //At least one marker detected
    if (!ids.empty()) {
        //cout << "detected a marker" << endl;
        aruco::drawDetectedMarkers(imageCopy, corners, ids);
        Vec3d rvec, tvec;

        int valid = aruco::estimatePoseBoard(corners, ids, board, cameraMatrix, distCoef, rvec, tvec);

        // if at least one board marker detected
        if(valid > 0) {
            foundMarker = true;

            if (visualize) {
                aruco::drawAxis(imageCopy, cameraMatrix, distCoef, rvec, tvec, 0.1);
            }


            Mat rotMat;
            Rodrigues(rvec, rotMat);            //Calculate rotation matrix for marker
            Mat pos = rotMat.t() * Mat(tvec);   //Calculate marker position in world space

            /* seems somewhat complex, so did not remove it, but could not find a use for it....
            Mat pixelsTranslated = rotMat * pos;
            Vec3d pixels;
            pixelsTranslated.copyTo(pixels);
            tvec = pixels;
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
            */

            cv2eigen(pos, translation);
            cv2eigen(rotMat, rotation);

            hoopTransVec = translation;
            hoopRotMat = rotation;

        }
    }

    imageCopy.copyTo(vision::debugFrame);       // for visualization

    if(debugWriter.isOpened()){
        debugWriter.write(imageCopy);
    }

    return foundMarker;
}





/*
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
                cout << "Printing points from " << j << " to " << (j + 9) << endl;
                for(int i = j; i < j + 10; i++){
                    cout << "x: " << path.at<double>(0,i) << ", y: " << path.at<double>(4,i) << ", z: " << path.at<double>(8,i) << endl;
                }
            }
        }
        char key = (char) waitKey(1);
        if (key == 27) break;
    }

    cleanup();

}*/



/*
double* vision::MatrixToArray(MatrixXd m) {
    static double db_array[100][12];
//    cout << "Path is: " << endl;
//    cout <<  m  << endl;
    Map<MatrixXd>(&db_array[0][0], m.rows(), m.cols()) = m;
    return &db_array[0][0];
}
*/

void vision::setupVariables(int camera, const char* calibrationFile){
    String filename = String(calibrationFile);
    cout << "Opening camera " << camera << endl;

    cap = VideoCapture();
    cap.open(camera);

    //if (!cap.isOpened())  // check if succeeded to connect to the camera
    //    CV_Assert("Cam open failed");

    cap.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);        // I(Arnoud) changed here from CV_CAP_PROP_FRAME_WIDTH
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    //cap.set(CV_CAP_PROP_FPS, 15);
    //cout << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
    //cout << cap.get(cv::CAP_PROP_FRAME_WIDTH) << endl;
    //cout << cap.get(cv::CAP_PROP_FPS) << endl;

    //debugStream.open("appsrc use-damage=false ! videoconvert ! videoscale ! vp8enc ! rtpvp8pay ! udpsink host=localhost port=9999", 0, (double)30, Size(640, 480), true);

    cout << "Reading camera parameters" << endl;
    cout << "Calibration file is" << filename << endl;


    if(!vision::readCameraParameters(filename, cameraMatrix, distCoef)){
        cout << "Could not load camera calibration file: " << calibrationFile << endl;
    }else{
        //cout << cameraMatrix << endl;
        //cout << distCoef << endl;
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

void vision::projectPointsOntoCam(vector<Point3d> cvPoints, vector<Point2d>& imagePoints){
    projectPoints(cvPoints, Vec3d(0, 0, 0), Vec3d(0, 0, 0), cameraMatrix, distCoef, imagePoints);
    return;
}


void vision::cleanup(){
    cap.release();

    debugWriter.release();
}
