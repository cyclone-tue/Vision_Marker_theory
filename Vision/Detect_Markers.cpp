#include "DetectMarker.h"

VideoCapture cap;
Mat cameraMatrix;
Mat distCoef;
Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);


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


bool vision::run(Vector3d *hoopTransVec, Matrix3d *hoopRotMat) {

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
        cout << "detected a marker" << endl;
        aruco::drawDetectedMarkers(imageCopy, corners, ids);
        Vec3d rvec, tvec;

        int valid = aruco::estimatePoseBoard(corners, ids, board, cameraMatrix, distCoef, rvec, tvec);

        if(valid > 0){
            foundMarker = true;

            aruco::drawAxis(imageCopy, cameraMatrix, distCoef, rvec, tvec, 0.1);
        }

        Mat rotMat;
        Rodrigues(rvec, rotMat);//Calculate rotation matrix for marker

        Mat pos = rotMat.t() * Mat(tvec); //Calculate marker position in world space


        cv2eigen(pos, translation);
        cv2eigen(rotMat, rotation);

        *hoopTransVec << translation;
        *hoopRotMat << rotation;
    }

    cout << "hoopTransVec0:" << endl;
    cout << translation << endl;

    imshow("out", imageCopy);
    waitKey(1);

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
        runFrame(true, path);
        char key = (char) waitKey(1);
        if (key == 27) break;
    }

    cap.release();

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
    cout << "Opening camera" << endl;
    //cap.set(cv::CAP_PROP_FRAME_WIDTH,4);
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT,8);

    cap = VideoCapture();

    //cap.set(cv::CAP_PROP_FPS, 1);;
    cap.open(camera);
    if (!cap.isOpened())  // check if succeeded to connect to the camera
        CV_Assert("Cam open failed");
    cout << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
    cout << cap.get(cv::CAP_PROP_FPS) << endl;
    //cap.set(CV_CAP_PROP_FRAME_WIDTH,150);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT,320);
    //cap.set(CV_CAP_PROP_FPS, 15);
    cout << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
    cout << cap.get(cv::CAP_PROP_FRAME_WIDTH) << endl;
    cout << cap.get(cv::CAP_PROP_FPS) << endl;
    //cap.set(cv::CAP_PROP_FPS, 1);

    cout << "Reading camera parameters" << endl;
    cout << "Calibration file is" << filename << endl;


    if(!vision::readCameraParameters(filename, cameraMatrix, distCoef)){
        cout << "Could not load camera calibration file: " << calibrationFile << endl;
    }else{
        cout << cameraMatrix << endl;
        cout << distCoef << endl;
    } // modules/highgui/doc/reading_and_writing_images_and_video.html#videocapture-set


    namedWindow("out", WINDOW_KEEPRATIO);
    resizeWindow("out", 300,300);
}

