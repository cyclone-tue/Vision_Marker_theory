#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;
int main(int argc, char* argv[]){
    VideoCapture cap;
    cap.open(0);

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);

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
        }
        imshow("out", imageCopy);
        char key = (char) waitKey(10);
        if (key == 27) break;

    }

}

