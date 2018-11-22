#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv/cv.h>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
    if (argc < 1) {
        cout << "Please specify camera input as argument." << endl;
        return 0;
    }
    VideoCapture cap(1);
    //cap.set(CV_CAP_PROP_FPS, 25);
    Mat image;
    Mat outImage;
    while (true) {
        cout << "Grabbing image... ";
        cap.read(image);
        cout << image.rows << ", " << image.cols << endl;

        resize(image, outImage, Size(1080, 720));
        cout << "Image retrieved!" << endl;
        //imshow("out", outImage);
        //cap.set(CV_CAP_PROP_FPS, 25);
        cout << "Image shown" << endl;
        //if(waitKey(1) >= 0) break;
    }
    return 0;
}

