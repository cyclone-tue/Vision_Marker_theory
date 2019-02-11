#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv/cv.h>
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
    if (argc < 1) {
        cout << "Please specify camera input as argument." << endl;
        return 0;
    }
    VideoCapture cap(1);
    int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cout << "Frame width " << width << endl;
    cout << "Frame height " << height << endl;
    cout << "Aspect ratio " << ((float) width/(float) height) << endl;

    cap.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    //cap.set(CV_CAP_PROP_FPS, 25);
    Mat image;
    Mat outImage;
    while (true) {
        cout << "Grabbing image... ";
        cap.grab();
        cap.retrieve(image);
        cout << image.rows << ", " << image.cols << endl;

        //resize(image, outImage, Size(1080, 720));
        cout << "Image retrieved!" << endl;
        imshow("out", image);
        //cap.set(CV_CAP_PROP_FPS, 25);
        cout << "Image shown" << endl;
        if(waitKey(1) >= 0) break;
        cout << "Returning from while loop" << endl;
    }
    return 0;
}

