// this file contains functions for logging and visualization.

#include "V_PP.h"


/*
path contains x times 12 elements.
 */
void runVisualize(VectorXd& currentState, MatrixXd& path, bool displayPath){

    Mat frame;
    vision::debugFrame.copyTo(frame);

    if(displayPath) {
        MatrixXd points(path.rows(), path.cols());


        vector<Point3d> cvPoints;
        for (int row = 0; row < points.rows(); row++) {
            cvPoints.push_back(Point3d(path(row, 0), path(row, 1), path(row, 2)));
        }

        vector<Point2d> imagePoints;
        vision::projectPointsOntoCam(cvPoints, currentState, imagePoints);


        for (int j = 0; j < imagePoints.size() - 1; j++) {
            line(frame, imagePoints[j], imagePoints[j + 1], Scalar(int(255 / imagePoints.size() * j), 0,
                                                                   int(255 / imagePoints.size() *
                                                                       (imagePoints.size() - j))), 3);
        }

    }

    imshow("out", frame);
    waitKey(1);
    return;
}








