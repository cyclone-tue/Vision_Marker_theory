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

            Point point0 = imagePoints[j];
            Point point1 = imagePoints[j+1];

            if(clipLine(Size(frame.cols, frame.rows), point0, point1)){
                line(frame, point0, point1, Scalar(int(255 / imagePoints.size() * j), 0,
                                                                       int(255 / imagePoints.size() *
                                                                           (imagePoints.size() - j))), 3);
            }

        }

    }

    imshow("out", frame);
    waitKey(1);
    return;
}


void showPathInteractive(MatrixXd& path, VectorXd& timeDiffs, Vector3d hoopTransVec, Matrix3d hoopRotMat){

    double maxTime = 0;
    for(int i = 0; i < path.rows(); i++) {
        maxTime += timeDiffs(i);
    }

    double time = 0;
    std::vector<boost::tuple<double, double, double>> traj;
    std::vector<boost::tuple<double, double, double>> dots;
    for(int i = 0; i < path.rows(); i++) {
        double x = path(i, 0);
        double y = path(i, 1);
        double z = path(i, 2);
        traj.push_back(boost::make_tuple(x, y, z));

        time += timeDiffs(i);
        if(time >= 0){
            time -= maxTime/20;
            dots.push_back(boost::make_tuple(x, y, z));
        }
    }

    double radius = 0.5;
    std::vector<boost::tuple<double, double, double>> hoop;
    for(double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/20){
        // hoop frame
        Vector3d hoopPoint;
        hoopPoint << 0, cos(theta), sin(theta);

        // world frame
        Vector3d hoopPoint_w = hoopRotMat*hoopPoint + hoopTransVec;
        hoop.push_back(boost::make_tuple(hoopPoint_w(0), hoopPoint_w(1), hoopPoint_w(2)));
    }

    gp << "set xlabel 'x'\n";
    gp << "set ylabel 'y'\n";
    gp << "set zlabel 'z'\n";
    gp << "set view equal xyz\n";
    gp << "splot '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 pt 7 ps 1\n";
    gp.send1d(traj);
    gp.send1d(hoop);
    gp.send1d(dots);

    getchar();    // wait for user input (to keep pipe open)
    return;
}




