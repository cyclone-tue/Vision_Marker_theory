// this file contains functions for logging and visualization.

#include "V_PP.h"
#include "general.h"
#include <chrono>
#include <thread>

/*
path contains x times 12 elements.
 */
void runVisualize(VectorXd& currentState, MatrixXd& path, VectorXd& timeDiffs, Vector3d& hoopTransVec, Matrix3d& hoopRotMat, bool displayPath){

    Mat frame;
    vision::debugFrame.copyTo(frame);

    if(displayPath) {
        MatrixXd points(path.rows(), 3);

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

    showPathInteractive(path, timeDiffs, hoopTransVec, hoopRotMat);

    return;
}


void showPathInteractive(MatrixXd& path, VectorXd& timeDiffs, Vector3d hoopTransVec, Matrix3d hoopRotMat){

    vpp_logger->flush();
    pp_logger->flush();
    v_logger->flush();

    double min_x = path.col(0).minCoeff();
    double max_x = path.col(0).maxCoeff();
    double min_y = path.col(1).minCoeff();
    double max_y = path.col(1).maxCoeff();
    double min_z = path.col(2).minCoeff();
    double max_z = path.col(2).maxCoeff();
    double maxTime = timeDiffs.sum();

    double time = 0;            // generate trajectory
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

    double radius = 0.5;        // generate hoop
    std::vector<boost::tuple<double, double, double>> hoop;
    for(double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/20){
        // hoop frame
        Vector3d hoopPoint;
        hoopPoint << 0, 0.5*cos(theta), 0.5*sin(theta);

        // world frame
        Vector3d hoopPoint_w = hoopRotMat*hoopPoint + hoopTransVec;
        hoop.push_back(boost::make_tuple(hoopPoint_w(0), hoopPoint_w(1), hoopPoint_w(2)));
    }
    gp << "set term x11 0\n";
    gp << "set xlabel 'x'\n";
    gp << "set ylabel 'y'\n";
    gp << "set zlabel 'z'\n";
    gp << "set view equal xyz\n";
    gp << "splot '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 pt 7 ps 1\n";
    gp.send1d(traj);
    gp.send1d(hoop);
    gp.send1d(dots);

    gp << "set view 240,110\n";

    gp << "set term x11 1\n";


    gp << "set xlabel 'x'\n";
    gp << "set ylabel 'y'\n";
    gp << "set zlabel 'z'\n";
    //gp << "set view azimuth 180\n";
    //gp << "set view -90,0\n";
    gp << "set xrange [" << min_x - 1 << ":" << max_x + 1 << "]\n";
    gp << "set yrange [" << min_y - 1 << ":" << max_y + 1 << "]\n";
    gp << "set zrange [" << max_z + 1 << ":" << min_z - 1 << "]\n";


    gp << "set view equal xyz\n";

    while(true) {
        for (int i = 0; i < path.rows(); i++) {
            double dt = timeDiffs(i);

            std::vector<std::vector<boost::tuple<double, double, double>>> drone;
            Vector3d pos = path.block<1, 3>(i, 0);
            Vector3d ang = path.block<1, 3>(i, 6);
            drone = getDronePoints(pos, ang);
            gp
                    << "splot '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines \n";
            gp.send1d(traj);
            gp.send1d(hoop);
            for (int i = 0; i < static_cast<int>(drone.size()); i++) {
                gp.send1d(drone.at(i));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt*1000)));

        }
        break;
    }

    //getchar();    // wait for user input (to keep pipe open)
    return;
}


std::vector<std::vector<boost::tuple<double, double, double>>> getDronePoints(Vector3d pos, Vector3d ang){
    double dist_diag = 0.3;  // distance between centers of rotors
    double radius_rotor = 0.1;

    // construct drone frame
    std::vector<boost::tuple<double, double, double>> bar_0;
    std::vector<boost::tuple<double, double, double>> bar_1;
    std::vector<boost::tuple<double, double, double>> bar_2;

    bar_0.push_back(boost::make_tuple(dist_diag/sqrt(2), dist_diag/sqrt(2), 0));
    bar_0.push_back(boost::make_tuple(-dist_diag/sqrt(2), -dist_diag/sqrt(2), 0));
    bar_1.push_back(boost::make_tuple(-dist_diag/sqrt(2), dist_diag/sqrt(2), 0));
    bar_1.push_back(boost::make_tuple(dist_diag/sqrt(2), -dist_diag/sqrt(2), 0));
    bar_2.push_back(boost::make_tuple(dist_diag/sqrt(2), dist_diag/sqrt(2), 0));
    bar_2.push_back(boost::make_tuple(dist_diag/sqrt(2), -dist_diag/sqrt(2), 0));

    // construct rotors
    std::vector<boost::tuple<double, double, double>> rotor_fr;
    std::vector<boost::tuple<double, double, double>> rotor_fl;
    std::vector<boost::tuple<double, double, double>> rotor_br;
    std::vector<boost::tuple<double, double, double>> rotor_bl;
    for(double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/20){
        double dist = dist_diag/sqrt(2);
        rotor_fr.push_back(boost::make_tuple(radius_rotor*cos(theta) + dist , radius_rotor*sin(theta) + dist, 0));
        rotor_fl.push_back(boost::make_tuple(radius_rotor*cos(theta) + dist , radius_rotor*sin(theta) - dist, 0));
        rotor_br.push_back(boost::make_tuple(radius_rotor*cos(theta) - dist , radius_rotor*sin(theta) + dist, 0));
        rotor_bl.push_back(boost::make_tuple(radius_rotor*cos(theta) - dist , radius_rotor*sin(theta) - dist, 0));
    }

    std::vector<std::vector<boost::tuple<double, double, double>>> dronePoints;
    dronePoints.push_back(bar_0);
    dronePoints.push_back(bar_1);
    dronePoints.push_back(bar_2);
    dronePoints.push_back(rotor_fr);
    dronePoints.push_back(rotor_fl);
    dronePoints.push_back(rotor_br);
    dronePoints.push_back(rotor_bl);

    dronePoints = rotateDrone(dronePoints, pos, ang);

    return dronePoints;
}

// not very well written yet.
std::vector<std::vector<boost::tuple<double, double, double>>> rotateDrone(std::vector<std::vector<boost::tuple<double, double, double>>> dronePoints, Vector3d pos, Vector3d ang){
    Matrix3d droneFrameToWorldFrameRotation = anglesToRotMatXYZ(ang(0), ang(1), ang(2));

    std::vector<std::vector<boost::tuple<double, double, double>>> newDronePoints;

    for(int i = 0; i < static_cast<int>(dronePoints.size()); i++){
        std::vector<boost::tuple<double, double, double>> newSegment;
        for(int j = 0; j < static_cast<int>(dronePoints.at(i).size()); j++){
            Vector3d oldPoint;
            oldPoint << boost::get<0>(dronePoints.at(i).at(j)), boost::get<1>(dronePoints.at(i).at(j)), boost::get<2>(dronePoints.at(i).at(j));
            Vector3d newPoint = droneFrameToWorldFrameRotation*oldPoint + pos;
            newSegment.push_back(boost::make_tuple(newPoint(0), newPoint(1), newPoint(2)));
        }
        newDronePoints.push_back(newSegment);
    }
    return newDronePoints;
}



