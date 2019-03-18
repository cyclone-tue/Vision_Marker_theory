//
// Created by arnoud on 23-2-19.
//

#ifndef MARKER_VISION_LOGGING_H
#define MARKER_VISION_LOGGING_H

#include <fstream>
#include <vector>
#include <map>
#include <limits>
#include <cmath>
#include <cstdio>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>
#include <opencv/cv.hpp>
#include "Path_Planning/trajectory.h"
#include "Vision/vision.h"

#include "spdlog/spdlog.h"
#include "gnuplot-iostream.h"


using namespace cv;


extern std::shared_ptr<spdlog::logger> vpp_logger;
extern std::shared_ptr<spdlog::logger> pp_logger;
extern std::shared_ptr<spdlog::logger> v_logger;
extern Gnuplot gp;




std::vector<std::vector<boost::tuple<double, double, double>>> getDronePoints(Vector3d pos, Vector3d ang);
std::vector<std::vector<boost::tuple<double, double, double>>> rotateDrone(std::vector<std::vector<boost::tuple<double, double, double>>> dronePoints, Vector3d pos, Vector3d ang);

void showPathInteractive(Trajectory traj, Vector3d hoopTransVec, Matrix3d hoopRotMat);



#endif //MARKER_VISION_LOGGING_H
