cmake_minimum_required(VERSION 3.12)
project(Marker_Vision)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")       #https://github.com/facebook/Surround360/issues/3

#set(CMAKE_MODULE_PATH /usr/local/lib/cmake)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
include_directories(/usr/local/lib)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake.modules/")


## libraries


# spdlog
include(cmake.modules/AddSPDlog.txt)
#include_directories(~/spdlog/include)       # v1.3.1

# gnuplot
find_package(Boost REQUIRED COMPONENTS system iostreams)  # Boost::system is needed for gnuplot-iostream.h
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
include(cmake.modules/AddGNUplot.txt)

# eigen
find_package(Eigen3 REQUIRED)   # is found because Eigen3_DIR is set. 1: When was it set? 2: How did the configuration files end up at Eigen3_DIR? Type "$ cmake -LAH" in folder of CMakeLists.txt.
#find_package(spdlog REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# opencv
find_package(OpenCV REQUIRED)
list(APPEND OpenCV_LIBS opencv_aruco)
include_directories(${OPENCV_INCLUDE_DIRS})
#include_directories("/usr/include/spdlog")
#include_directories(~/spdlog/include)       # v1.3.1
add_library(opencv INTERFACE IMPORTED)
set_property(TARGET opencv PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${OPENCV_INCLUDE_DIR})


## other things

macro( config_project PROJNAME LIBNAME )
    include_directories( ${${LIBNAME}_INCLUDE_DIR} )        # seems to be done for eigen already
    target_link_libraries( ${PROJNAME} ${${LIBNAME}_LIBRARY} )
endmacro()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})    # I dont think this is needed


## build executables

# debug executables
#add_executable(GenerateMarker Vision/Generate_Markers.cpp)
#add_executable(DetectMarker Vision/Detect_Markers.cpp Vision/CircleBoard.cpp)
#add_executable(CalibrateCamera Vision/calibrate_camera_charuco.cpp constants.h)
#add_executable(CreateCalibration Vision/create_calibration.cpp)
#add_executable(WebcamTest webcam_test.cpp)

# Building the main executable
set(CVXGEN_PATH Path_Planning/Mueller/cvxgen)
set(CVXGEN ${CVXGEN_PATH}/solver.c ${CVXGEN_PATH}/ldl.c ${CVXGEN_PATH}/matrix_support.c ${CVXGEN_PATH}/util.c)
set(MUELLER Path_Planning/Mueller/path_planner.cpp Path_Planning/trajectory.cpp ${CVXGEN})
set(POLYNOMIAL Path_Planning/Polynomial/gettingPosDers.cpp Path_Planning/Polynomial/math_tools.cpp Path_Planning/Polynomial/posToStatePath.cpp Path_Planning/Polynomial/path_planner.cpp)

set(VISION Vision/Detect_Markers.cpp Vision/CircleBoard.cpp)

add_executable(V_PP  ${VISION} ${MUELLER}  V_PP.cpp logging.cpp general.cpp)


## build libraries

# Building the main library
add_library(CycloneVision MODULE ${VISION} ${MUELLER} V_PP.cpp logging.cpp)


## link libraries

#target_link_libraries(GenerateMarker ${OpenCV_LIBS})
#target_link_libraries(DetectMarker ${OpenCV_LIBS})
#target_link_libraries(CalibrateCamera ${OpenCV_LIBS})
#target_link_libraries(CreateCalibration ${OpenCV_LIBS})
target_link_libraries(CycloneVision ${OpenCV_LIBS})
target_link_libraries(V_PP ${OpenCV_LIBS})
#target_link_libraries(WebcamTest ${OpenCV_LIBS})
target_link_libraries(V_PP ${Boost_LIBRARIES})


# something else
add_dependencies(CycloneVision opencv)
add_dependencies(V_PP spdlog)
add_dependencies(V_PP gnuplot)