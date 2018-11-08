cmake_minimum_required(VERSION 3.5)
project(Marker_Vision)

set(CMAKE_MODULE_PATH /usr/local/lib/cmake)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
include_directories(/usr/local/lib)
set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
list(APPEND OpenCV_LIBS opencv_aruco)
include_directories(${OPENCV_INCLUDE_DIRS})
add_library(opencv INTERFACE IMPORTED)
set_property(TARGET opencv PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${OPENCV_INCLUDE_DIR})

macro( config_project PROJNAME LIBNAME )
    include_directories( ${${LIBNAME}_INCLUDE_DIR} )
    target_link_libraries( ${PROJNAME} ${${LIBNAME}_LIBRARY} )
endmacro()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})


add_executable(GenerateMarker Generate_Markers.cpp)
add_executable(DetectMarker Detect_Markers.cpp)
add_executable(CalibrateCamera calibrate_camera_charuco.cpp)
add_executable(CreateCalibration create_calibration.cpp)
add_library(CycloneVision MODULE Detect_Markers.cpp)

target_link_libraries(GenerateMarker ${OpenCV_LIBS})
target_link_libraries(DetectMarker ${OpenCV_LIBS})
target_link_libraries(CalibrateCamera ${OpenCV_LIBS})
target_link_libraries(CreateCalibration ${OpenCV_LIBS})
target_link_libraries(CycloneVision ${OpenCV_LIBS})
add_dependencies(CycloneVision opencv)