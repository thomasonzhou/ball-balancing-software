cmake_minimum_required(VERSION 3.12)
project(mte-380-software)
set(CMAKE_CXX_STANDARD 17)

add_executable(aruco_detection src/aruco_detection_rpi.cpp)
add_executable(calibration src/calibration_rpi.cpp)
add_executable(marker_generation src/marker_generation.cpp)
add_executable(ball_detection src/ball_detection.cpp)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

find_package(OpenCV)
include_directories( 
    include
    ${LIBCAMERA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

target_link_libraries( aruco_detection -llccv ${OpenCV_LIBS} )
target_link_libraries( calibration -llccv ${OpenCV_LIBS} )
target_link_libraries( marker_generation ${OpenCV_LIBS} )
target_link_libraries( ball_detection ${OpenCV_LIBS} )

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
