cmake_minimum_required(VERSION 3.12)
project(mte-380-software)
set(CMAKE_CXX_STANDARD 17)

add_executable(homing src/homing.cpp)
add_executable(calibration src/calibration.cpp)
add_executable(marker_generation src/marker_generation.cpp)
add_executable(ball_detection src/ball_detection.cpp)

set(OpenCV_DIR "/Users/thomasonzhou/repos/build_opencv") # example dir, your installation path may be different

find_package(OpenCV)
include_directories( 
    include
    ${OpenCV_INCLUDE_DIRS}
   "/Users/thomasonzhou/repos/opencv-4.10.0" 
    )

target_link_libraries( homing ${OpenCV_LIBS} )
target_link_libraries( calibration ${OpenCV_LIBS} )
target_link_libraries( marker_generation ${OpenCV_LIBS} )
target_link_libraries( ball_detection ${OpenCV_LIBS} )

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
