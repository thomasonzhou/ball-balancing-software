cmake_minimum_required(VERSION 3.12)
project(mte-380-software)
set(CMAKE_CXX_STANDARD 17)

add_executable(pose_estimation src/pose_estimation.cc)


set(OpenCV_DIR "/Users/thomasonzhou/repos/build_opencv") # example dir, your installation path may be different

find_package(OpenCV)
include_directories( 
    include
    ${OpenCV_INCLUDE_DIRS}
   "/Users/thomasonzhou/repos/opencv-4.10.0" 
    )

target_link_libraries( pose_estimation ${OpenCV_LIBS} )
