cmake_minimum_required(VERSION 3.12)
project(mte-380-software)

add_executable(pose_estimation pose_estimation/pose_estimation.cc)

# find_package( OpenCV REQUIRED )
# include_directories( ${OpenCV_INCLUDE_DIRS} )

# target_link_libraries( DisplayImage ${OpenCV_LIBS} )
