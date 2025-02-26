cmake_minimum_required(VERSION 3.10)
project(FingerModel)

# Add the path to the Eigen library
set(EIGEN3_INCLUDE_DIR "/opt/homebrew/include/eigen3")

# Include the Eigen directory
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(finger_model src/finger_model.cpp)
