cmake_minimum_required(VERSION 3.1)
project (EKF)

# Bring headers into the project
include_directories(include)
# Bring Eigen library
include_directories(../Eigen)

# Can manually add the sources using the set command as follows:
# set(SOURCES src/main.cpp src/kalman_filter.cpp)
 
# However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "src/*.cpp")
 
add_executable(EKF ${SOURCES})

