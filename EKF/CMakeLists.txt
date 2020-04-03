cmake_minimum_required(VERSION 3.1)
project (EKF)

add_definitions(-std=c++11)

# set(CXX_FLAGS "-Wall")
# set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

# Bring headers into the project
include_directories(include)
# Bring Eigen library
include_directories(../Eigen)

# Can manually add the sources using the set command as follows:
# set(SOURCES src/main.cpp src/KalmanFilter.cpp ...)
 
# However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "src/*.?pp")

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
 
add_executable(EKF ${SOURCES})
target_link_libraries(EKF z ssl uv uWS)

