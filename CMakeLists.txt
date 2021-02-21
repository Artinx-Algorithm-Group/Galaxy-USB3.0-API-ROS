cmake_minimum_required(VERSION 3.1)
project(galaxy_camera)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/inc/)

add_executable(GxCamera_node 
    src/GxCamera_node.cc
    src/GxCamera.cc)

target_link_libraries(GxCamera_node 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
    libgxiapi.so)