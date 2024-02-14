cmake_minimum_required(VERSION 3.10)
project(TestYarpGazeboControlBoard VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(YARP REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

add_definitions(${YARP_DEFINITIONS})

set(SOURCE_FILES testYarpGazeboControlBoard.cpp)
add_executable(testYarpGazeboControlBoard ${SOURCE_FILES})
target_link_libraries(testYarpGazeboControlBoard ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES})