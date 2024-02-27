cmake_minimum_required(VERSION 3.22)

# Busca el paquete YARP
find_package(YARP REQUIRED)

# Verifica si se encontró el paquete YARP
if(NOT YARP_FOUND)
    message(FATAL_ERROR "YARP not found. Please install YARP or adjust your CMake configuration.")
endif()

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

option(ENABLE_GazeboYarpPluginLoader "Enable/disable GazeboYarpPluginLoader" ON)

if(ENABLE_GazeboYarpPluginLoader)

    add_library(GazeboYarpPluginLoader MODULE GazeboYarpPluginLoader.cpp
                                              GazeboYarpPluginLoader.hpp
                                              LogComponent.hpp
                                              LogComponent.cpp)

    target_link_libraries(GazeboYarpPluginLoader YARP::YARP_os
                                                 YARP::YARP_init
                                                 YARP::YARP_dev
                                                 ${GAZEBO_LIBRARIES})

    #include(InstallGazeboPlugin)
    #install_gazebo_plugin(GazeboYarpPluginLoader)

endif()
