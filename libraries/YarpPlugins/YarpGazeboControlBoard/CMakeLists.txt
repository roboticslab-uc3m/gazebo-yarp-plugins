
# Verifica si se encontró el paquete YARP
if(NOT YARP_FOUND)
    message(FATAL_ERROR "YARP not found. Please install YARP or adjust your CMake configuration.")
endif()

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

yarp_prepare_plugin(YarpGazeboControlBoard
                    CATEGORY device
                    TYPE roboticslab::YarpGazeboControlBoard
                    INCLUDE YarpGazeboControlBoard.hpp
                    DEFAULT ON
                    EXTRA_CONFIG WRAPPER=controlBoardGazebo_nws_yarp)

find_package(Boost REQUIRED)

if(NOT SKIP_YarpGazeboControlBoard)

    yarp_add_plugin(YarpGazeboControlBoard YarpGazeboControlBoard.hpp
                                            IDeviceDriver.cpp
                                            IAxisInfo.cpp
                                            IPositionDirect.cpp
                                            LogComponent.hpp
                                            LogComponent.cpp)

    target_link_libraries(YarpGazeboControlBoard YARP::YARP_os
                                                   YARP::YARP_dev
                                                   Boost::boost)

    yarp_install(TARGETS YarpGazeboControlBoard
                 LIBRARY DESTINATION  ${ROBOTICSLAB-GAZEBO-YARP-PLUGINS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION  ${ROBOTICSLAB-GAZEBO-YARP-PLUGINS_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-GAZEBO-YARP-PLUGINS_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()