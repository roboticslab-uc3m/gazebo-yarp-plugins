option(ENABLE_GazeboYarpPluginLoader "Enable/disable GazeboYarpPluginLoader" ON)

if(ENABLE_GazeboYarpPluginLoader)

    add_library(GazeboYarpPluginLoader MODULE GazeboYarpPluginLoader.cpp
                                                GazeboYarpPluginLoader.hpp
                                                LogComponent.hpp
                                                LogComponent.cpp
                                                OyplPeriodicWrite.cpp
                                                OyplPeriodicWrite.hpp
                                                OyplPortReader.cpp
                                                OyplPortReader.hpp)

    target_link_libraries(GazeboYarpPluginLoader YARP::YARP_os
                                                   YARP::YARP_init
                                                   YARP::YARP_dev
                                                   Boost::boost)

    include(InstallGazeboPlugin)
    install_gazebo_plugin(GazeboYarpPluginLoader)

endif()
