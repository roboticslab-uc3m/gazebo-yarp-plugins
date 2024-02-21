
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __GAZEBO_YARP_PLUGIN_LOADER_HPP__
#define __GAZEBO_YARP_PLUGIN_LOADER_HPP__

#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>

#include <yarp/dev/PolyDriver.h>

// Incluir las bibliotecas de Gazebo necesarias
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

namespace roboticslab
{

/**
 * @ingroup GazeboPlugins
 * @defgroup GazeboYarpPluginLoader
 * @brief Contains roboticslab::GazeboYarpPluginLoader.
 */

/**
 * @ingroup GazeboYarpPluginLoader
 * @brief Loads one or several YARP Plugins, passing world pointer.
 */
    class GazeboYarpPluginLoader : public gazebo::ModelPlugin
    {
        public:
            GazeboYarpPluginLoader(gazebo::physics::WorldPtr world);
            ~GazeboYarpPluginLoader();

            bool addYarpPluginsLists(yarp::os::Bottle & info);
            int main(const std::string& cmd);

            bool Open(std::ostream& sout, std::istream& sinput);
            bool GetWorld(std::ostream& sout, std::istream& sinput);
            bool close(const int i);

        private:
            yarp::os::Network yarp;
            std::vector<yarp::dev::PolyDriver*> yarpPlugins;
            std::vector<yarp::os::Property> yarpPluginsProperties;

            yarp::os::RpcServer oyplRpcServer;
            
            gazebo::physics::WorldPtr world; // Referencia al mundo de Gazebo
    };

} // namespace roboticslab

#endif // __GAZEBO_YARP_PLUGIN_LOADER_HPP__
