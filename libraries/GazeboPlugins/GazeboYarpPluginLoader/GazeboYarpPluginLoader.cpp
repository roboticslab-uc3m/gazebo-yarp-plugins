// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "GazeboYarpPluginLoader.hpp"

#include <algorithm>

#include <algorithm>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/WrapperSingle.h>

#include "LogComponent.hpp"

using namespace roboticslab;

namespace
{
    bool tryOpenDevice(yarp::os::Property & options, std::vector<yarp::dev::PolyDriver *> & drivers) {

        auto * driver = new yarp::dev::PolyDriver;
        drivers.push_back(driver);

        if (options.check("wrap"))
        {
            yCInfo(ORYPL) << "Requested subdevice wrapping mode";
            return driver->open(options);
        }

        yarp::os::Property mainOptions(options);
        mainOptions.unput("subdevice");

        yarp::os::Property subOptions(options);
        subOptions.put("device", options.find("subdevice"));
        subOptions.unput("subdevice");

        if (!driver->open(mainOptions))
        {
            yCError(ORYPL) << "Could not open main device";
            return false;
        }

        yarp::dev::WrapperSingle * wrapper = nullptr;

        if (!driver->view(wrapper))
        {
            yCError(ORYPL) << "Could not view WrapperSingle";
            return false;
        }

        auto * subDriver = new yarp::dev::PolyDriver;
        drivers.push_back(subDriver);

        if (!subDriver->open(subOptions))
        {
            yCError(ORYPL) << "Could not open subdevice";
            return false;
        }

        if (!wrapper->attach(subDriver))
        {
            yCError(ORYPL) << "Could not attach subdevice";
            return false;
        }

        return true;
        
    }

    void setPortNames(yarp::os::Property & options, const std::string & name)
    {
        if (options.check("ros"))
        {
            options.put("node_name", "/" + name + "/node"); // ROS only
            options.put("topic_name", "/" + name + "/topic"); // ROS only
        }
        else if (options.check("ros2"))
        {
            std::string nodeName = name;
            std::replace(nodeName.begin(), nodeName.end(), '/', '_'); // only alphanumerics and underscores allowed
            options.put("node_name", nodeName); // ROS2 only
            options.put("topic_name", "/" + name); // ROS2 only
            options.put("msgs_name", "/" + name); // ROS2 only (control board-specific)
        }
        else
        {
            options.put("name", "/" + name); // YARP only
        }
    }
}

// -----------------------------------------------------------------------------
GazeboYarpPluginLoader::GazeboYarpPluginLoader(gazebo::physics::WorldPtr world) : world(world){
    yCInfo(ORYPL) << "GazeboYarpPluginLoader constructor";

}

GazeboYarpPluginLoader::~GazeboYarpPluginLoader()
{
    for(int i=0;i<yarpPlugins.size();i++)
    {
        yarpPlugins[i]->close();
        delete yarpPlugins[i];
        yarpPlugins[i] = 0;
    }

    oyplRpcServer.interrupt();
    oyplPeriodicWrite.interrupt();

    oyplRpcServer.close();
    oyplPeriodicWrite.close();
}

// -----------------------------------------------------------------------------

bool GazeboYarpPluginLoader::addYarpPluginsLists(yarp::os::Bottle& info)
{
    // Similar functionality, adapted for Gazebo context
}

// -----------------------------------------------------------------------------

int GazeboYarpPluginLoader::main(const std::string& cmd)
{
    // Process command specific to Gazebo and YARP integration
}

// -----------------------------------------------------------------------------

bool GazeboYarpPluginLoader::Open(std::ostream& sout, std::istream& sinput)
{
    // Adapted functionality for opening YARP plugins in Gazebo
}

// -----------------------------------------------------------------------------

bool GazeboYarpPluginLoader::GetWorld(std::ostream& sout, std::istream& sinput)
{
    // Functionality to get and handle the Gazebo world pointer or information
}

// -----------------------------------------------------------------------------

bool GazeboYarpPluginLoader::close(const int i)
{
    // Similar to the original, adjusted for Gazebo context
}

