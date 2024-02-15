// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __OPEN_PORT_PERIODIC_WRITE_HPP__
#define __OPEN_PORT_PERIODIC_WRITE_HPP__

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Port.h>

namespace roboticslab
{

class GazeboYarpPluginLoader;

/**
 * @ingroup GazeboYarpPluginLoader
 * @brief Publishes open YARP plugin list.
 */
class OyplPeriodicWrite : yarp::os::PeriodicThread, public yarp::os::Port
{
public:
    OyplPeriodicWrite();
    void setGazeboYarpPluginLoaderPtr(GazeboYarpPluginLoader *value) { gazeboYarpPluginLoaderPtr = value; }
private:
    GazeboYarpPluginLoader* gazeboYarpPluginLoaderPtr;
    virtual void run() override;
};

} // namespace roboticslab

#endif // __OPEN_PORT_PERIODIC_WRITE_HPP__
