// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __OPEN_PORT_READER_HPP__
#define __OPEN_PORT_READER_HPP__

#include <yarp/os/PortReader.h>

namespace roboticslab
{

class GazeboYarpPluginLoader;

/**
 * @ingroup GazeboYarpPluginLoader
 * @brief Processes RPC requests for GazeboYarpPluginLoader.
 */
class OyplPortReader: public yarp::os::PortReader
{
public:
    void setGazeboYarpPluginLoaderPtr(GazeboYarpPluginLoader *value) { gazeboYarpPluginLoaderPtr = value; }
private:
    GazeboYarpPluginLoader* gazeboYarpPluginLoaderPtr;

    bool read(yarp::os::ConnectionReader& in) override;
};

} // namespace roboticslab

#endif // __OPEN_PORT_READER_HPP__
