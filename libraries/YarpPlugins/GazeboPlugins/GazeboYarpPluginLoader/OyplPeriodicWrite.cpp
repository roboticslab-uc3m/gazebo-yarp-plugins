// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Bottle.h>

#include "GazeboYarpPluginLoader.hpp"

#include "OyplPeriodicWrite.hpp"

// -----------------------------------------------------------------------------

roboticslab::OyplPeriodicWrite::OyplPeriodicWrite() : yarp::os::PeriodicThread(1.0)
{
    PeriodicThread::start();
}

// -----------------------------------------------------------------------------

void roboticslab::OyplPeriodicWrite::run()
{
    if(!Port::isOpen())
        return;

    yarp::os::Bottle info;
    gazeboYarpPluginLoaderPtr->addYarpPluginsLists(info);

    if(0 == info.size())
        return;

    Port::write(info);
}

// -----------------------------------------------------------------------------
