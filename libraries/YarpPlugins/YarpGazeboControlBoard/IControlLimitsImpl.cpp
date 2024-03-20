#include "YarpGazeboControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::setLimits(int axis, double min, double max)
{
    return false;
}

bool YarpGazeboControlBoard::getLimits(int axis, double *min, double *max)
{
    return false;
}

bool YarpGazeboControlBoard::setVelLimits(int axis, double min, double max)
{
    return false;
}

bool YarpGazeboControlBoard::getVelLimits(int axis, double *min, double *max)
{
    return false;
}
