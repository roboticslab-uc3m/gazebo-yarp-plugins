#include "YarpGazeboControlBoard.hpp"

#include <vector>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::setLimits(int axis, double min, double max)
{
    return true;
}

bool YarpGazeboControlBoard::getLimits(int axis, double *min, double *max)
{
    return true;
}

bool YarpGazeboControlBoard::setVelLimits(int axis, double min, double max)
{
    return true;
}

bool YarpGazeboControlBoard::getVelLimits(int axis, double *min, double *max)
{
    return true;
}
