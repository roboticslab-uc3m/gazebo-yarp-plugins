
#include "YarpGazeboControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::getEncodersTimed (double *encs, double *time)
{
    return true;
}

bool YarpGazeboControlBoard::getEncoderTimed (int j, double *encs, double *time)
{
    return true;
}