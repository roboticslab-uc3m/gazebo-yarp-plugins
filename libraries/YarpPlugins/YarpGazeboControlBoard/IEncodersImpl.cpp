#include "YarpGazeboControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::getAxes(int *ax)
{
    return false;
}

bool YarpGazeboControlBoard::resetEncoder(int j)
{
    return false;
}

bool YarpGazeboControlBoard::resetEncoders()
{
    return false;
}

bool YarpGazeboControlBoard::setEncoder(int j, double val)
{
    return false;
}

bool YarpGazeboControlBoard::setEncoders(const double *vals)
{
    return false;
}

bool YarpGazeboControlBoard::getEncoder(int j, double *v)
{
    return false;
}

bool YarpGazeboControlBoard::getEncoders(double *encs)
{
    return false;
}

bool YarpGazeboControlBoard::getEncoderSpeed(int j, double *sp)
{
    return false;
}

bool YarpGazeboControlBoard::getEncoderSpeeds(double *spds)
{
    return false;
}

bool YarpGazeboControlBoard::getEncoderAcceleration(int j, double *spds)
{
    return false;
}

bool YarpGazeboControlBoard::getEncoderAccelerations(double *accs)
{
    return false;
}
