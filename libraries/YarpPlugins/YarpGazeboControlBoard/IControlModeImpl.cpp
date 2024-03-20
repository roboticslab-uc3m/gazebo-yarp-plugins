#include "YarpGazeboControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::getControlMode(int j, int *mode)
{
    return false;
}

bool YarpGazeboControlBoard::getControlModes(int *modes)
{
    return false;
}

bool YarpGazeboControlBoard::getControlModes(const int n_joint, const int *joints, int *modes)
{
    return false;
}

bool YarpGazeboControlBoard::setControlMode(const int j, const int mode)
{
    return false;
}

bool YarpGazeboControlBoard::setControlModes(const int n_joint, const int *joints, int *modes)
{
    return false;
}

bool YarpGazeboControlBoard::setControlModes(int *modes)
{
    return false;
}
