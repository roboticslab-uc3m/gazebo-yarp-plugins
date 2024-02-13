#include "YarpGazeboControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::getAxisName(int axis, std::string& name)
{
    return true;
}

bool YarpGazeboControlBoard::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    return true;
}