#include "YarpGazeboControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode)
{
    return false;
}

bool getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    return false;
}

bool setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    return false;
}
