#include "YarpGazeboControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;


bool YarpGazeboControlBoard::velocityMove(int j, double sp)
{
    return false;
}

bool YarpGazeboControlBoard::velocityMove(const double *spds)
{
    return false;
}

bool YarpGazeboControlBoard::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    return false;
}

bool YarpGazeboControlBoard::getRefVelocity(const int joint, double *vel)
{
    return false;
}

bool YarpGazeboControlBoard::getRefVelocities(double *vels)
{
    return false;
}

bool YarpGazeboControlBoard::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    return false;
}
