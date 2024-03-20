#include "YarpGazeboControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::getAxes(int *ax)
{
    return false;
}

bool YarpGazeboControlBoard::velocityMove(int j, double sp)
{
    return false;
}

bool YarpGazeboControlBoard::velocityMove(const double *spds)
{
    return false;
}

bool YarpGazeboControlBoard::setRefAcceleration(int j, double acc)
{
    return false;
}

bool YarpGazeboControlBoard::setRefAccelerations(const double *accs)
{
    return false;
}

bool YarpGazeboControlBoard::getRefAcceleration(int j, double *acc)
{
    return false;
}

bool YarpGazeboControlBoard::getRefAccelerations(double *accs)
{
    return false;
}

bool YarpGazeboControlBoard::stop(int j)
{
    return false;
}

bool YarpGazeboControlBoard::stop()
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

bool YarpGazeboControlBoard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    return false;
}

bool YarpGazeboControlBoard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    return false;
}

bool YarpGazeboControlBoard::stop(const int n_joint, const int *joints)
{
    return false;
}
