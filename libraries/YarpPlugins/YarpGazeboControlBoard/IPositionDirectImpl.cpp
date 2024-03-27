#include "YarpGazeboControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::setPosition(int j, double ref)
{
    return false;
}

bool YarpGazeboControlBoard::setPositions(const int n_joint, const int *joints, const double *refs)
{
    return false;
}

bool YarpGazeboControlBoard::setPositions(const double *refs)
{
    return false;
}

bool YarpGazeboControlBoard::getRefPosition(const int joint, double *ref)
{
    return false;
}

bool YarpGazeboControlBoard::getRefPositions(double *refs)
{
    return false;
}

bool YarpGazeboControlBoard::getRefPositions(const int n_joint, const int *joints, double *refs)
{
    return false;
}
