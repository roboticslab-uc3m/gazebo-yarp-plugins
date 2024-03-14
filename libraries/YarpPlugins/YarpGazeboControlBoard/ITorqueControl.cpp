#include "YarpGazeboControlBoard.hpp"

#include <vector>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::getAxes(int *ax)
{
    return true;
}

bool YarpGazeboControlBoard::getRefTorques(double *t)
{
    return true;
}

bool YarpGazeboControlBoard::getRefTorque(int j, double *t)
{
    return true;
}

bool YarpGazeboControlBoard::setRefTorques(const double *t)
{
    return true;
}

bool YarpGazeboControlBoard::setRefTorque(int j, double t)
{
    return true;
}

bool YarpGazeboControlBoard::setRefTorques(const int n_joint, const int *joints, const double *t)
{
    return true;
}

bool YarpGazeboControlBoard::getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params)
{
    return true;
}

bool YarpGazeboControlBoard::setMotorTorqueParams(int j,  const yarp::dev::MotorTorqueParameters params)
{
    return true;
}

bool YarpGazeboControlBoard::getTorque(int j, double *t)
{
    return true;
}

bool YarpGazeboControlBoard::getTorques(double *t)
{
    return true;
}

bool YarpGazeboControlBoard::getTorqueRange(int j, double *min, double *max)
{
    return true;
}

bool YarpGazeboControlBoard::getTorqueRanges(double *min, double *max)
{
    return true;
}
