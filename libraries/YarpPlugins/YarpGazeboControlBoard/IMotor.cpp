
#include "YarpGazeboControlBoard.hpp"

#include <vector>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::getNumberOfMotors(int *num)
{
    *num = _robotModel->GetJoints().size();
    return true;
}

bool YarpGazeboControlBoard::getTemperature(int m, double* val)
{
    auto joints = _robotModel->GetJoints();
    auto joint  = _robotModel->GetJoint(joints[m]->GetName());
    if(joint)
    {
        *val = joint->GetTemperature(0);
    }
    return true;
}

bool YarpGazeboControlBoard::getTemperatures(double *vals)
{
    auto joints = _robotModel->GetJoints();
    for (unsigned int i = 0; i < axes; i++)
    {
        auto joint  = _robotModel->GetJoint(joints[i]->GetName());
        if(joint)
        {
            vals[i] = joint->GetTemperature(0);
        }
    }
    return true;
}

bool YarpGazeboControlBoard::getTemperatureLimit(int m, double *temp)
{
    return true;
}

bool YarpGazeboControlBoard::setTemperatureLimit(int m, const double temp)
{
    return true;
}

bool YarpGazeboControlBoard::getGearboxRatio(int m, double *val)
{
    return true;
}

bool YarpGazeboControlBoard::setGearboxRatio(int m, const double val)
{
    return true;
}
