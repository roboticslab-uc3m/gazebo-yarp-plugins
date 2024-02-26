
#include "YarpGazeboControlBoard.hpp"

#include <vector>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::getAxes(int *ax)
{
    return true;
}

bool YarpGazeboControlBoard::setPosition(int j, double ref)
{
    auto joints = this->robotModel->GetJoints();
    auto joint  = this->robotModel->GetJoint(joints[j]->GetName());
    if(joint)
    {
        joint->SetPosition(0, ref);
    }
    return true;
}

bool YarpGazeboControlBoard::setPositions(const double *refs)
{
    return true;
}

bool YarpGazeboControlBoard::setPositions(int n_joint, const int *joints, const double *refs)
{
    return true;
}

bool YarpGazeboControlBoard::getRefPosition(int j, double *ref)
{
    return true;
}

bool YarpGazeboControlBoard::getRefPositions(double *refs)
{
    return true;
}

bool YarpGazeboControlBoard::getRefPositions(int n_joint, const int *joints, double *refs)
{
    return true;
}