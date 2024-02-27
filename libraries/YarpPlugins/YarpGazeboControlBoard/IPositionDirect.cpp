
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
    yCTrace(ECB) << j << ref;

    auto joints = _robotModel->GetJoints();
    auto joint  = _robotModel->GetJoint(joints[j]->GetName());
    if(joint)
    {
        joint->SetPosition(0, ref);
    }
    return true;
}

bool YarpGazeboControlBoard::setPositions(const double *refs)
{
    yCTrace(ECB);
    bool ok = true;
    
    for (unsigned int i = 0; i < axes; i++)
        ok &= setPosition(i, refs[i]);
    return ok;
}

bool YarpGazeboControlBoard::setPositions(int n_joint, const int *joints, const double *refs)
{
    yCTrace(ECB) << n_joint;
    bool ok = true;
    
    for (int i = 0; i < n_joint; i++)
    {
        ok &= setPosition(joints[i], refs[i]);
    }
    return ok;
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