
#include "YarpGazeboControlBoard.hpp"

#include <vector>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::getAxes(int *ax)
{
    return true;
}

bool YarpGazeboControlBoard::velocityMove(int j, double sp)
{
    yCTrace(ECB) << j << sp;

    auto joints = _robotModel->GetJoints();
    auto joint  = _robotModel->GetJoint(joints[j]->GetName());
    if(joint)
    {
        joint->SetVelocity(0, sp);
    }
    return true;
}

bool YarpGazeboControlBoard::velocityMove(const double *sp)
{
    yCTrace(ECB);
    bool ok = true;
    
    for (unsigned int i = 0; i < axes; i++)
        ok &= velocityMove(i, sp[i]);
    return ok;
}

bool YarpGazeboControlBoard::setRefAcceleration(int j, double acc)
{
    yCTrace(ECB) << j << acc;
    return true;
}

bool YarpGazeboControlBoard::setRefAccelerations(const double *accs)
{
    yCTrace(ECB);
    return true;
}

bool YarpGazeboControlBoard::getRefAcceleration(int j, double *acc)
{
    return true;
}

bool YarpGazeboControlBoard::getRefAccelerations(double *accs)
{
    return true;
}

bool YarpGazeboControlBoard::stop(int j)
{
    yCTrace(ECB) << j;
    return true;
}

bool YarpGazeboControlBoard::stop()
{
    yCTrace(ECB);
    return true;
}

bool YarpGazeboControlBoard::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    yCTrace(ECB) << n_joint;
    bool ok = true;
    
    for (int i = 0; i < n_joint; i++)
    {
        ok &= velocityMove(joints[i], spds[i]);
    }
    return ok;
}

bool YarpGazeboControlBoard::getRefVelocity(const int joint, double *vel)
{
    return true;
}

bool YarpGazeboControlBoard::getRefVelocities(double *vels)
{
    return true;
}

bool YarpGazeboControlBoard::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    return true;
}

bool YarpGazeboControlBoard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    return true;
}

bool YarpGazeboControlBoard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    return true;
}

bool YarpGazeboControlBoard::stop(const int n_joint, const int *joints)
{
    return true;
}
