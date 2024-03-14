
#include "YarpGazeboControlBoard.hpp"

#include <vector>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::getNumberOfMotorEncoders(int *num)
{
    *num = _robotModel->GetJoints().size();
    return true;
}   

bool YarpGazeboControlBoard::resetMotorEncoder(int m)
{
    auto joints = _robotModel->GetJoints();
    auto joint  = _robotModel->GetJoint(joints[m]->GetName());
    if(joint)
    {
        joint->SetPosition(0, 0);
    }
    return true;
}

bool YarpGazeboControlBoard::resetMotorEncoders()
{
    auto joints = _robotModel->GetJoints();
    for (unsigned int i = 0; i < axes; i++)
    {
        auto joint  = _robotModel->GetJoint(joints[i]->GetName());
        if(joint)
        {
            joint->SetPosition(0, 0);
        }
    }
    return true;
}

bool YarpGazeboControlBoard::setMotorEncoderCountsPerRevolution(int m, const double cpr)
{
    return true;
}

bool YarpGazeboControlBoard::getMotorEncoderCountsPerRevolution(int m, double *cpr)
{
    return true;
}

bool YarpGazeboControlBoard::setMotorEncoder(int m, const double val)
{
    auto joints = _robotModel->GetJoints();
    auto joint  = _robotModel->GetJoint(joints[m]->GetName());
    if(joint)
    {
        joint->SetPosition(0, val);
    }
    return true;
}

bool YarpGazeboControlBoard::setMotorEncoders(const double *vals)
{
    bool ok = true;
    for (unsigned int i = 0; i < axes; i++)
        ok &= setMotorEncoder(i, vals[i]);
    return ok;
}

bool YarpGazeboControlBoard::getMotorEncoder(int m, double *val)
{
    auto joints = _robotModel->GetJoints();
    auto joint  = _robotModel->GetJoint(joints[m]->GetName());
    if(joint)
    {
        *val = joint->GetAngle(0).Radian();
    }
    return true;
}

bool YarpGazeboControlBoard::getMotorEncoders(double *vals)
{
    bool ok = true;
    for (unsigned int i = 0; i < axes; i++)
        ok &= getMotorEncoder(i, &vals[i]);
    return ok;
}

bool YarpGazeboControlBoard::getMotorEncodersTimed(double *vals, double *time)
{
    return true;
}

bool YarpGazeboControlBoard::getMotorEncoderTimed(int m, double *val, double *time)
{
    return true;
}

bool YarpGazeboControlBoard::getMotorEncoderSpeed(int m, double *sp)
{
    return true;
}

bool YarpGazeboControlBoard::getMotorEncoderSpeeds(double *spds)
{
    return true;
}

bool YarpGazeboControlBoard::getMotorEncoderAcceleration(int m, double *spds)
{
    return true;
}

bool YarpGazeboControlBoard::getMotorEncoderAccelerations(double *accs)
{
    return true;
}
