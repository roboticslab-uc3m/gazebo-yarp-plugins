#include "YarpGazeboControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::positionMove(int j, double ref)
{
    return false;
}

bool YarpGazeboControlBoard::positionMove(const double * refs)
{
    return false;
}

bool YarpGazeboControlBoard::relativeMove(int j, double delta)
{
    return false;
}

bool YarpGazeboControlBoard::relativeMove(const double * deltas)
{
    return false;
}

bool YarpGazeboControlBoard::checkMotionDone(int j, bool * flag)
{
    return false;
}

bool YarpGazeboControlBoard::checkMotionDone(bool * flag)
{
    return false;
}

bool YarpGazeboControlBoard::setRefSpeed(int j, double sp)
{
    return false;
}

bool YarpGazeboControlBoard::setRefSpeeds(const double * spds)
{
    return false;
}

bool YarpGazeboControlBoard::setRefAcceleration(int j, double acc)
{
    return false;
}

bool YarpGazeboControlBoard::setRefAccelerations(const double * accs)
{
    return false;
}

bool YarpGazeboControlBoard::getRefSpeed(int j, double * ref)
{
    return false;
}

bool YarpGazeboControlBoard::getRefSpeeds(double * spds)
{
    return false;
}

bool YarpGazeboControlBoard::getRefAcceleration(int j, double * acc)
{
    return false;
}

bool YarpGazeboControlBoard::getRefAccelerations(double * accs)
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

bool YarpGazeboControlBoard::positionMove(const int n_joint, const int * joints, const double * refs)
{
    return false;
}

bool YarpGazeboControlBoard::relativeMove(const int n_joint, const int * joints, const double * deltas)
{
    return false;
}

bool YarpGazeboControlBoard::checkMotionDone(const int n_joint, const int * joints, bool * flag)
{
    return false;
}

bool YarpGazeboControlBoard::setRefSpeeds(const int n_joint, const int * joints, const double * spds)
{
    return false;
}

bool YarpGazeboControlBoard::setRefAccelerations(const int n_joint, const int * joints, const double * accs)
{
    return false;
}

bool YarpGazeboControlBoard::getRefSpeeds(const int n_joint, const int * joints, double * spds)
{
    return false;
}

bool YarpGazeboControlBoard::getRefAccelerations(const int n_joint, const int * joints, double * accs)
{
    return false;
}

bool YarpGazeboControlBoard::stop(const int n_joint, const int * joints)
{
    return false;
}

bool YarpGazeboControlBoard::getTargetPosition(const int joint, double * ref)
{
    return false;
}

bool YarpGazeboControlBoard::getTargetPositions(double * refs)
{
    return false;
}

bool YarpGazeboControlBoard::getTargetPositions(const int n_joint, const int * joints, double * refs)
{
    return false;
}
