
#include "YarpGazeboControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::setPid(const PidControlTypeEnum& pidtype, int j, const Pid &pid)
{
    return true;
}

bool YarpGazeboControlBoard::setPids(const PidControlTypeEnum& pidtype, const Pid *pids)
{
    return true;
}

bool YarpGazeboControlBoard::setPidReference(const PidControlTypeEnum& pidtype, int j, double ref)
{
    return true;
}

bool YarpGazeboControlBoard::setPidReferences(const PidControlTypeEnum& pidtype, const double *refs)
{
    return true;
}

bool YarpGazeboControlBoard::setPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double limit)
{
    return true;
}

bool YarpGazeboControlBoard::setPidErrorLimits(const PidControlTypeEnum& pidtype, const double *limits)
{
    return true;
}

bool YarpGazeboControlBoard::getPidError(const PidControlTypeEnum& pidtype, int j, double *err)
{
    return true;
}

bool YarpGazeboControlBoard::getPidErrors(const PidControlTypeEnum& pidtype, double *errs)
{
    return true;
}

bool YarpGazeboControlBoard::getPidOutput(const PidControlTypeEnum& pidtype, int j, double *out)
{
    return true;
}

bool YarpGazeboControlBoard::getPidOutputs(const PidControlTypeEnum& pidtype, double *outs)
{
    return true;
}

bool YarpGazeboControlBoard::getPid(const PidControlTypeEnum& pidtype, int j, Pid *pid)
{
    return true;
}

bool YarpGazeboControlBoard::getPids(const PidControlTypeEnum& pidtype, Pid *pids)
{
    return true;
}

bool YarpGazeboControlBoard::getPidReference(const PidControlTypeEnum& pidtype, int j, double *ref)
{
    return true;
}

bool YarpGazeboControlBoard::getPidReferences(const PidControlTypeEnum& pidtype, double *refs)
{
    return true;
}

bool YarpGazeboControlBoard::getPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double *limit)
{
    return true;
}

bool YarpGazeboControlBoard::getPidErrorLimits(const PidControlTypeEnum& pidtype, double *limits)
{
    return true;
}

bool YarpGazeboControlBoard::resetPid(const PidControlTypeEnum& pidtype, int j)
{
    return true;
}

bool YarpGazeboControlBoard::disablePid(const PidControlTypeEnum& pidtype, int j)
{
    return true;
}

bool YarpGazeboControlBoard::enablePid(const PidControlTypeEnum& pidtype, int j)
{
    return true;
}

bool YarpGazeboControlBoard::setPidOffset(const PidControlTypeEnum& pidtype, int j, double v)
{
    return true;
}

bool YarpGazeboControlBoard::isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled)
{
    return true;
}
