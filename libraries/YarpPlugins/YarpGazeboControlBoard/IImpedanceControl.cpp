
#include "YarpGazeboControlBoard.hpp"

#include <vector>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::getAxes(int *ax)
{
    return true;
}

bool YarpGazeboControlBoard::getImpedance(int j, double *stiffness, double *damping)
{
    return true;
}

bool YarpGazeboControlBoard::setImpedance(int j, double stiffness, double damping)
{
    return true;
}

bool YarpGazeboControlBoard::setImpedanceOffset(int j, double offset)
{
    return true;
}

bool YarpGazeboControlBoard::getImpedanceOffset(int j, double* offset)
{
    return true;
}

bool YarpGazeboControlBoard::getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp)
{
    return true;
}
