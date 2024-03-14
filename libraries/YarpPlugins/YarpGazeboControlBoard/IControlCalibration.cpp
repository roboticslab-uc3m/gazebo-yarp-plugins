#include "YarpGazeboControlBoard.hpp"

#include <vector>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::calibrateAxisWithParams(int axis, unsigned int type, double p1, double p2, double p3)
{
    return true;
}

bool YarpGazeboControlBoard::setCalibrationParameters(int axis, const CalibrationParameters& params)
{
    return true;
}

bool YarpGazeboControlBoard::calibrationDone(int j)
{
    return true;
}

bool YarpGazeboControlBoard::setCalibrator(ICalibrator *c)
{
    return true;
}

bool YarpGazeboControlBoard::calibrateRobot()
{
    return true;
}

bool YarpGazeboControlBoard::park(bool wait=true)
{
    return true;
}

bool YarpGazeboControlBoard::abortCalibration()
{
    return true;
}

bool YarpGazeboControlBoard::abortPark()
{
    return true;
}
