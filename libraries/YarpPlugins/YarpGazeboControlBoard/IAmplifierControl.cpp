#include "YarpGazeboControlBoard.hpp"

#include <vector>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

bool YarpGazeboControlBoard::enableAmp(int j)
{
    return true;
}

bool YarpGazeboControlBoard::disableAmp(int j)
{
    return true;
}

bool YarpGazeboControlBoard::getAmpStatus(int *st)
{
    return true;
}

bool YarpGazeboControlBoard::getAmpStatus(int j, int *v)
{
    return true;
}

bool YarpGazeboControlBoard::getCurrents(double *vals)
{
    return true;
}

bool YarpGazeboControlBoard::getCurrent(int j, double *val)
{
    return true;
}

bool YarpGazeboControlBoard::getMaxCurrent(int j, double *v)
{
    return true;
}

bool YarpGazeboControlBoard::setMaxCurrent(int j, double v)
{
    return true;
}

bool YarpGazeboControlBoard::getNominalCurrent(int m, double *val)
{
    return true;
}

bool YarpGazeboControlBoard::setNominalCurrent(int m, const double val)
{
    return true;
}

bool YarpGazeboControlBoard::getPeakCurrent(int m, double *val)
{
    return true;
}

bool YarpGazeboControlBoard::setPeakCurrent(int m, const double val)
{
    return true;
}

bool YarpGazeboControlBoard::getPWM(int j, double* val)
{
    return true;
}

bool YarpGazeboControlBoard::getPWMLimit(int j, double* val)
{
    return true;
}

bool YarpGazeboControlBoard::setPWMLimit(int j, const double val)
{
    return true;
}

bool YarpGazeboControlBoard::getPowerSupplyVoltage(int j, double* val)
{
    return true;
}
