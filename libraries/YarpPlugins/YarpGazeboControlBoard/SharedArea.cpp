#include "YarpGazeboControlBoard.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------

void YarpGazeboControlBoard::setEncRaw(const int index, const double position)
{
    encRawMutex.wait();
    encRaw[index] = position;
    encRawMutex.post();
}

// ----------------------------------------------------------------------------

void YarpGazeboControlBoard::setEncsRaw(const std::vector<double> & positions)
{
    encRawMutex.wait();
    encRaw = positions;
    encRawMutex.post();
}

// ----------------------------------------------------------------------------

double YarpGazeboControlBoard::getEncRaw(const int index)
{
    double position;
    encRawMutex.wait();
    position = encRaw[index];
    encRawMutex.post();
    return position;
}

// ----------------------------------------------------------------------------

std::vector<double> YarpGazeboControlBoard::getEncsRaw()
{
    std::vector<double> positions;
    encRawMutex.wait();
    positions = encRaw;
    encRawMutex.post();
    return positions;
}

// ----------------------------------------------------------------------------

double YarpGazeboControlBoard::getEncExposed(const int index)
{
    double rawPosition = getEncRaw(index);
    return rawPosition / encRawExposed[index];
}

// ----------------------------------------------------------------------------

std::vector<double> YarpGazeboControlBoard::getEncsExposed()
{
    std::vector<double> rawPositions = getEncsRaw();

    for (unsigned int i = 0; i < rawPositions.size(); i++)
    {
        rawPositions[i] /= encRawExposed[i];
    }

    return rawPositions;
}
