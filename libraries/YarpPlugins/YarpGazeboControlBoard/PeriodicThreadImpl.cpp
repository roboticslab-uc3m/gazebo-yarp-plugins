#include "YarpGazeboControlBoard.hpp"

#include <yarp/os/Log.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- PeriodicThread Related ------------------------------------

bool YarpGazeboControlBoard::threadInit()
{
    lastTime = yarp::os::Time::now();
    return true;
}

// -----------------------------------------------------------------------------

void YarpGazeboControlBoard::run()
{
    std::vector<double> encsRaw = getEncsRaw();
    std::vector<double> encsExposed = getEncsExposed();

    const double now = yarp::os::Time::now();

    for (unsigned int motor = 0; motor < axes; motor++)
    {
        encsRaw[motor] += velRaw[motor] * (now - lastTime);

        if (jointStatus[motor] != NOT_CONTROLLING)  // if set to move...
        {
            if (encsExposed[motor] > maxLimit[motor] && velRaw[motor] > 0)  // SW max JL
            {
                stop(motor);  // puts jointStatus[motor]=0;
                yCWarning(ECB, "Moving joint q%d at configured max joint limit, stopping", motor + 1);
            }
            else if (encsExposed[motor] < minLimit[motor] && velRaw[motor] < 0)  // SW min JL
            {
                stop(motor);  // puts jointStatus[motor]=0;
                yCWarning(ECB, "Moving joint q%d at configured min joint limit, stopping", motor + 1);
            }
            else if (jointStatus[motor] == POSITION_MOVE)  // check if target reached in pos or rel
            {
                if (velRaw[motor] > 0 &&  // moving positive...
                    encsExposed[motor] > (targetExposed[motor] - jointTol[motor]))
                {
                    stop(motor);  // puts jointStatus[motor]=0;
                    yCInfo(ECB, "Joint q%d reached target", motor + 1);
                }
                else if (velRaw[motor] < 0 &&  // moving negative...
                    encsExposed[motor] < (targetExposed[motor] + jointTol[motor]))
                {
                    stop(motor);  // puts jointStatus[motor]=0;
                    yCInfo(ECB, "Joint q%d reached target", motor + 1);
                }
            }
        }
    }

    setEncsRaw(encsRaw);

    lastTime = yarp::os::Time::now();
}
