
#ifndef __YARP_GAZEBO_CONTROL_BOARD_HPP__
#define __YARP_GAZEBO_CONTROL_BOARD_HPP__

#include <vector>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

namespace roboticslab {

    class YarpGazeboControlBoard : public yarp::dev::DeviceDriver,
                                   public yarp::dev::IAxisInfo,
                                   public yarp::dev::IPositionDirect
    {

    public:

        // ------ DeviceDriver declarations. Implementation in IDeviceDriver.cpp ------
        bool open(yarp::os::Searchable& config) override;
        bool close() override;

        // ------- IAxisInfo declarations. Implementation in IAxisInfo.cpp -------
        bool getAxisName(int axis, std::string& name) override;
        bool getJointType(int axis, yarp::dev::JointTypeEnum& type) override;

        // ------ IPositionDirect declarations. Implementation in IPositionDirect.cpp ------
        bool getAxes         (int *ax) override;
        bool setPosition     (int j, double ref) override;
        bool setPositions    (const double * refs) override;
        bool setPositions    (int n_joint, const int *joints, const double *refs) override;
        bool getRefPosition  (int j, double *ref) override;
        bool getRefPositions (double *refs) override;
        bool getRefPositions (int n_joint, const int *joints, double *refs) override;

    private:

        unsigned int axes;

        std::vector<double> encRaw;
        std::vector<double> encRawExposed;
        std::vector<double> targetExposed;

    };

}

#endif