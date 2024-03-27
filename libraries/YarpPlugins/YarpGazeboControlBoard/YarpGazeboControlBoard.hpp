
#ifndef __YARP_GAZEBO_CONTROL_BOARD_HPP__
#define __YARP_GAZEBO_CONTROL_BOARD_HPP__

#include <vector>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
//#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Semaphore.h>

// Incluir las bibliotecas de Gazebo necesarias
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>


namespace roboticslab {

    class YarpGazeboControlBoard : public yarp::dev::DeviceDriver,
                                   public yarp::dev::IControlLimits,
                                   public yarp::dev::IControlMode,
                                   public yarp::dev::IEncoders,
                                   public yarp::dev::IPositionControl,
                                   public yarp::dev::IPositionDirect,
                                   public yarp::dev::IVelocityControl
                                   //public yarp::dev::PeriodicThread
    {

        public:

            // Set the thread period in the class constructor
            //YarpGazeboControlBoard() : PeriodicThread(1.0) {} // In seconds

            // ------ DeviceDriver declarations. Implementation in IDeviceDriver.cpp ------
            bool open(yarp::os::Searchable& config) override;
            bool close() override;

            // ------ IControlLimits declarations. Implementation in IControlLimits.cpp ------
            bool setLimits(int axis, double min, double max) override;
            bool getLimits(int axis, double *min, double *max) override;
            bool setVelLimits(int axis, double min, double max) override;
            bool getVelLimits(int axis, double *min, double *max) override;

            // ------ IControlMode declarations. Implementation in IControlMode.cpp ------
            virtual bool getControlMode(int j, int *mode) override;
            virtual bool getControlModes(int *modes) override;
            virtual bool getControlModes(const int n_joint, const int *joints, int *modes) override;
            virtual bool setControlMode(const int j, const int mode) override;
            virtual bool setControlModes(const int n_joint, const int *joints, int *modes) override;
            virtual bool setControlModes(int *modes) override;

            // ------ IEncoders declarations. Implementation in IEncoders.cpp ------
            virtual bool getAxes(int *ax) override;
            virtual bool resetEncoder(int j) override;
            virtual bool resetEncoders() override;
            virtual bool setEncoder(int j, double val) override;
            virtual bool setEncoders(const double *vals) override;
            virtual bool getEncoder(int j, double *v) override;
            virtual bool getEncoders(double *encs) override;
            virtual bool getEncoderSpeed(int j, double *sp) override;
            virtual bool getEncoderSpeeds(double *spds) override;
            virtual bool getEncoderAcceleration(int j, double *spds) override;
            virtual bool getEncoderAccelerations(double *accs) override;

            // ------ IPositionControl declarations. Implementation in IPositionControl.cpp ------
            virtual bool positionMove(int j, double ref) override;
            virtual bool positionMove(const double *refs) override;
            virtual bool relativeMove(int j, double delta) override;
            virtual bool relativeMove(const double *deltas) override;
            virtual bool checkMotionDone(int j, bool *flag) override;
            virtual bool checkMotionDone(bool *flag) override;
            virtual bool setRefSpeed(int j, double sp) override;
            virtual bool setRefSpeeds(const double *spds) override;
            virtual bool setRefAcceleration(int j, double acc) override;
            virtual bool setRefAccelerations(const double *accs) override;
            virtual bool getRefSpeed(int j, double *ref) override;
            virtual bool getRefSpeeds(double *spds) override;
            virtual bool getRefAcceleration(int j, double *acc) override;
            virtual bool getRefAccelerations(double *accs) override;
            virtual bool stop(int j) override;
            virtual bool stop() override;
            virtual bool positionMove(const int n_joint, const int *joints, const double *refs) override;
            virtual bool relativeMove(const int n_joint, const int *joints, const double *deltas) override;
            virtual bool checkMotionDone(const int n_joint, const int *joints, bool *flag) override;
            virtual bool setRefSpeeds(const int n_joint, const int *joints, const double *spds) override;
            virtual bool setRefAccelerations(const int n_joint, const int *joints, const double *accs) override;
            virtual bool getRefSpeeds(const int n_joint, const int *joints, double *spds) override;
            virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs) override;
            virtual bool stop(const int n_joint, const int *joints) override;
            virtual bool getTargetPosition(const int joint, double *ref) override;
            virtual bool getTargetPositions(double *refs) override;
            virtual bool getTargetPositions(const int n_joint, const int *joints, double *refs) override;

            // ------ IPositionDirect declarations. Implementation in IPositionDirect.cpp ------
            virtual bool setPosition(int j, double ref) override;
            virtual bool setPositions(const int n_joint, const int *joints, const double *refs) override;
            virtual bool setPositions(const double *refs) override;
            virtual bool getRefPosition(const int joint, double *ref) override;
            virtual bool getRefPositions(double *refs) override;
            virtual bool getRefPositions(const int n_joint, const int *joints, double *refs) override;

            // ------ IVelocityControl declarations. Implementation in IVelocityControl.cpp ------
            virtual bool velocityMove(int j, double sp) override;
            virtual bool velocityMove(const double *sp) override;
            virtual bool velocityMove(const int n_joint, const int *joints, const double *spds) override;
            virtual bool getRefVelocity(const int joint, double *vel) override;
            virtual bool getRefVelocities(double *vels) override;
            virtual bool getRefVelocities(const int n_joint, const int *joints, double *vels) override;

            // -------- PeriodicThread declarations. Implementation in PeriodicThreadImpl.cpp --------
            //bool threadInit() override;
            //void run() override;

            // ----- Shared Area Funcion declarations. Implementation in SharedArea.cpp -----
            void setEncRaw(const int index, const double position);
            void setEncsRaw(const std::vector<double> & positions);
            double getEncRaw(const int index);
            std::vector<double> getEncsRaw();
            double getEncExposed(const int index);
            std::vector<double> getEncsExposed();

        private:

            enum jmc_state { NOT_CONTROLLING, POSITION_MOVE, RELATIVE_MOVE, VELOCITY_MOVE };
            enum jmc_mode { POSITION_MODE, VELOCITY_MODE, POSITION_DIRECT_MODE };

            bool setPositionMode(int j);
            bool setVelocityMode(int j);
            bool setTorqueMode(int j);
            bool setPositionDirectMode(int j);

            // General Joint Motion Controller parameters //
            unsigned int axes;
            double jmcMs;
            jmc_mode controlMode;
            double lastTime;

            yarp::os::Semaphore encRawMutex;  // SharedArea

            std::vector<jmc_state> jointStatus;

            std::vector<double> encRaw;
            std::vector<double> encRawExposed;  // For conversion.
            std::vector<double> initPos;  // Exposed.
            std::vector<double> jointTol;  // Exposed.
            std::vector<double> maxLimit;  // Exposed.
            std::vector<double> minLimit;  // Exposed.
            std::vector<double> refAcc;  // Exposed.
            std::vector<double> refSpeed;  // Exposed.
            std::vector<double> targetExposed;  // Exposed.
            std::vector<double> velRawExposed;  // For conversion.
            std::vector<double> velRaw;

            gazebo::physics::ModelPtr _robotModel;

    };

}

#endif