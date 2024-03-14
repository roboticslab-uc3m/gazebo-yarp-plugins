
#ifndef __YARP_GAZEBO_CONTROL_BOARD_HPP__
#define __YARP_GAZEBO_CONTROL_BOARD_HPP__

#include <vector>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

// Incluir las bibliotecas de Gazebo necesarias
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

namespace roboticslab {

    class YarpGazeboControlBoard : public yarp::dev::DeviceDriver,
                                   public yarp::dev::IAxisInfo,
                                   public yarp::dev::IPositionDirect,
                                   public yarp::dev::IJointFault,
                                   public yarp::dev::IPidControl,
                                   public yarp::dev::IPositionControl,
                                   public yarp::dev::IVelocityControl,
                                   public yarp::dev::IMotor,
                                   public yarp::dev::IMotorEncoders,
                                   public yarp::dev::IAmplifierControl,
                                   public yarp::dev::IControlLimits,
                                   public yarp::IControlCalibration,
                                   public yarp::dev::ITorqueControl,
                                   public yarp::dev::IImpedanceControl,
                                   public yarp::dev::IEncoders,
                                   public yarp::dev::IEncodersTimed,
                                   public yarp::dev::IInteractionMode
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

            // ------ IJointFault declarations. Implementation in IJointFault.cpp ------
            bool getLastJointFault(int j, int& fault, std::string& message) override;

            // ------ IPidControl declarations. Implementation in IPidControl.cpp ------
            bool setPid(const PidControlTypeEnum& pidtype, int j, const Pid &pid) override;
            bool setPids(const PidControlTypeEnum& pidtype, const Pid *pids) override;
            bool setPidReference(const PidControlTypeEnum& pidtype, int j, double ref) override;
            bool setPidReferences(const PidControlTypeEnum& pidtype, const double *refs) override;
            bool setPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double limit) override;
            bool setPidErrorLimits(const PidControlTypeEnum& pidtype, const double *limits) override;
            bool getPidError(const PidControlTypeEnum& pidtype, int j, double *err) override;
            bool getPidErrors(const PidControlTypeEnum& pidtype, double *errs) override;
            bool getPidOutput(const PidControlTypeEnum& pidtype, int j, double *out) override;
            bool getPidOutputs(const PidControlTypeEnum& pidtype, double *outs) override;
            bool getPid(const PidControlTypeEnum& pidtype, int j, Pid *pid) override;
            bool getPids(const PidControlTypeEnum& pidtype, Pid *pids) override;
            bool getPidReference(const PidControlTypeEnum& pidtype, int j, double *ref) override;
            bool getPidReferences(const PidControlTypeEnum& pidtype, double *refs) override;
            bool getPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double *limit) override;
            bool getPidErrorLimits(const PidControlTypeEnum& pidtype, double *limits) override;
            bool resetPid(const PidControlTypeEnum& pidtype, int j) override;
            bool disablePid(const PidControlTypeEnum& pidtype, int j) override;
            bool enablePid(const PidControlTypeEnum& pidtype, int j) override;
            bool setPidOffset(const PidControlTypeEnum& pidtype, int j, double v) override;
            bool isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled) override;
            
            // ------ IPositionControl declarations. Implementation in IPositionControl.cpp ------
            bool positionMove(int j, double ref) override;
            bool positionMove(const double *refs) override;
            bool relativeMove(int j, double delta) override;
            bool relativeMove(const double *deltas) override;
            bool checkMotionDone(int j, bool *flag) override;
            bool checkMotionDone(bool *flag) override;
            bool setRefSpeed(int j, double sp) override;
            bool setRefSpeeds(const double *spds) override;
            bool setRefAcceleration(int j, double acc) override;
            bool setRefAccelerations(const double *accs) override;
            bool getRefSpeed(int j, double *ref) override;
            bool getRefSpeeds(double *spds) override;
            bool getRefAcceleration(int j, double *acc) override;
            bool getRefAccelerations(double *accs) override;
            bool stop(int j) override;
            bool stop() override;
            bool positionMove(const int n_joint, const int *joints, const double *refs) override;
            bool relativeMove(const int n_joint, const int *joints, const double *deltas) override;
            bool checkMotionDone(const int n_joint, const int *joints, bool *flag) override;
            bool setRefSpeeds(const int n_joint, const int *joints, const double *spds) override;
            bool setRefAccelerations(const int n_joint, const int *joints, const double *accs) override;
            bool getRefSpeeds(const int n_joint, const int *joints, double *spds) override;
            bool getRefAccelerations(const int n_joint, const int *joints, double *accs) override;
            bool stop(const int n_joint, const int *joints) override;
            bool getTargetPosition(const int joint, double *ref) override;
            bool getTargetPositions(double *refs) override;
            bool getTargetPositions(const int n_joint, const int *joints, double *refs) override;

            // ------ IVelocityControl declarations. Implementation in IVelocityControl.cpp ------
            bool getAxes(int *axes) override;
            bool velocityMove(int j, double sp) override;
            bool velocityMove(const double *sp) override;
            bool setRefAcceleration(int j, double acc) override;
            bool setRefAccelerations(const double *accs) override;
            bool getRefAcceleration(int j, double *acc) override;
            bool getRefAccelerations(double *accs) override;
            bool stop(int j) override;
            bool stop() override;
            bool velocityMove(const int n_joint, const int *joints, const double *spds) override;
            bool getRefVelocity(const int joint, double *vel) override;
            bool getRefVelocities(double *vels) override;
            bool getRefVelocities(const int n_joint, const int *joints, double *vels) override;
            bool setRefAccelerations(const int n_joint, const int *joints, const double *accs) override;
            bool getRefAccelerations(const int n_joint, const int *joints, double *accs) override;
            bool stop(const int n_joint, const int *joints) override;

            // ------ IEncodersTimed declarations. Implementation in IEncodersTimed.cpp ------
            bool getEncodersTimed (double *encs, double *time);
            bool getEncoderTimed (int j, double *encs, double *time);

            // ------ IMotor declarations. Implementation in IMotor.cpp ------
            bool getNumberOfMotors(int *num)
            bool getTemperature(int m, double *val)
            bool getTemperatures(double *vals)
            bool getTemperatureLimit(int m, double *temp)
            bool setTemperatureLimit(int m, const double temp)
            bool getGearboxRatio(int m, double *val)
            bool setGearboxRatio(int m, const double val)

            // ------ IMotorEncoders declarations. Implementation in IMotorEncoders.cpp ------
            bool getNumberOfMotorEncoders(int *num) override;
            bool resetMotorEncoder(int m) override;
            bool resetMotorEncoders() override;
            bool setMotorEncoderCountsPerRevolution(int m, double cpr) override;
            bool getMotorEncoderCountsPerRevolution(int m, double *cpr) override;
            bool setMotorEncoder(int m, double val) override;
            bool setMotorEncoders(const double *vals) override;
            bool getMotorEncoder(int m, double *v) override;
            bool getMotorEncoders(double *encs) override;
            bool getMotorEncodersTimed(double *encs, double *time) override;
            bool getMotorEncoderTimed(int m, double *encs, double *time) override;
            bool getMotorEncoderSpeed(int m, double *sp) override;
            bool getMotorEncoderSpeeds(double *spds) override;
            bool getMotorEncoderAcceleration(int m, double *spds) override;
            bool getMotorEncoderAccelerations(double *accs) override;

            // ------ IAmplifierControl declarations. Implementation in IAmplifierControl.cpp ------
            bool enableAmp(int j) override;
            bool disableAmp(int j) override;
            bool getAmpStatus(int *st) override;
            bool getAmpStatus(int j, int *v) override;
            bool getCurrents(double *vals) override;
            bool getCurrent(int j, double *val) override;
            bool getMaxCurrent(int j, double *v) override;
            bool setMaxCurrent(int j, double v) override;
            bool getNominalCurrent(int m, double *val) override;
            bool setNominalCurrent(int m, const double val) override;
            bool getPeakCurrent(int m, double *val) override;
            bool setPeakCurrent(int m, const double val) override;
            bool getPWM(int j, double* val) override;
            bool getPWMLimit(int j, double* val) override;
            bool setPWMLimit(int j, const double val) override;
            bool getPowerSupplyVoltage(int j, double* val) override;

            // ------ IControlLimits declarations. Implementation in IControlLimits.cpp ------
            bool setLimits(int axis, double min, double max) override;
            bool getLimits(int axis, double *min, double *max) override;
            bool setVelLimits(int axis, double min, double max) override;
            bool getVelLimits(int axis, double *min, double *max) override;

            // ------ IControlCalibration declarations. Implementation in IControlCalibration.cpp ------
            bool calibrateAxisWithParams(int axis, unsigned int type, double p1, double p2, double p3) override;
            bool setCalibrationParameters(int axis, const CalibrationParameters& params) override;
            bool calibrationDone(int j) override;
            bool setCalibrator(ICalibrator *c) override;
            bool calibrateRobot() override;
            bool park(bool wait=true) override;
            bool abortCalibration() override;
            bool abortPark() override;

            // ------ ITorqueControl declarations. Implementation in ITorqueControl.cpp ------
            bool getAxes(int *ax) override;
            bool getRefTorques(double *t) override;
            bool getRefTorque(int j, double *t) override;
            bool setRefTorques(const double *t) override;
            bool setRefTorque(int j, double t) override;
            bool setRefTorques(const int n_joint, const int *joints, const double *t) override;
            bool getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params) override;
            bool setMotorTorqueParams(int j,  const yarp::dev::MotorTorqueParameters params) override;
            bool getTorque(int j, double *t) override;
            bool getTorques(double *t) override;
            bool getTorqueRange(int j, double *min, double *max) override;
            bool getTorqueRanges(double *min, double *max) override;

            // ------ IImpedanceControl declarations. Implementation in IImpedanceControl.cpp ------
            bool getAxes(int *ax) override;
            bool getImpedance(int j, double *stiffness, double *damping) override;
            bool setImpedance(int j, double stiffness, double damping) override;
            bool setImpedanceOffset(int j, double offset) override;
            bool getImpedanceOffset(int j, double* offset) override;
            bool getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp) override;


            // ------ IEncoders declarations. Implementation in IEncoders.cpp ------
            bool resetEncoder(int j) override;
            bool resetEncoders() override;
            bool setEncoder(int j, double val) override;
            bool setEncoders(const double *vals) override;
            bool getEncoder(int j, double *v) override;
            bool getEncoders(double *encs) override;
            bool getEncoderSpeed(int j, double *sp) override;
            bool getEncoderSpeeds(double *spds) override;
            bool getEncoderAcceleration(int j, double *spds) override;
            bool getEncoderAccelerations(double *accs) override;    
            
            // ------ IInteractionMode declarations. Implementation in IInteractionMode.cpp ------
            bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode) override;
            bool getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
            bool getInteractionModes(yarp::dev::InteractionModeEnum* modes) override;
            bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
            bool setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
            bool setInteractionModes(yarp::dev::InteractionModeEnum* modes) override;
            

        private:

            unsigned int axes;

            std::vector<double> encRaw;
            std::vector<double> encRawExposed;
            std::vector<double> targetExposed;

            gazebo::physics::ModelPtr _robotModel;

    };

}

#endif