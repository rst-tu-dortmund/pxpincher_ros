#ifndef PXPINCHER_H
#define PXPINCHER_H

#include <string>
#include <math.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "pxpincher_rst_msgs/pxpincher_rst_diagnostic.h"

#include "ros/ros.h"

#include "pxpincher_comm/pxprotocol.h"
#include "pxpincher_comm/serialcomm.h"
#include "pxpincher_comm/servostatus.h"
#include "pxparameter.h"

#include "simulation.h"

class PxPincher : public hardware_interface::RobotHW
{
public:
    PxPincher();
    ~PxPincher();

    void start();

private:

    void update();
    void fillControlRegister(std::vector<ServoStatus> stati);
    void calculateControlStep();
    void performAction();
    void initRobot();

    std::vector<double> tick2rad(std::vector<int> positions);
    std::vector<double> tick2rads(std::vector<int> speeds);
    std::vector<double> convVoltage(std::vector<int> volt);
    sensor_msgs::JointState getJointState(std::vector<ServoStatus> stati);
    pxpincher_rst_msgs::pxpincher_rst_diagnostic getDiagnostics(std::vector<ServoStatus> stati);

    void simulationCallback(const sensor_msgs::JointStateConstPtr &state);

    ros::Publisher statePublisher_, diagnosticPublisher_;
    ros::Subscriber simSubscriber_;
    ros::NodeHandle nHandle_;

    hardware_interface::JointStateInterface jntStateInterface;
    hardware_interface::PositionJointInterface jntPositionInterface;
    controller_manager::ControllerManager controllerManager_;

    Simulation simObject_;
    PXParameter paramObject_;

    SerialComm comm_;
    PXProtocol protocol_;

    double cmd[5];
    double pos[5];
    double vel[5];
    double eff[5];

    const double PI = 3.141592653589793;
    const double conversionFactor1 = 5*PI/3066;
    const double conversionFactor2 = 19*PI/5115;

    ros::Time last_;

    bool sim_;
    int rate_;

};

#endif // PXPINCHER_H
