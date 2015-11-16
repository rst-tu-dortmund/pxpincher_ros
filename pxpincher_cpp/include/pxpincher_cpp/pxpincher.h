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
    void fillControlRegister(const std::vector<ServoStatus>& stati);
    void calculateControlStep();
    void performAction();
    void initRobot();

    static double tick2rad(int position);
    static std::vector<double> tick2rad(const std::vector<int>& positions);
    
    static double tick2rads(int speed);
    static std::vector<double> tick2rads(const std::vector<int>& speeds);
    
    static double convVoltage(int volt);
    static std::vector<double> convVoltage(const std::vector<int>& volt);

    sensor_msgs::JointState getJointState();
    pxpincher_rst_msgs::pxpincher_rst_diagnostic getDiagnostics(const std::vector<ServoStatus>& stati);

    void simulationCallback(const sensor_msgs::JointStateConstPtr &state);

    ros::Publisher statePublisher_, diagnosticPublisher_;
    ros::Subscriber simSubscriber_;
    ros::NodeHandle nHandle_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_position_interface_;

    
    controller_manager::ControllerManager controllerManager_;

    Simulation simObject_;
    PXParameter paramObject_;

    SerialComm comm_;
    PXProtocol protocol_;

    
    struct JointInfo
    {
        double cmd = 0;
        double pos = 0;
        double vel = 0;
        double eff = 0;
    };
    
    std::vector<JointInfo> joint_info_;


    constexpr static double PI = 3.141592653589793;
    constexpr static double conversionFactor1 = 5*PI/3066;
    constexpr static double conversionFactor2 = 19*PI/5115;

    ros::Time last_;

    bool sim_;
    int rate_;

};

#endif // PXPINCHER_H
