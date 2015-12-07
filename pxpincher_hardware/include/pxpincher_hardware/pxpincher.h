/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Maximilian Krämer, Christoph Rösmann
 *********************************************************************/

#ifndef PXPINCHER_H
#define PXPINCHER_H

#include <string>
#include <math.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <pxpincher_msgs/pxpincher_diagnostic.h>

#include <ros/ros.h>

#include <pxpincher_comm/pxprotocol.h>
#include <pxpincher_comm/serialcomm.h>
#include <pxpincher_comm/servostatus.h>
#include "pxparameter.h"

#include "simulation.h"


namespace pxpincher
{

class PxPincher : public hardware_interface::RobotHW
{
public:
    PxPincher();
    ~PxPincher();

    void start();
    
    
    /**
    * Check (in non-realtime) if given controllers could be started and stopped from the current state of the RobotHW
    * with regard to necessary hardware interface switches. Start and stop list are disjoint.
    * This is just a check, the actual switch is done in doSwitch()
    */
    virtual bool canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) const override;

    /**
    * Perform (in non-realtime) all necessary hardware interface switches in order to start and stop the given controllers.
    * Start and stop list are disjoint. The feasability was checked in canSwitch() beforehand.
    */
    virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) override;


protected:
    
    void loadDefaultControllers();
    void update();
    void fillControlRegister(const std::vector<ServoStatus>& stati);
    void calculateControlStep();
    void performAction();
    bool isMoving();
    void driveToHomePosition();
    void initRobot();
    void emergencyStopIfRequired(const std::vector<ServoStatus>& stati);
    bool relaxServos(bool relaxed);
    bool isInsideBounds(const std::vector<ServoStatus>& stati);
    
private:
    
    sensor_msgs::JointState getJointState();
    pxpincher_msgs::pxpincher_diagnostic getDiagnostics(const std::vector<ServoStatus>& stati);

    void simulationCallback(const sensor_msgs::JointStateConstPtr &state);

    ros::Publisher state_publisher_, diagnostic_publisher_;
    ros::ServiceServer relax_service_;
    ros::NodeHandle nhandle_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_position_interface_;
	hardware_interface::VelocityJointInterface jnt_velocity_interface_;
    ros::Time last_;
      
    controller_manager::ControllerManager controller_manager_;
        
    Simulation sim_object_;
    PXParameter params_;

    SerialComm comm_;
    PXProtocol protocol_;

    
    struct JointData
    {   
        double cmd_pos = NAN; // this must be checked before sending! (see performAction()), // NAN defined for C++11
        double cmd_vel = 0;
        double pos = 0;
        double vel = 0;
        double eff = 0;
    };
    
    std::vector<JointData> joint_data_;

    bool initial_loop_ = true;

    bool ctrl_enabled_ = true; //!< This state allows controlling the robot (it is set to false if motors are relaxed)
    bool ctrl_reset_requested_ = true;
    bool sim_;
    int rate_;

};

} // end namespace pxpincher

#endif // PXPINCHER_H
