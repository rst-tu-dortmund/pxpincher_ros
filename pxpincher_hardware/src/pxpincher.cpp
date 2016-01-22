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

#include <pxpincher_hardware/pxpincher.h>
#include <pxpincher_hardware/misc.h>
#include <pxpincher_msgs/Relax.h>
#include <algorithm>
#include <cmath>

namespace pxpincher
{

PxPincher::PxPincher():
    comm_(),
    sim_(params_.simulation_),
    rate_(params_.rate_),
    last_(ros::Time::now()),
    controller_manager_(this),
    nhandle_("pxpincher")
{
    if (!sim_ && !comm_.open(params_.port_,params_.baud_))
    {
        ROS_ERROR_STREAM("Could not open serial connection on port: " << params_.port_ << " with baudrate: " << params_.baud_);
        ROS_ERROR_STREAM("Exiting node, since simulation mode is not activated.");
        ros::shutdown();
    }

    std::size_t no_joints = params_.names_.size();
    
    joint_data_.resize(no_joints);
    
    initRobot();
    
    
    // create ros_control handles
    for (int i = 0; i < no_joints; ++i)
    {
        hardware_interface::JointStateHandle state_handle_joint( params_.names_[i] , &joint_data_[i].pos, &joint_data_[i].vel, &joint_data_[i].eff);
        jnt_state_interface_.registerHandle(state_handle_joint);

        hardware_interface::JointHandle pos_handle_joint(jnt_state_interface_.getHandle(params_.names_[i]),  &joint_data_[i].cmd_pos); // TODO: maybe use handle from above
        jnt_position_interface_.registerHandle(pos_handle_joint);

        hardware_interface::JointHandle vel_handle_joint(jnt_state_interface_.getHandle(params_.names_[i]),  &joint_data_[i].cmd_vel); // TODO: maybe use handle from above
        jnt_velocity_interface_.registerHandle(vel_handle_joint);
    }
    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_position_interface_);
    registerInterface(&jnt_velocity_interface_);

    
    loadDefaultControllers();
    
    state_publisher_ = nhandle_.advertise<sensor_msgs::JointState>("/joint_states",1); // joint_states topic is always root
    diagnostic_publisher_ = nhandle_.advertise<pxpincher_msgs::pxpincher_diagnostic>("diagnostics",1);
    auto relax_fun = [this](pxpincher_msgs::Relax::Request& req, pxpincher_msgs::Relax::Response& resp) {resp.success = relaxServos(req.relaxed>0); return true;};
    relax_service_ = nhandle_.advertiseService("Relax", boost::function<bool(pxpincher_msgs::Relax::Request&,pxpincher_msgs::Relax::Response&)>(relax_fun)); // implicit casting does not work here (gcc)
}

PxPincher::~PxPincher()
{
    ROS_ERROR("Hardware node died. The servos are relaxed now, please catch the falling robot.");
    relaxServos(true);
    ros::shutdown();
    comm_.close();
}

bool PxPincher::canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) const
{
    auto not_vel_and_not_pos = std::find_if(start_list.begin(), start_list.end(), [](const hardware_interface::ControllerInfo& info)
    {
        return !(info.hardware_interface.compare("hardware_interface::PositionJointInterface")==0 || info.hardware_interface.compare("hardware_interface::VelocityJointInterface")==0);
    } );
    return not_vel_and_not_pos == start_list.end(); // we only support position and velocity interfaces
}

void PxPincher::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    for (const hardware_interface::ControllerInfo& info : stop_list)
    {
        for (const std::string& joint : info.resources)
        {
            try
            {
                params_.hardware_modes_[params_.names_idx_map_[joint]] = HardwareMode::STOPPED;
            }
            catch (const std::out_of_range& oor)
            {
                ROS_ERROR_STREAM("doSwitch() Controller Resource " << joint << " is unknown.");
            }
        }
    }
    
    for (const hardware_interface::ControllerInfo& info : start_list)
    {
        for (const std::string& joint : info.resources)
        {
            try
            {
                if (info.hardware_interface.compare("hardware_interface::PositionJointInterface")==0)
                    params_.hardware_modes_[params_.names_idx_map_[joint]] = HardwareMode::POSITION_INTERFACE;
                else if (info.hardware_interface.compare("hardware_interface::VelocityJointInterface")==0)
                    params_.hardware_modes_[params_.names_idx_map_[joint]] = HardwareMode::VELOCITY_INTERFACE;
                else
                {
                    ROS_ERROR_STREAM("Cannot switch controller for joint " << joint << ", desired hardware mode not supported");
                    params_.hardware_modes_[params_.names_idx_map_[joint]] = HardwareMode::STOPPED;
                }
            }
            catch (const std::out_of_range& oor)
            {
                ROS_ERROR_STREAM("doSwitch() Controller Resource " << joint << " is unknown.");
            }
        }
    }
}


void PxPincher::loadDefaultControllers()
{
    std::vector<std::string> controllers;
    
    // load arm_controller
    if (nhandle_.hasParam("/arm_controller"))
    {
        if (controller_manager_.loadController("/arm_controller"))
        {
            ROS_INFO("Arm controller loaded");
            controllers.emplace_back("/arm_controller");
        }
        else
            ROS_WARN("'/arm_controller' could not be loaded by controller_manager. You must load it manually.");
    }
    else
        ROS_WARN("No '/arm_controller' found on ros parameter server. You must define it or load it manually using the controller manager.");
    
    // load gripper controller
    if (nhandle_.hasParam("/gripper_controller"))
    {
        if (controller_manager_.loadController("/gripper_controller"))
        {
            ROS_INFO("Gripper controller loaded");
            controllers.emplace_back("/gripper_controller");
        }
        else
            ROS_WARN("'/gripper_controller' could not be loaded by controller_manager. You must load it manually.");
    }
    else
        ROS_WARN("No '/gripper_controller' found on ros parameter server. You must define it or load it manually using the controller manager.");
    
    
    // we also load our speed forward controller, but we do not start it (it can be started by service calls)
    if (nhandle_.hasParam("/arm_speed_forwarder"))
    {
        if (controller_manager_.loadController("/arm_speed_forwarder"))
        {
            ROS_INFO("Speed forwarding controller loaded");
        }
        else
            ROS_WARN("'/arm_speed_forwarder' could not be loaded by controller_manager. Speed forwarding not supported");
    }
    else
        ROS_WARN("No '/arm_speed_forwarder' found on ros parameter server. Load it if you want to use the speed forwarding interface/controller");
    
    
    if (!controllers.empty())
    {
        auto switch_fun = [this, controllers]()
        {
            if (controller_manager_.switchController(controllers, {}, 2))
            {
                ROS_INFO("Arm controller and gripper controller started.");
            }
            else
                ROS_WARN("Arm controller and gripper controller could not be started.");
        };
        std::thread t(switch_fun); // we must switch them in a separate thread, since we need to call controller_mananger_.update() to actually update
        t.detach(); // this is not safe in case of destructing PxPincher early since the thread accesses a class member, we should create a std::thread t as class member and join in the destructor
    }
    
}

void PxPincher::start()
{
    ros::Rate rate(rate_);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ROS_INFO("PxPincher hardware node started.");

    last_ = ros::Time::now();
    
    while(ros::ok())
    {
        update();
        ros::spinOnce();
        rate.sleep();
    }
}

void PxPincher::update()
{
    // Get current servo status
    std::vector<ServoStatus> stati;
    for (int id : params_.ids_)
    {
        stati.emplace_back(id);
    }
    if (sim_)
        sim_object_.readServoStatus(stati);
    else
        protocol_.readServoStatus(stati,comm_);

    // Verify servo status (saftey check)
    emergencyStopIfRequired(stati);

    // Fill control variables
    fillControlRegister(stati);
    
    // Do control step
    calculateControlStep();

    // Write control command
    performAction();

    // Publish current information
    state_publisher_.publish(getJointState());
    diagnostic_publisher_.publish(getDiagnostics(stati));
}


void PxPincher::emergencyStopIfRequired(const std::vector<ServoStatus>& stati)
{
    if (!ctrl_enabled_)
        return;

    int idx = 0;
    for (const ServoStatus& status : stati)
    {
        if (status.speed_ > 2.0 * params_.speeds_[idx]) // 200% above speed limit
        {
            ROS_ERROR_STREAM("Critical velocity above bounds detected (vel of joint" << (int) params_.ids_[idx] << ": " <<  status.speed_ << "). Emergency stop.");
            ROS_ERROR("Please move the robot to its working space manually and restart the node.");
            
            relaxServos(true);
            ros::shutdown();
        }
        ++idx;
    }
    
    if (!isInsideBounds(stati))
    {
        ROS_ERROR("The robot is currently outside its working space. The servos are relaxed now, please catch the falling robot.");
        ROS_ERROR("Please move the robot to its working space manually and restart the node or use the services to unrelax servos.");
        relaxServos(true);
    }
}



void PxPincher::fillControlRegister(const std::vector<ServoStatus>& stati)
{
    if (stati.size() != joint_data_.size())
    {
        ROS_ERROR("Cannot fill controll register. Number of stati obtained from the hardware communication node does not match number of joints.");
        return;
    }
    
    int idx = 0;
    for (const ServoStatus& status : stati)
    {
        joint_data_[idx].pos = tick2rad( status.position_ - params_.offsets_[idx]);
        joint_data_[idx].vel = tick2rads( status.speed_ );
        ++idx;
    }
    
    // the controller takes some time to initialize, therefore use the initial position as control variable
    if (initial_loop_)
    {
        for (JointData& joint : joint_data_)
        {
            joint.cmd_pos = joint.pos;
        }
        initial_loop_ = false;
    }
}

void PxPincher::calculateControlStep()
{
    ros::Time now = ros::Time::now();
    ros::Duration period = now - last_;
    controller_manager_.update(now,period, ctrl_reset_requested_);
    ctrl_reset_requested_ = false;
    last_ = now;
}

void PxPincher::performAction()
{
    if (!ctrl_enabled_)
        return;
    
    std::vector<int> pos_ticks, vel_ticks;
    
    int idx = 0;
    for(const JointData& joint : joint_data_)
    {
        if (params_.hardware_modes_[idx] == HardwareMode::POSITION_INTERFACE && !std::isnan(joint.cmd_pos))
        {
            pos_ticks.push_back( rad2tick(joint.cmd_pos) + params_.offsets_[idx] );
            vel_ticks.push_back( params_.speeds_[idx] );
        }
        else if (params_.hardware_modes_[idx] == HardwareMode::VELOCITY_INTERFACE && joint.cmd_vel!=0 && !std::isnan(joint.cmd_vel))
        {
            // drive to bounds
            pos_ticks.push_back( joint.cmd_vel < 0 ? params_.cwlimits_[idx] : params_.ccwlimits_[idx] );
            vel_ticks.push_back( rads2tick( std::abs(joint.cmd_vel) ) );
        }
        else // STOP at current position
        {
//              // TODO This does not work as intended, since the robot relaxes itself if all servos are set to keep current position.
//              // WORKAROUND: let the servos drive with speed=2 (workaround below) to the bounds.
//              // The robot does not move actually due to friction.
//             pos_ticks.push_back( joint.cmd_vel < 0 ? params_.cwlimits_[idx] : params_.ccwlimits_[idx] );
            int new_pos;
            if (sim_)
            {
              new_pos = rad2tick(joint_data_[idx].pos) + params_.offsets_[idx]; // keep current position (from sensor reading)
            }
            else
            {
              new_pos = rad2tick(joint_data_[idx].pos) + params_.offsets_[idx] + 5; // workaround only in real-mode
              if (new_pos >= params_.ccwlimits_[idx])
                new_pos -= 10;
            }
            pos_ticks.push_back( new_pos ); // keep current position (from sensor reading) (workaround +5)
            vel_ticks.push_back( 0 );
        }
//         else // STOP at current position
//         {
//             //ROS_INFO_STREAM(idx << ": stopped. is nan: " << std::isnan(joint.cmd_pos) << ", name: " << params_.names_[idx]);
//             pos_ticks.push_back( rad2tick(joint_data_[idx].pos) + params_.offsets_[idx] ); // keep current position (from sensor reading)
//             vel_ticks.push_back( 0 );
//         }

        // workaround: the dynamixel controller drives with full speed if vel = 0 (therefore set to at least 1)
        if (vel_ticks.back() <= 1)
           vel_ticks.back() = 1;
       
        ++idx;
    }
    
    
    // TODO should we always utilize the combined method (speed and position) ?
    if (sim_)
        sim_object_.setGoalPositionAndSpeed( params_.ids_, pos_ticks, vel_ticks );
    else
        protocol_.setGoalPositionAndSpeed( params_.ids_, pos_ticks, vel_ticks, comm_ );
}


bool PxPincher::isInsideBounds(const std::vector<ServoStatus>& stati)
{
    int idx = 0;
    for (const ServoStatus& status : stati)
    {
        if ( status.position_ < 0.95 * params_.cwlimits_[idx] || status.position_ > 1.05 * params_.ccwlimits_[idx])
            return false;
        ++idx;
    }
    return true;
}

sensor_msgs::JointState PxPincher::getJointState()
{
    sensor_msgs::JointState jointStates;

    jointStates.header.stamp = ros::Time::now();

    int idx = 0;
    for (const JointData& joint : joint_data_)
    {
        jointStates.name.push_back( params_.names_[idx] );
        jointStates.position.push_back( joint.pos );
        jointStates.velocity.push_back( joint.vel );
        ++idx;
    }
    
    return jointStates;
}


pxpincher_msgs::pxpincher_diagnostic PxPincher::getDiagnostics(const std::vector<ServoStatus>& stati)
{
    pxpincher_msgs::pxpincher_diagnostic diag;
    
    diag.header.stamp = ros::Time::now();

    int idx = 0;
    for (const ServoStatus& status : stati)
    {
        diag.name.push_back( params_.names_[idx] );
        diag.temperature.push_back( (double) status.temperature_ );
        diag.voltage.push_back( convVoltage( status.voltage_ ) );
        ++idx;
    }

    return diag;
}

bool PxPincher::isMoving()
{
    if (sim_)
        return sim_object_.isMoving();
    
    std::vector<int> moving_vec;
    protocol_.readMoving(params_.ids_, moving_vec, comm_ );
    return std::count_if(moving_vec.begin(), moving_vec.end(), [](int val){return val>0;}) > 0;
}

void PxPincher::initRobot()
{ 
    if (sim_)
    {
        sim_object_.clearJoints();
        for (int i=0; i<(int)params_.ids_.size(); ++i) // TODO: check sizes
        {
            sim_object_.addJoint(params_.ids_[i], params_.names_[i], params_.offsets_[i], params_.speeds_[i], params_.cwlimits_[i], params_.ccwlimits_[i]);
        }
        sim_object_.start(ros::Rate(2*rate_)); // let simulator run with a faster rate (otherwise the timing (execution) might not be well)
    }
    else
    {
        std::vector<UBYTE> ids = params_.ids_;
        std::vector<int> cw_limits = params_.cwlimits_;
        std::vector<int> ccw_limits = params_.ccwlimits_;
        std::vector<int> speeds = params_.speeds_;

        protocol_.setCCWAngleLimit(ids,ccw_limits,comm_);
        ros::Duration(0.01).sleep();

        protocol_.setCWAngleLimit(ids,cw_limits,comm_);
        ros::Duration(0.01).sleep();

        protocol_.setGoalSpeed(ids,speeds,comm_);
        ros::Duration(0.01).sleep();

        // Set comp slope to 128 for all servos
        std::vector<int> cws(params_.ids_.size(),128);
        std::vector<int> ccws(params_.ids_.size(),128);

        protocol_.setComplianceSlope(params_.ids_,cws,ccws,comm_);
        ros::Duration(0.01).sleep();
    }

    // Drive to Home-Position
    //     ROS_INFO("Driving to default position...");
    //     driveToHomePosition(); // blocking call
    //     ROS_INFO("Default position reached. Waiting for trajectory actions or messages ...");
}

void PxPincher::driveToHomePosition()
{
    if (sim_)
        sim_object_.setGoalPosition(params_.ids_, params_.offsets_);
    else
        protocol_.setGoalPosition(params_.ids_, params_.offsets_ ,comm_);
    

    while (isMoving() && ros::ok())
    {
        if (!sim_)
        {
            std::vector<ServoStatus> stati;
            for (int id : params_.ids_)
            {
                stati.emplace_back(id);
            }
            protocol_.readServoStatus(stati,comm_);

            // Verify servo status (saftey check)
            emergencyStopIfRequired(stati);
        }
        ros::Duration(0.01).sleep();
    }
    ctrl_reset_requested_ = true;
}


bool PxPincher::relaxServos(bool relaxed)
{
    if (sim_)
        return true;
    
    bool ret_val = true;
    
    if (!ctrl_enabled_ && !relaxed)
        ctrl_reset_requested_ = true; // reset controllers since torque is going to be enabled and the robot may has moved
    
    ctrl_enabled_ = !relaxed; // switching booleans is thread safe

    std::vector<int> states(params_.ids_.size(),(int) !relaxed);
    protocol_.setTorqueState(params_.ids_,states,comm_);

    return ret_val;
}

} // end namespace pxpincher
