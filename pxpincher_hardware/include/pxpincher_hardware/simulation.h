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

#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <sensor_msgs/JointState.h>
#include <pxpincher_hardware/misc.h>
#include <pxpincher_comm/codes.h>
#include <pxpincher_comm/servostatus.h>
#include <ros/callback_queue.h>

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

namespace pxpincher
{

class Simulation
{
public:
      
    Simulation();
    ~Simulation();
	
    void start(ros::Rate rate);

    void addJoint(UBYTE id, const std::string& name, int default_pos, int default_speed, int lower_bound, int upper_bound);
    
    void clearJoints();
    
    void setGoalPosition(UBYTE id, int position);
    void setGoalPosition(const std::vector<UBYTE>& ids, const std::vector<int>& positions);
	
	void setGoalPositionAndSpeed(UBYTE id, int position, int speed);
    void setGoalPositionAndSpeed(const std::vector<UBYTE>& ids, const std::vector<int>& positions, const std::vector<int>& speeds);
	
    void readServoStatus(std::vector<ServoStatus>& stati);
    
    void simCallback(const ros::TimerEvent& event);
    
    bool isMoving();
       

protected:
      void performSimulationStep(double duration);
      bool isGoalReached();
    
private:

    struct JointData 
    {
      JointData() : name(""), pos_rad(0), speed_rad(0), cmd_pos(0), cmd_speed(0), lower(std::numeric_limits<int>::min()), upper(std::numeric_limits<int>::max()), default_pos(0), default_speed(0) {};
      JointData(const std::string& name, double pos_rad, double speed_rad, int cmd_pos, int cmd_speed, int lower, int upper, int default_pos, int default_speed) : name(name), pos_rad(pos_rad), speed_rad(speed_rad),
													      cmd_pos(cmd_pos), cmd_speed(cmd_speed), lower(lower), upper(upper), default_pos(default_pos), default_speed(default_speed) {};
      std::string name;
      double pos_rad;
	  double speed_rad;
      int cmd_pos;
	  int cmd_speed;
      int lower;
      int upper;
	  int default_pos;
	  int default_speed;
    };
    
    bool moving_ = false;
    
    ros::Timer sim_callback_; 
    
    ros::Time last_step_;
    bool started_ = false;
    
    ros::NodeHandle nhandle_;
    
    std::map<UBYTE, JointData> joint_data_;   
    boost::mutex data_mutex_;
    
};

} // end namespace pxpincher

#endif // SIMULATION_H
