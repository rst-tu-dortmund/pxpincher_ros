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

#include <pxpincher_hardware/simulation.h>
#include <cmath>


namespace pxpincher
{

Simulation::Simulation() : nhandle_("sim") 
{
}

Simulation::~Simulation()
{
	sim_callback_.stop();
}

void Simulation::start(ros::Rate rate)
{
    sim_callback_= nhandle_.createTimer(rate, &Simulation::simCallback, this);
}

void Simulation::addJoint(UBYTE id, const std::string& name, int default_pos, int default_speed, int lower_bound, int upper_bound)
{
	boost::mutex::scoped_lock lock(data_mutex_);
    joint_data_[id] = JointData(name, 0, 0, default_pos, default_speed, lower_bound, upper_bound, default_pos, default_speed);
}

void Simulation::clearJoints()
{
	boost::mutex::scoped_lock lock(data_mutex_);
    joint_data_.clear();
}

void Simulation::setGoalPosition(UBYTE id, int position)
{
    boost::mutex::scoped_lock lock(data_mutex_);
    try
    {
		joint_data_.at(id).cmd_pos = position;
		joint_data_.at(id).cmd_speed = joint_data_.at(id).default_speed;
		moving_ = true;
		
		if (!started_)
		{
			last_step_ = ros::Time::now();
			started_ = true;
			return;
		}
    }
    catch (const std::out_of_range& oor)
    {
      ROS_ERROR_STREAM("Simulation::setGoalPosition(): invalid id: " << (int) id);
    }
}

void Simulation::setGoalPosition(const std::vector<UBYTE>& ids, const std::vector<int>& positions)
{
    ROS_ASSERT(ids.size() == positions.size());
    int idx = 0;
    for (UBYTE id : ids)
    {
      setGoalPosition(id, positions[idx]);
      ++idx;
    }
}

void Simulation::setGoalPositionAndSpeed(UBYTE id, int position, int speed)
{
	boost::mutex::scoped_lock lock(data_mutex_);
    try
    {
		joint_data_.at(id).cmd_pos = position;
		joint_data_.at(id).cmd_speed = speed;
		moving_ = true;
		
		if (!started_)
		{
			last_step_ = ros::Time::now();
			started_ = true;
			return;
		}
    }
    catch (const std::out_of_range& oor)
    {
      ROS_ERROR_STREAM("Simulation::setGoalPosition(): invalid id: " << (int) id);
    }
}
void Simulation::setGoalPositionAndSpeed(const std::vector<UBYTE>& ids, const std::vector<int>& positions, const std::vector<int>& speeds)
{
     ROS_ASSERT(ids.size() == positions.size() && ids.size() == speeds.size());
    int idx = 0;
    for (UBYTE id : ids)
    {
      setGoalPositionAndSpeed(id, positions[idx], speeds[idx]);
      ++idx;
    }
}

void Simulation::readServoStatus(std::vector<ServoStatus>& stati)
{
    boost::mutex::scoped_lock lock(data_mutex_);
    
    // Get the ids
    for(ServoStatus& elem : stati)
    { 
		try
		{
			const JointData& data = joint_data_.at(elem.id_);
			elem.load_ = 0; // no load model simulated
			elem.position_ = rad2tick(data.pos_rad) + data.default_pos;
			elem.speed_ = rads2tick(data.speed_rad);
			elem.temperature_ = 0;
			elem.voltage_ = 0;
		}
		catch (const std::out_of_range& oor)
		{
			ROS_ERROR_STREAM("Simulation::readServoStatus(): invalid id: " << (int) elem.id_);
		}
    }
    
}

void Simulation::simCallback(const ros::TimerEvent& event)
{
  if (!moving_) // moving_ is requested by setGoalPosition
    return;
  
  ros::Time now = ros::Time::now();
  
  performSimulationStep( (now-last_step_).toSec() );
  
  last_step_ = now;
  
  // check goal condition to cancel moving
  if (isGoalReached())
  {
    moving_ = false;
  }
}

bool Simulation::isGoalReached()
{
    std::vector<bool> reached;
	{ // scope for mutex
		boost::mutex::scoped_lock lock(data_mutex_);
		for (const auto& data : joint_data_)
		{
			reached.push_back( std::abs(tick2rad(data.second.cmd_pos - data.second.default_pos) - data.second.pos_rad) < 0.001 );
		}
	}
    return std::find(reached.begin(), reached.end(), false) == reached.end();
}

bool Simulation::isMoving()
{
    return moving_;
}

void Simulation::performSimulationStep(double duration)
{
    boost::mutex::scoped_lock lock(data_mutex_);
    int idx = 0;
    for (std::pair<const UBYTE, JointData>& data : joint_data_)
    {
      double speed_max = tick2rads(data.second.cmd_speed); 
      // limit speed close to goal
      double dist = tick2rad(data.second.cmd_pos-data.second.default_pos) - data.second.pos_rad;
      double speed_req = std::abs(dist) / duration;
      double speed_des = std::min(speed_max, speed_req);
	  //ROS_INFO_STREAM(idx << " max: " << speed_max << " req: " << speed_req << " des: " << speed_des);
	  data.second.speed_rad = speed_des; // use desired and commanded speed (model) for the status message
            
      data.second.pos_rad += double(sign(dist)) * speed_des * duration; // simple integrator model
    }
}


} // end namespace pxpincher
