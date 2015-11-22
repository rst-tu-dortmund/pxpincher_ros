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

namespace pxpincher
{

Simulation::Simulation()
{
}

void Simulation::addJoint(UBYTE id, const std::string& name, int default_pos, int lower_bound, int upper_bound)
{
    joint_data_[id] = JointData(name, default_pos, 0, default_pos, lower_bound, upper_bound);
}

void Simulation::clearJoints()
{
  joint_data_.clear();
}

void Simulation::setGoalPosition(UBYTE id, int position)
{
  boost::mutex::scoped_lock lock(data_mutex_);
  try
  {
    joint_data_.at(id).goal = position;
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
	  elem.position_ = data.pos;
	  elem.speed_ = data.speed;
	  elem.temperature_ = 0;
	  elem.voltage_ = 0;
	}
	catch (const std::out_of_range& oor)
	{
	  ROS_ERROR_STREAM("Simulation::readServoStatus(): invalid id: " << (int) elem.id_);
	}
    }
    
}

bool Simulation::isMoving()
{
  return false;
}

sensor_msgs::JointState Simulation::performSimulationStep(double duration)
{
  /*
    std::vector<std::string> names = {"J1","J2","J3","J4","J5"};

    for(int i = 0; i < qDots_.size(); ++i){
        offsets_[i] += qDots_[i]*duration;
    }


    currentState_.header.stamp = ros::Time::now();
    currentState_.name = names;
    currentState_.position = offsets_;
    currentState_.velocity = qDots_;

    return currentState_;
    */
  return sensor_msgs::JointState();
}

sensor_msgs::JointState Simulation::getCurrentState()
{
   // return currentState_;
  return sensor_msgs::JointState();
}

} // end namespace pxpincher
