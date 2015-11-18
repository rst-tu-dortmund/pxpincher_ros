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
 * Author: Christoph Rösmann
 *********************************************************************/

#include <pxpincher_lib/phantomx_interface.h>
#include <pxpincher_hardware/misc.h> // conversions between ticks and angles in radiant
#include <pxpincher_msgs/Relax.h>
#include <signal.h>
#include <sstream>
#include <algorithm>


namespace pxpincher
{
  
PhantomXControl::PhantomXControl()
{
}

PhantomXControl::~PhantomXControl()
{
  if (_joints_sub_spinner)
    _joints_sub_spinner->stop();
  if (_arm_action)
    _arm_action->cancelAllGoals();
}
  
void PhantomXControl::initialize()
{
  if (_initialized)
  {
      ROS_WARN("PhantomXControl class already initialized. Skipping new initialization...");
      return;
  }
    
  ros::NodeHandle n;
    
  // overwrite signal handler in order to allow cancellation of actions after pressing ctrl-c
  signal(SIGINT, pxpincher::PhantomXControl::phantomXSigHandler);
    
  // instantiate arm action client
  std::string arm_action_topic = "/arm_controller/follow_joint_trajectory";
  n.param("arm_action_topic", arm_action_topic, arm_action_topic);
  ROS_INFO("Waiting for arm action server to start.");
  _arm_action = make_unique<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(arm_action_topic, true);
  _arm_action->waitForServer(); // will wait for infinite time

  
  // instantiate gripper action client
  std::string gripper_action_topic = "/gripper_controller/gripper_cmd";
  n.param("gripper_action_topic", gripper_action_topic, gripper_action_topic);
  ROS_INFO("Arm action server found. Waiting for gripper action server to start.");
  _gripper_action = make_unique<actionlib::SimpleActionClient<control_msgs::GripperCommandAction>>(gripper_action_topic, true);
  _gripper_action->waitForServer(); // will wait for infinite time
  ROS_INFO("All action servers started.");
  
  // setup subscriber (get joint_states in a separate thread)
  _joints_sub_queue = new ros::CallbackQueue();
  n.setCallbackQueue(_joints_sub_queue);
  _joints_sub = n.subscribe("/joint_states", 1, &PhantomXControl::jointStateCallback, this);
  _joints_sub_spinner = make_unique<ros::AsyncSpinner>(1, _joints_sub_queue);
  ROS_ERROR_COND(!_joints_sub_spinner->canStart(),"Asynchronous spinner for receiving new joint state messages cannot be started.");
  _joints_sub_spinner->start();

  // setup joint names
  _joint_names_arm = {"arm_shoulder_pan_joint", // TODO : parse pxpincher_config
		      "arm_shoulder_lift_joint",
		      "arm_elbow_flex_joint",
		      "arm_wrist_flex_joint"};
  for (int i=0; i< _joint_names_arm.size(); ++i)
  {
    _map_joint_to_index[_joint_names_arm.at(i)] = i;
  }
   
  JointVector offsets; 
   
  // get joint information (angle limits and max speed)
  std::string arbotix_joints_ns = "/pxpincher/joints/"; // TODO config param
  for (int i=0; i<_joint_names_arm.size(); ++i)
  {
	std::string param_prefix = arbotix_joints_ns + _joint_names_arm[i]; 
        std::string min_angle_key = param_prefix + "/cwlimit";
	std::string max_angle_key = param_prefix + "/ccwlimit";
        std::string offset_key = param_prefix + "/offset";
	std::string max_speed_key = param_prefix + "/speed";
	
	if (!n.hasParam(min_angle_key) || !n.hasParam(max_angle_key) || !n.hasParam(offset_key) || !n.hasParam(max_speed_key))
	  ROS_ERROR("Could not find one or all of the following parameters:\n %s\n %s\n %s\n %s", min_angle_key.c_str(), max_angle_key.c_str(), offset_key.c_str() ,max_speed_key.c_str());
	
	n.getParam(min_angle_key, _joint_lower_bounds[i]);
	n.getParam(max_angle_key, _joint_upper_bounds[i]);
        n.getParam(offset_key, offsets[i]);
	n.getParam(max_speed_key, _joint_max_speeds[i]);
	
	// convert angles to radiant
	_joint_lower_bounds[i] = tick2rad( _joint_lower_bounds[i] - offsets[i] );
	_joint_upper_bounds[i] = tick2rad( _joint_upper_bounds[i] - offsets[i] );
	_joint_max_speeds[i] = tick2rads( _joint_max_speeds[i] );
  }     
  
  // setup services for relaxing the servos
  _joint_relax_service = n.serviceClient<pxpincher_msgs::Relax>( "/pxpincher/Relax" ); // TODO param

  
  // setup gripper
  _gripper_joint_name = "gripper_joint";
  double gripper_offset = 0;
  if (!n.hasParam(arbotix_joints_ns + _gripper_joint_name))
    ROS_ERROR("Could not find the specified gripper joint name: %s.", _gripper_joint_name.c_str());
  n.getParam(arbotix_joints_ns + _gripper_joint_name + "/cwlimit", _gripper_lower_bound);
  n.getParam(arbotix_joints_ns + _gripper_joint_name + "/ccwlimit", _gripper_upper_bound);
  n.getParam(arbotix_joints_ns + _gripper_joint_name + "/offset", gripper_offset);
//   n.param(arbotix_joints_ns + _gripper_joint_name + "/neutral", _gripper_neutral, 0.5*(_gripper_upper_bound+_gripper_lower_bound));
//   n.getParam(arbotix_joints_ns + _gripper_joint_name + "/max_speed", _gripper_max_speed);
  // convert angles to radiant
  _gripper_lower_bound = tick2rad( _gripper_lower_bound - gripper_offset );
  _gripper_upper_bound = tick2rad( _gripper_upper_bound - gripper_offset );
//   _gripper_neutral = normalize_angle_rad( deg_to_rad(_gripper_neutral) );
//   _gripper_max_speed = deg_to_rad( _gripper_max_speed );
  
  _map_joint_to_index[_gripper_joint_name] = 255; // set gripper joint idx to 255 in our map
  

  // Setup kinematic model
	
  // Wait for joint_state messages
  while (!_joint_values_received && ros::ok())
  {
      ROS_INFO_ONCE("Waiting for joint_states message...");
      _joints_sub_queue->callAvailable();
  }
  // Drive into default position
  ROS_INFO("Driving into default position in order to setup kinematic model...");
  setJointsDefault(0.2);

  
  // get transform: base to first joint 
  tf::StampedTransform transform;
  try
  {
    _tf.waitForTransform("/arm_base_link", "/arm_shoulder_pan_servo_link", ros::Time(0), ros::Duration(10.0) );
    _tf.lookupTransform("/arm_base_link", "/arm_shoulder_pan_servo_link", ros::Time(0), transform); // TODO: param
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  tf::transformTFToEigen(transform, _base_T_j1); // we refere to a class property here in order to store it for further access (since it is constant).
  _kinematics.setBaseToJoint1Transform(_base_T_j1);
  _j1_T_base = _base_T_j1.inverse();
  
  // get transform: first joint to second joint
  try
  {
    _tf.waitForTransform("/arm_shoulder_pan_servo_link", "/arm_shoulder_lift_servo_link", ros::Time(0), ros::Duration(10.0) );
    _tf.lookupTransform("/arm_shoulder_pan_servo_link", "/arm_shoulder_lift_servo_link", ros::Time(0), transform); // TODO: param
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  Eigen::Affine3d j1_T_j2;
  tf::transformTFToEigen(transform, j1_T_j2);
  _kinematics.setJoint1ToJoint2Transform(j1_T_j2);

  // get transform: second joint to third joint
  try
  {
    _tf.waitForTransform("/arm_shoulder_lift_servo_link", "/arm_elbow_flex_servo_link", ros::Time(0), ros::Duration(10.0) );
    _tf.lookupTransform("/arm_shoulder_lift_servo_link", "/arm_elbow_flex_servo_link", ros::Time(0), transform); // TODO: param
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  Eigen::Affine3d j2_T_j3;
  tf::transformTFToEigen(transform, j2_T_j3);
  _kinematics.setJoint2ToJoint3Transform(j2_T_j3);

  // get transform: forth joint to second joint
  try
  {
    _tf.waitForTransform("/arm_elbow_flex_servo_link", "/arm_wrist_flex_servo_link", ros::Time(0), ros::Duration(10.0) );
    _tf.lookupTransform("/arm_elbow_flex_servo_link", "/arm_wrist_flex_servo_link", ros::Time(0), transform); // TODO: param
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  Eigen::Affine3d j3_T_j4;
  tf::transformTFToEigen(transform, j3_T_j4);
  _kinematics.setJoint3ToJoint4Transform(j3_T_j4);

  // get transform: last joint to gripper
  try
  {
    _tf.waitForTransform("/arm_wrist_flex_servo_link", "/gripper_link", ros::Time(0), ros::Duration(10.0) );
    _tf.lookupTransform("/arm_wrist_flex_servo_link", "/gripper_link", ros::Time(0), transform); // TODO: param
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  Eigen::Affine3d arm_T_gripper;
  tf::transformTFToEigen(transform, arm_T_gripper);
  _kinematics.setJoint4ToGripperTransform(arm_T_gripper);
  
  // notify the kinematics model about lower and upper joint limits
  _kinematics.setLowerAndUpperJointLimits(_joint_lower_bounds, _joint_upper_bounds);
  
  _initialized = true;
  ROS_INFO("Initialization completed.");
}

void PhantomXControl::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(_joints_mutex);
  for (int i=0; i<msg->name.size(); ++i)
  {
    int joint_idx;
    try
    {
        joint_idx = _map_joint_to_index.at(msg->name[i]); // throws except. if key is not included
    }
    catch (std::out_of_range)
    {
        // Everything ok since we've already specified our mapping table in the initialize method.
        // Just ignore / skip states of other joints...
        continue;
    }
    if (joint_idx==255) // this must be our gripper! (see initialze())
        _gripper_value = msg->position[i];
    else
    {
        _joint_angles[joint_idx] = msg->position[i];
         if (!_joint_values_received)
           _joint_values_received = true;
    }
  }
}

void PhantomXControl::getJointAngles(Eigen::Ref<JointVector> values_out)
{
  std::lock_guard<std::mutex> lock(_joints_mutex);
  values_out = _joint_angles;
}

void PhantomXControl::getJointAngles(std::vector<double>& values_out)
{
  std::lock_guard<std::mutex> lock(_joints_mutex);
  values_out.assign(_joint_angles.data(), _joint_angles.data()+_joint_angles.rows());
}

JointVector PhantomXControl::getJointAngles()
{
    JointVector q;
    getJointAngles(q);
    return q;
}

void PhantomXControl::getJointVelocities(Eigen::Ref<JointVector> velocities_out)
{
  std::lock_guard<std::mutex> lock(_joints_mutex);
  velocities_out = _joint_velocities;
}

double PhantomXControl::getSlowestMaxSpeed() const
{
  return _joint_max_speeds.minCoeff();
}

  
void PhantomXControl::setJointsDefault(const ros::Duration& duration, bool blocking)
{
  setJoints(JointVector::Zero(), duration, false, blocking);
}
  
void PhantomXControl::setJointsDefault(double speed, bool blocking)
{
  setJoints(JointVector::Zero(), speed, false, blocking);
}  
  
void PhantomXControl::setJoints(const Eigen::Ref< const JointVector>& values, const ros::Duration& duration, bool relative, bool blocking)
{
  trajectory_msgs::JointTrajectoryPoint state;
  state.positions.assign(values.data(), values.data()+values.rows());
  state.time_from_start = duration;
  setJoints(state, relative, blocking);  
}

void PhantomXControl::setJoints(const std::vector<double>& values, const ros::Duration& duration, bool relative, bool blocking)
{
  Eigen::Map<const JointVector> values_map(values.data());
  setJoints(values_map, duration, relative, blocking);
}

void PhantomXControl::setJoints(const Eigen::Ref<const JointVector>& values, double speed, bool relative, bool blocking)
{
  ROS_ASSERT_MSG(speed>0, "You cannot command a zero velocity.");
//   ROS_ASSERT_MSG(_initialized, "Setting new joints with a desired velocity requires querying current joint values, that is only possible after initialization is completed.");
  
  JointVector current_states;  
  getJointAngles(current_states); // we need to copy here, rather accessing '_joint_angles' directly,
				  // since receiving new states is multi threaded.
  
  // get maximum absolute angle difference (TODO: do we need to normalize the angles before taking abs()?)
  double max_diff = (values - current_states).cwiseAbs().maxCoeff();

  if (max_diff<0.001)
      return; // we are already there...

  // get time corresponding to the distace max_diff and the given speed value
  // assume a constant velocity: phi=omega*t
  double duration = max_diff/speed;
  
  if (duration<0)
  {
    ROS_ERROR("PhantomXControl::setJoints(): obtained an invalid (negative) duration. Cannot set new joint values.");
    return;
  }
  setJoints(values,ros::Duration(duration), relative, blocking);
}
  
void PhantomXControl::setJoints(const std::vector<double>& values, double speed, bool relative, bool blocking)
{
  Eigen::Map<const JointVector> values_map(values.data());
  setJoints(values_map, speed, relative, blocking);
}  
  
  
void PhantomXControl::setJoints(const Eigen::Ref<const JointVector>& values, const Eigen::Ref<const JointVector>& speed, bool relative, bool blocking)
{
    JointVector current_states;  
    getJointAngles(current_states); // we need to copy here, rather accessing '_joint_angles' directly,
                                    // since receiving new states is multi threaded.
    
    trajectory_msgs::JointTrajectory traj;
    createP2PTrajectoryWithIndividualVel(current_states, values, speed, traj);
    setJointTrajectory(traj, blocking);
}
  
void PhantomXControl::setJoints(const std::vector<double>& values, const std::vector<double>& speed, bool relative, bool blocking)
{
  Eigen::Map<const JointVector> values_map(values.data());
  Eigen::Map<const JointVector> vel_map(speed.data());
  setJoints(values_map, vel_map, relative, blocking);
}
  

void PhantomXControl::setJoints(const trajectory_msgs::JointTrajectoryPoint& joint_state, bool relative, bool blocking)
{
  bool sync_duration_mode = joint_state.time_from_start.sec!=0 || joint_state.time_from_start.nsec!=0;
  if ( (sync_duration_mode && !joint_state.velocities.empty()) || (!sync_duration_mode && joint_state.velocities.empty()) )
  {
    ROS_ERROR("PhantomXControl::setJoints(): you must either specify a total duration (time_from_start) or individual velocities. Do not choose both.");
    return;
  }
  
  // create trajectory to the single goal
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = _joint_names_arm;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.points.push_back(joint_state);
  
  trajectory_msgs::JointTrajectoryPoint& new_state = goal.trajectory.points.front(); // get reference for further calculations
                                                                                     // since joint_state argument is read-only, we modify it here.
  
  // Get current joint angles 
  JointVector current_states;  
  getJointAngles(current_states); // we need to copy here, rather accessing '_joint_angles' directly,
                                  // since receiving new states is multi threaded.
  
  Eigen::Map<JointVector> values(new_state.positions.data()); // get an Eigen map for the position part for further computations.
  
  if (relative)
      values += current_states; // this also changes values in "goal"
  
  // check joint angle limits
  if (isExceedingJointLimits(values))
  {
    ROS_WARN_STREAM("PhantomXControl::setJoints(): cannot set new joint values since they exceed joint limits.\ndesired angles: ["
		    << values.transpose() << "]\nlower bounds: [" 
		    << _joint_lower_bounds.transpose() << "]\nupper bounds: [" << _joint_upper_bounds.transpose() << "]");
    return;
  }  
     
  // check velocities and/or duration before adding
  for (int i=0; i<new_state.velocities.size();++i)
  {
    new_state.velocities[i] = fabs(new_state.velocities[i]); // we consider only absolute velocities
    if (new_state.velocities[i] > _joint_max_speeds[i])
    {
      ROS_WARN("PhantomXControl::setJoints(): Velocity of joint %d exceeds limits. Bounding...", i);
      new_state.velocities[i] = _joint_max_speeds.coeffRef(i);
    }
  }
  if (sync_duration_mode)
  {
      // get maximum absolute angle difference
      
      double max_diff = (values - current_states).cwiseAbs().maxCoeff();

      // check and bound velocity
      // get maximum allowed duration (w.r.t. min of all max joint speeds)
      double duration_bounded = lower_bound(max_diff / getSlowestMaxSpeed(), new_state.time_from_start.toSec()); // phi = omega*t -> t = phi/omega
      ROS_WARN_COND(duration_bounded>new_state.time_from_start.toSec(), "PhantomXControl::setJoints(): desired speed is too high in order to drive all joints at this speed. Bounding...");
      new_state.time_from_start = ros::Duration(duration_bounded);
  }
  
  setJointTrajectory(goal, blocking);
}
 
  
void PhantomXControl::setJointVel(const Eigen::Ref<const JointVector>& velocities)
{
  // Bound velocities
  JointVector vel_bounded = pxpincher::bound(-_joint_max_speeds, velocities, _joint_max_speeds);  
    
  // Select lower and upper bound values for each joint as goal w.r.t. the direction of the velocity
  // For zero velocities stay at the current position
  JointVector current;
  getJointAngles(current);
  JointVector goal = (vel_bounded.array()==0).select( current, (vel_bounded.array()<0).select(_joint_lower_bounds,_joint_upper_bounds) );
  _collision_check_enabled = false; // Disable collision check, be careful!!
  setJoints(goal, vel_bounded, false, false); // we do not want to block in case of commanding velocities
}
  
void PhantomXControl::setJointVel(const std::vector<double>& velocities)
{
  Eigen::Map<const JointVector> vel_map(velocities.data());
  setJointVel(vel_map);
}  
  
  
void PhantomXControl::setJointTrajectory(trajectory_msgs::JointTrajectory& trajectory, bool blocking)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  
  setJointTrajectory(goal, blocking);
}

void PhantomXControl::setJointTrajectory(control_msgs::FollowJointTrajectoryGoal& trajectory, bool blocking)
{
  ROS_ASSERT_MSG(_arm_action, "PhantomXControl: class not initialized, call initialize().");
    
  // cancel any previous goals
  stopMoving();
  
  // verify trajectory and adjust speeds if necessary
  bool feasible = verifyTrajectory(trajectory.trajectory); // returns false if angle limits are exceeded or something goes wrong
  if (!feasible)
  {
    ROS_ERROR("The trajectory is not feasible. Printing trajectory now: ");
    printTrajectory(trajectory.trajectory);
    return; // Errors are printed in verifyTrajectory()
  }
  
  if (blocking)
    _arm_action->sendGoalAndWait(trajectory, ros::Duration(20));
  else
    _arm_action->sendGoal(trajectory);
}


bool PhantomXControl::isExceedingJointLimits(const Eigen::Ref<const JointVector>& joint_values)
{
  return (joint_values.array()<_joint_lower_bounds.array()).any() || (joint_values.array()>_joint_upper_bounds.array()).any();
}

bool PhantomXControl::checkSelfCollision(const Eigen::Ref<const JointVector>& joint_values)
{
  if (!_initialized) 
      return false; // we can only check after calling initialization, since we need a kinematic model
                    // but we return no collision, since inside the initialization only setJointsDefault is called
                    // that is admissable
             
  // We have some problems with velocity control here, that need to be fixed.
  // But for now we bypass this problem by deactivating the collision check in case of velocity control;
  // Velocity control is activated if all target joint values correspond to the bounds
  // WORKAROUND:
  int count_equal = 0;
  for (int i=0; i<joint_values.rows(); ++i)
  {
    if ( is_approx(joint_values.coeffRef(i), _joint_lower_bounds.coeffRef(i)) || is_approx(joint_values.coeffRef(i), _joint_upper_bounds.coeffRef(i)) || joint_values.coeffRef(i)==0)
      count_equal = count_equal +1;
  }
  if (count_equal == joint_values.rows())
    return false; // velocity control detected
  
  // get endeffector position
  Eigen::Affine3d  transform = kinematics().computeForwardKinematics(joint_values);
  // transform into the system of joint 1 since we know that point on the robot
  transform = _j1_T_base * transform;
  // the y axis of system1 corresponds to the z-axis of the base system
  // the z axis of system1 corresponds to the x-axis of the base system
  // the x axis of system1 corresponds to the y-axis of the base system
  
  // get the point in the z-x plane (z-x such that is right-handed)
  const Eigen::Vector3d& point = transform.translation();
  
  // now everything is expressed in j1-system:
  // front (positive z): z_c + 7cm
  // back (negative z): z_c - 13cm
  // left (positive x): x_c + 11cm
  // right (negative x): x_c - 11cm

  bool inside = point.z() > -0.13 && point.z() < 0.07 && point.x() > -0.11 && point.x() < 0.11; // TODO: param
  if (inside)
  {
      // check y axis (z axis of the base system)
      // it must be at least 10cm over the joint 1 (hardcoded, TODO: param)
      if (point.y() < 0.1)
          return true;
  }
  
  // now test the fourth joint
  transform = kinematics().computeForwardKinematics(joint_values,3);
  transform = _j1_T_base * transform;
  const Eigen::Vector3d& point2 = transform.translation();
  inside = point2.z() > -0.13 && point2.z() < 0.07 && point2.x() > -0.11 && point2.x() < 0.11;
  if (inside)
  {
      if (point2.y() < 0) // we reduce this number for the fourth joint to zero
          return true;
  }
  
  return false;
}

bool PhantomXControl::verifyTrajectory(trajectory_msgs::JointTrajectory& trajectory)
{
  // Get current joint angles 
  JointVector current_states;
  getJointAngles(current_states); // we need to copy here, rather accessing '_joint_angles' directly,
				  // since receiving new states is multi threaded.
  
  // Store previous angle positions as Eigen type (required inside the loop)
  Eigen::Map<JointVector> prev_pos(current_states.data());  
 
  int pt_idx = 0;
  for (trajectory_msgs::JointTrajectoryPoint& state : trajectory.points)
  {

      Eigen::Map<const JointVector> values(state.positions.data()); // get an Eigen map for the position part for further computations.
      
      // check joint angle limits
      if (isExceedingJointLimits(values))
      {
	ROS_WARN_STREAM("PhantomXControl::verifyTrajectory(): cannot set new joint values since they exceed joint limits at point " << pt_idx << ".\ndesired angles: ["
			<< values.transpose() << "]\nlower bounds: [" 
			<< _joint_lower_bounds.transpose() << "]\nupper bounds: [" << _joint_upper_bounds.transpose() << "]");
	return false;
      }  
      
      // check velocities and/or duration before adding
      for (int i=0; i<state.velocities.size();++i)
      {
	state.velocities[i] = fabs(state.velocities[i]); // we consider only absolute velocities
	if (state.velocities[i] > _joint_max_speeds[i])
	{
	  ROS_WARN("PhantomXControl::verifyTrajectory(): Velocity of joint %d at point %d exceeds limits. Bounding...", i, pt_idx);
	  state.velocities[i] = _joint_max_speeds.coeffRef(i);
	}
      }
      
      // check transition mode (synchronous transition or individual velocities
      bool sync_duration_mode = state.time_from_start.sec!=0 || state.time_from_start.nsec!=0;
      if (sync_duration_mode)
      {
	  // get maximum absolute angle difference 
	  double max_diff = (values - prev_pos).cwiseAbs().maxCoeff();

	  // check and bound velocity
	  // get maximum allowed duration (w.r.t. min of all max joint speeds)
	  double duration_bounded = lower_bound(max_diff / getSlowestMaxSpeed(), state.time_from_start.toSec()); // phi = omega*t -> t = phi/omega
	  ROS_WARN_COND(duration_bounded>state.time_from_start.toSec(), "PhantomXControl::verifyTrajectory(): desired speed at point %d is too high in order to drive all joints at this speed. Bounding...", pt_idx);
	  state.time_from_start = ros::Duration(duration_bounded);
      }
      
      // check self-collision
      if (_collision_check_enabled && checkSelfCollision(values))
      {
        ROS_ERROR("PhantomXControl::verifyTrajectory(): Self-collision detected at point %d.", pt_idx);
        return false;
      }
      

      
      // Store pos vector for the subsequent iteration (using C++ placement new operator)
      new (&prev_pos) Eigen::Map<JointVector>(state.positions.data());
      
      ++pt_idx;
  }
  // set check enabled for next time
  _collision_check_enabled = true;
  return true;
}
  
  
void PhantomXControl::stopMoving()
{
    _arm_action->cancelAllGoals();
    ros::Duration(0.001).sleep(); // we wait for a small time, since cancelling goal is scheduled in a separate thread.
				  // But this small duration does not guarantee, that a goal immediately set after calling stop stopMoving
				  // is actually excecuted.
}
  
  
void PhantomXControl::getEndeffectorState(Eigen::Affine3d& base_T_gripper)
{
    // get tf::Transform
    tf::StampedTransform tf_transform;
    getEndeffectorState(tf_transform);
    tf::transformTFToEigen(tf_transform, base_T_gripper);
}
  
void PhantomXControl::getEndeffectorState(tf::StampedTransform& base_T_gripper)
{
    try
    {
      _tf.waitForTransform("/arm_base_link", "/gripper_link", ros::Time(0), ros::Duration(10.0) );
      _tf.lookupTransform("/arm_base_link", "/gripper_link", ros::Time(0), base_T_gripper); // TODO: param
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }
}

void PhantomXControl::setEndeffectorPose(const Eigen::Affine3d& desired_pose, const ros::Duration& duration, bool relative, bool blocking)
{
    JointVector joint_angles;
    getJointAngles(joint_angles); // use current joint angles as initialization
    bool success = false;
    if (relative)
    {
      Eigen::Affine3d current_tcp = kinematics().computeForwardKinematics(joint_angles);
      success = kinematics().computeInverseKinematics( desired_pose * current_tcp , joint_angles);
    }
    else
      success = kinematics().computeInverseKinematics(desired_pose, joint_angles);
    if (success)
      setJoints(joint_angles, duration, false, blocking);
}

void PhantomXControl::setEndeffectorPose(const Eigen::Affine3d& desired_pose, double speed, bool relative, bool blocking)
{
    JointVector joint_angles;
    getJointAngles(joint_angles); // use current joint angles as initialization
    bool success = false;
    if (relative)
    {
      Eigen::Affine3d current_tcp = kinematics().computeForwardKinematics(joint_angles);
      success = kinematics().computeInverseKinematics( desired_pose * current_tcp , joint_angles);
    }
    else
      success = kinematics().computeInverseKinematics(desired_pose, joint_angles);
    if (success)
      setJoints(joint_angles, speed, false, blocking);
}

bool PhantomXControl::setEndeffectorPose(const Eigen::Ref<const Eigen::Vector3d>& desired_xyz, double desired_pitch, const ros::Duration& duration, bool relative, bool blocking)
{
    JointVector joint_angles;
    getJointAngles(joint_angles); // use current joint angles as initialization
    bool success = false;
    if (relative)
    {
      Eigen::Affine3d current_tcp = kinematics().computeForwardKinematics(joint_angles);
      success = kinematics().computeInverseKinematics(createPoseFromPosAndPitch(desired_xyz, desired_pitch) * current_tcp, joint_angles);
    }
    else
      success = kinematics().computeInverseKinematics(desired_xyz, desired_pitch, joint_angles);
    if (success)
      setJoints(joint_angles, duration, false, blocking);
}

bool PhantomXControl::setEndeffectorPose(const Eigen::Ref<const Eigen::Vector3d>& desired_xyz, double desired_pitch, double speed, bool relative, bool blocking)
{
    JointVector joint_angles;
    getJointAngles(joint_angles); // use current joint angles as initialization
    bool success = false;
    if (relative)
    {
      Eigen::Affine3d current_tcp = kinematics().computeForwardKinematics(joint_angles);
      success = kinematics().computeInverseKinematics(createPoseFromPosAndPitch(desired_xyz, desired_pitch) * current_tcp, joint_angles);
    }
    else
      success = kinematics().computeInverseKinematics(desired_xyz, desired_pitch, joint_angles);
    if (success)
      setJoints(joint_angles, speed, false, blocking);
}

bool PhantomXControl::setEndeffectorPose(const std::vector<double>& desired_xyz, double desired_pitch, double speed, bool relative, bool blocking)
{
    setEndeffectorPose(desired_xyz, desired_pitch, speed, relative, blocking);
}


void PhantomXControl::getJacobian(RobotJacobian& jacobian)
{
    JointVector joint_angles;
    getJointAngles(joint_angles);
    kinematics().computeJacobian(joint_angles, jacobian);
}

void PhantomXControl::getJacobianReduced(RobotJacobianReduced& jacobian4d)
{
    JointVector joint_angles;
    getJointAngles(joint_angles);
    kinematics().computeJacobianReduced(joint_angles, jacobian4d);
}


void PhantomXControl::setGripperJoint(int percent_open, bool blocking)
{
  double joint_value = _gripper_lower_bound + 0.01*double(percent_open)*( _gripper_upper_bound-_gripper_lower_bound );
  setGripperRawJointAngle(joint_value, blocking);
}


void PhantomXControl::setGripperRawJointAngle(double joint_value, bool blocking)
{
  control_msgs::GripperCommandGoal goal;
  goal.command.position = joint_value;
    
  if (blocking)
    _gripper_action->sendGoalAndWait(goal, ros::Duration(5));
  else
    _gripper_action->sendGoal(goal);
}

double PhantomXControl::getGripperJointAngle()
{
    std::lock_guard<std::mutex> lock(_joints_mutex);
    return _gripper_value;
}



void PhantomXControl::createP2PTrajectoryWithIndividualVel(const std::vector<double>& start_conf,
                                                           const std::vector<double>& goal_conf,
                                                           const std::vector<double>& speed,
                                                           trajectory_msgs::JointTrajectory& trajectory)
{
    Eigen::Map<const JointVector> start_map(start_conf.data());
    Eigen::Map<const JointVector> goal_map(goal_conf.data());
    Eigen::Map<const JointVector> speed_map(speed.data());
    createP2PTrajectoryWithIndividualVel(start_map, goal_map, speed_map, trajectory);
}


void PhantomXControl::createP2PTrajectoryWithIndividualVel(const Eigen::Ref<const JointVector>& start_conf,
                                                           const Eigen::Ref<const JointVector>& goal_conf,
                                                           const Eigen::Ref<const JointVector>& speed,
                                                           trajectory_msgs::JointTrajectory& trajectory)
{
    // Limit absolute nonzeros to a minimum value
    JointVector new_speed = (speed.array()!=0).select(speed.cwiseAbs(), 1e-2); // only the absolute value of the speed, we will determine the sign later
    
    JointVector diff = goal_conf-start_conf;
        
    // Determine speed sign
    new_speed = (diff.array()<0).select(-new_speed, new_speed);
    
    // Let's determine the durations for each transition
    JointVector durations = diff.cwiseQuotient(new_speed).cwiseAbs();
    
    // Limit durations
    durations = (durations.array()<20.0).select(durations, 20.0); // limit to 20 seconds (which is actually really long for velocity control)
    
    double max_duration = durations.maxCoeff();

    // sort and keep track about indices
    std::vector<int> indices(durations.rows());
    std::iota(indices.begin(), indices.end(), 0); // Fill with 0,1,...,n

    // now sort indices based on durations (from short to long)
    std::sort(indices.begin(), indices.end(),
              [&durations](std::size_t i, std::size_t j) {return durations.coeffRef(i) < durations.coeffRef(j);});
    
//     ROS_INFO_STREAM("indices:" << (Eigen::Map<const Eigen::Matrix<int,-1,1> >(indices.data(),durations.rows())).transpose());
    
    // init trajectory
    int n_joints = durations.rows();
    trajectory.header.stamp = ros::Time::now();
    trajectory.joint_names = getJointNames();  
    trajectory.points.reserve(indices.size());
    // create waypoints for intermediate goals (for each joint a single goal with a temporal distance of the specified duration)
    // fill in all durations and init joint position sizes
    for (int i=0; i<indices.size(); ++i)
    {
        if (durations.coeffRef(indices[i]) > 1e-2) // skip joint, if duration is approx 0
        {
            trajectory.points.emplace_back();
            trajectory.points.back().time_from_start = ros::Duration(durations.coeffRef(indices[i])); 
            trajectory.points.back().positions.resize(n_joints);
            trajectory.points.back().velocities.resize(n_joints);
        }
    }

    int n_points = (int) trajectory.points.size();
    int n_skipped = (int) indices.size() - n_points;
//         ROS_INFO_STREAM("durations: " << durations.transpose() << " n_skipped: " << n_skipped);
    // iterate single components to initialize joint values
    for (int i=0; i<n_joints; ++i)
    {
        int curr_joint = indices[i];
        // iterate waypoints
        for (int j=0; j<n_points; ++j)
        {	
            trajectory_msgs::JointTrajectoryPoint& pt = trajectory.points[j];
	    if (j==0)
            {
                if (fabs(diff.coeffRef(curr_joint))>1e-2)
                {
                    pt.positions[curr_joint] = start_conf[curr_joint] +  trajectory.points[j].time_from_start.toSec() * new_speed[curr_joint];
                    pt.velocities[curr_joint] = new_speed[curr_joint];
                }
                else
                {
                    pt.positions[curr_joint] = start_conf[curr_joint];
                    pt.velocities[curr_joint] = 0.0;
                }
            }
	    else 
            {
                trajectory_msgs::JointTrajectoryPoint& last_pt = trajectory.points[j-1];
                if ( !is_approx(last_pt.positions[curr_joint], goal_conf[curr_joint], 1e-2) )
                {
                    pt.positions[curr_joint] = last_pt.positions[curr_joint] + (pt.time_from_start - last_pt.time_from_start).toSec() * new_speed[curr_joint];
                    pt.velocities[curr_joint] = new_speed[curr_joint];
                }     
                else
                {
                    pt.positions[curr_joint] = goal_conf[curr_joint];
                    pt.velocities[curr_joint] = 0.0;
                }
            }
        }    
    }
}


void PhantomXControl::createQuinticPolynomialJointTrajectory(const Eigen::Ref<const JointVector>& q0, const Eigen::Ref<const JointVector>& qf,
                                                              unsigned int n, double dt, trajectory_msgs::JointTrajectory& trajectory, 
                                                              const JointVector* v0, const JointVector* vf)
{
    // I decided to make a copy of the joint trajectory here in order to avoid crazy template versions of createQuintic...
    using JointVecContainer = std::vector<JointVector, Eigen::aligned_allocator<JointVector>>;
    JointVecContainer q_traj;
    createQuinticPolynomialTrajectory<JointVector, JointVecContainer>(q0, qf, n, q_traj, v0, vf);
    // now copy
    trajectory.points.clear();
    trajectory.joint_names = _joint_names_arm;
    trajectory.header.stamp = ros::Time::now();
    double time = 0;
    for (const JointVector& q : q_traj)
    {
        trajectory_msgs::JointTrajectoryPoint point;
        point.time_from_start = ros::Duration(time);
        convertEigenVectorToSTL<double>( q, point.positions );
        trajectory.points.push_back(point);
        time += dt;
    }
}



bool PhantomXControl::relaxServos(bool relaxed)
{
    bool ret_val = true;
    
    pxpincher_msgs::Relax relax;
    
    relax.request.relaxed = relaxed;
    
        if (_joint_relax_service.call(relax))
        {
            ret_val = relax.response.success;
            ROS_WARN_COND(!ret_val, "Not all servos could be relaxed");
        }
        else
        {
            ret_val = false;
            ROS_WARN("Failed to relax servos");
        }
    
    return ret_val;    
}

bool PhantomXControl::testKinematicModel()
{

  bool retval = true;
  
  // Default position
  JointVector values;
  values.setZero();
  retval = retval && testKinematicModel(values);
     
  // Pos 1
  values << -M_PI/2, 0, 0, 0;
  retval = retval && testKinematicModel(values);
  
  // Pos 2
  values << 0, M_PI/3, 0, 0;
  retval = retval && testKinematicModel(values);
  
  // Pos 3
  values << 0, 0, M_PI/2, 0;
  retval = retval && testKinematicModel(values);
  
  // Pos 4
  values << 0, 0, 0, -M_PI/2;
  retval = retval && testKinematicModel(values);
  
  // Pos 5
  values << M_PI/2, -M_PI/3, M_PI/5, -M_PI/2;
  retval = retval && testKinematicModel(values);
  
  if (retval)
    ROS_INFO_STREAM("testKinematicModel successfully completed.");
  else
    ROS_INFO_STREAM("testKinematicModel completed with errors.");
  
  return retval;
}
  
bool PhantomXControl::testKinematicModel(const Eigen::Ref<const JointVector>& joint_values)
{
  bool retval = true;

  ROS_INFO_STREAM("Testing joint configuration: q=[" << joint_values.transpose() << "]...");
  
  // command robot
  setJoints(joint_values,ros::Duration(5));
  // get current state
  Eigen::Affine3d real;
  getEndeffectorState(real);
  // compute forward kinematics using the model class
  Eigen::Affine3d model = kinematics().computeForwardKinematics(joint_values);
  bool trans_flag = model.translation().isApprox( real.translation(), 0.1 );
  if ( !trans_flag)
  {
    ROS_WARN_STREAM("Translation part does not match\nreal: [" << real.translation().transpose()
                    << "] model: [" << model.translation().transpose() << "]");
    retval = false;
  }
  Eigen::Quaterniond qreal = Eigen::Quaterniond(real.rotation());
  Eigen::Quaterniond qmodel = Eigen::Quaterniond(model.rotation());
  bool rot_flag = qmodel.isApprox( qreal, 0.1 ) || qmodel.isApprox( Eigen::Quaterniond(-qreal.w(),-qreal.x(),-qreal.y(),-qreal.z()), 0.1 );
  if ( !rot_flag )
  {
    ROS_WARN_STREAM("Rotation part does not match\nreal: [" << qreal.w() << "," << qreal.vec().transpose()
             << "] model: [" << qmodel.w() << "," << qmodel.vec().transpose() << "]");
    tf::Quaternion tf_qmodel;
    tf::quaternionEigenToTF(qmodel,tf_qmodel);
    ROS_WARN_STREAM("yaw: " << tf::getYaw(tf_qmodel));
    retval = false;
  }
  ROS_INFO("Test finished.");
  return retval;  
}

  
void PhantomXControl::printTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  if (trajectory.points.empty())
  {
    ROS_WARN("PhantomXControl::printTrajectory() - cannot print empty trajectory.");
    return;
  }
  
  int no_joints = trajectory.points.front().positions.size();
  int no_points = trajectory.points.size();
  int no_vel = trajectory.points.front().velocities.size();

  if (no_joints==0)
    ROS_INFO_STREAM("Position profile empty.");
  if (no_vel==0)
    ROS_INFO_STREAM("Velocity profile empty.");  
  
  ROS_INFO_STREAM(""); // add  blank line
  
  // Position:
  // print header
  std::stringstream header;
  for (int i=0; i<no_points; ++i)
  {
      header << "p" << i << "\t";
      ROS_ASSERT_MSG(trajectory.points[i].positions.size() == no_joints, "Number of joints is not consistent for all trajectory points. Before: %d, now: %d",
                   (int) no_joints, (int) trajectory.points[i].positions.size());
      ROS_ASSERT_MSG(trajectory.points[i].velocities.size() == no_vel, "Number of joint velocities is not consistent for all trajectory points. Before: %d, now: %d",
                   (int) no_vel, (int) trajectory.points[i].velocities.size());
  }
  ROS_INFO_STREAM(header.str());
    
  for (int i=0; i<no_joints; ++i)
  {
    std::stringstream row;
    for (int j=0; j<no_points; ++j)
        row << std::fixed << std::setprecision(2) << trajectory.points[j].positions[i] << "\t";
    ROS_INFO_STREAM(row.str());
  }
  
  ROS_INFO_STREAM(""); // add  blank line
  
  // Velocity:
  // print header
  header.str("");
  header.clear();
  for (int i=0; i<no_points; ++i)
      header << "v" << i << "\t";
  ROS_INFO_STREAM(header.str());
    
  for (int i=0; i<no_vel; ++i)
  {
    std::stringstream row;
    for (int j=0; j<no_points; ++j)
      row << std::fixed << std::setprecision(2) << trajectory.points[j].velocities[i] << "\t";
    ROS_INFO_STREAM(row.str());
  }
  
  ROS_INFO_STREAM(""); // add  blank line
  
  // Time:
  // print header
  header.str("");
  header.clear();
  for (int i=0; i<no_points; ++i)
      header << "t" << i << "\t";
  ROS_INFO_STREAM(header.str());
  std::stringstream row;
  for (int j=0; j<no_points; ++j)
    row << std::fixed << std::setprecision(2) << trajectory.points[j].time_from_start.toSec() << "\t";
  ROS_INFO_STREAM(row.str());
}
  
  
} // end namespace pxpincher
