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

#ifndef PHANTOMX_INTERFACE_H_
#define PHANTOMX_INTERFACE_H_

// stl
#include <memory>
#include <mutex>
#include <numeric>
#include <cmath>

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/GripperCommandAction.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// own stuff
#include <phantomx_lib/types.h>
#include <phantomx_lib/misc.h>
#include <phantomx_lib/kinematics.h>



namespace phantomx
{
  

/**
 * @class PhantomXControl
 * @brief This class provides a common interface to communicate with the PhantomX Pincher Arm robot.
 * 
 * This class does not implement a low-level hardware API in order to send actual robot commands to the 
 * Arbotix board on the robot. Rather the arbotix-ros package is utilized for this job.
 * The PhantomXControl class provides a slighly more high-level API to set velocity, position and trajectory following commands.
 * It is not expected to replace MoveIt (which is also compatible with the PhantomX), but this package is intended for a
 * much simpler and guided usage on the control level suited for education and research.
 */
class PhantomXControl
{
public:
  
  /**
   * @brief Construct the class
   */
  PhantomXControl();
  
  /**
   * @brief Destruct the class
   */
  virtual ~PhantomXControl();
  
  /**
  * @brief Initializes the class
  */
  void initialize(); 
  
  /** @name Receive joint state information */
  //@{
  
  /**
   * @brief Reveive joint names according to the ordering in the JointVector
   * @return read-only reference to the vector containing joint name strings
   */
  const std::vector<std::string>& getJointNames() const {return _joint_names_arm;}
  
  /**
   * @brief Get current joint angles 
   * @param[out] values_out Eigen Matrix/Vector type q=[q1,q2,q3,q4]^T (doubles) which the joint angles are written to.
   */
  void getJointAngles(Eigen::Ref<JointVector> values_out);
  
  /**
   * @brief Get current joint angles 
   * @param[out] values_out std::vector< double > which the joint angles are written to.
   */
  void getJointAngles(std::vector<double>& values_out);
  
  /**
   * @brief Get current joint angles (copy version)
   * @param[out] values_out Eigen Matrix/Vector type q=[q1,q2,q3,q4]^T (doubles) which the joint angles are written to.
   */
  JointVector getJointAngles();
  
  /**
   * @brief Get current joint velocities 
   * @param[out] values_out Eigen Matrix/Vector type q=[q1,q2,q3,q4]^T (doubles) which the joint velocities are written to.
   */
  void getJointVelocities(Eigen::Ref<JointVector> velocities_out);
  

  /**
   * @brief Get the slowest max speed of the set of all joints
   * @todo this could be calculated once and stored as class property
   * @return min( qdot1_max, qdot2_max, qdot3_max, qdot4_max )
   */  
  double getSlowestMaxSpeed() const;
  
  //@}
  
  
  /** @name Command joint angles */
  //@{
    
  /**
   * @brief Set joints to the default position q=[0,0,0,0]^T
   * @param duration duration for the transition to the new joint state.
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJointsDefault(const ros::Duration& duration=ros::Duration(5), bool blocking=true);
  
  /**
   * @brief Set joints to the default position q=[0,0,0,0]^T
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the default position.
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */ 
  void setJointsDefault(double speed, bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * @param values vector of new joint angles q=[q1,q2,q3,q4]^T
   * @param duration duration for the transition to the new joint state.
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJoints(const Eigen::Ref<const JointVector>& values, const ros::Duration& duration=ros::Duration(5), bool relative=false, bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * @param values std::vector< double > of new joint angles q=[q1,q2,q3,q4]^T
   * @param duration duration for the transition to the new joint state.
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   * @remarks This overload accepts initializer-lists: <code> setJoints({0.3, 0, 0, 0}) </code>
   */
  void setJoints(const std::vector<double>& values, const ros::Duration& duration=ros::Duration(5), bool relative=false, bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * @param values vector of new joint angles q=[q1,q2,q3,q4]^T
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the final state.
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJoints(const Eigen::Ref<const JointVector>& values, double speed, bool relative=false, bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * @param values std::vector< double > of new joint angles q=[q1,q2,q3,q4]^T
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the final state.
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   * @remarks This overload accepts initializer-lists: <code> setJoints({0.3, 0, 0, 0}) </code>
   */
  void setJoints(const std::vector<double>& values, double speed, bool relative=false, bool blocking=true);
  
  /**
   * @brief Command new joint angles by commanding individual joint velocities
   * @param values vector of new joint angles q=[q1,q2,q3,q4]^T
   * @param speed vector containing joint velocities qdot = [omega1, omega2, omega3, omega4]^T
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJoints(const Eigen::Ref<const JointVector>& values, const Eigen::Ref<const JointVector>& speed, bool relative=false, bool blocking=true);
  
  /**
   * @brief Command new joint angles by commanding individual joint velocities
   * @param values std::vector< double > of new joint angles q=[q1,q2,q3,q4]^T
   * @param speed std::vector< double > containing joint velocities qdot = [omega1, omega2, omega3, omega4]^T
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   * @remarks This overload accepts initializer-lists: <code> setJoints({0.3, 0, 0, 0}, {0.1, 0, 0, 0}) </code>
   */
  void setJoints(const std::vector<double>& values, const std::vector<double>& speed, bool relative=false, bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * 
   * You can either specify a synchronous transition by setting \c joint_state.time_from_start to a given duration,
   * or you can set each joint velocity individual by providing \c joint_state.velocities.
   * @param joint_state trajectory_msgs::JointTrajectoryPoint type
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJoints(const trajectory_msgs::JointTrajectoryPoint& joint_state, bool relative=false, bool blocking=true);
  
  //@}
  
  /** @name Command joint velocities */
  //@{
  
  /**
   * @brief Command robot by specifying joint velocities.
   * 
   * Be careful, this call is a non-blocking call and the robot will
   * hold the specified velocities until joint limits are exceeded or
   * a new command is sent.
   * Specify additional sleep commands with ros::Duration(s).sleep() or ros::Rate.
   * @warning Some collision checks are disabled, cancel your program if a possible crash seems to be occure...
   * @param velocities vector of desired joint velocities
   */
  void setJointVel(const Eigen::Ref<const JointVector>& velocities);
  
  /**
   * @brief Command robot by specifying joint velocities.
   * 
   * Be careful, this call is a non-blocking call and the robot will
   * hold the specified velocities until joint limits are exceeded or
   * a new command is sent.
   * Specify additional sleep commands with ros::Duration(s).sleep() or ros::Rate.
   * @remarks This overload accepts initializer-lists: <code> setJointVel({0.1, 0.1, 0, 0}) </code>
   * @warning Some collision checks are disabled, cancel your program if a possible crash seems to be occure...
   * @param velocities std::vector< double > of desired joint velocities
   */
  void setJointVel(const std::vector<double>& velocities);
  
  //@}
  
  
  /** @name Follow joint trajectory */
  //@{
  
  /**
   * @brief Follow a given joint trajectory.
   * 
   * The trajectory should contain a sequence of positions. For each position the joint angles needs to be
   * matched with the joint names. You can obtain joint names from the getJointAngles() method.
   * @code
   *    // Example trajectory:
   * 	trajectory_msgs::JointTrajectory trajectory;
   *	trajectory.header.stamp = ros::Time::now(); // we want to execute the trajectry now
   *	trajectory.joint_names = robot.getJointNames(); // robot denotes a PhantomXControl object instance.
   *	trajectory.points.resize(2); // we want two goal states
   *	// intermediate goal:
   *	robot.getJointAngles( trajectory.points[0].positions ); // set intermediate goal to current values
   *	trajectory.points[0].positions[0] = -M_PI/3; // set first joint to -pi/3
   *	trajectory.points[0].time_from_start = ros::Duration(4); // 2 seconds for the transition
   *	// final goal:
   *	trajectory.points[1].positions.resize(4, 0); // initialize all 4 joints to 0
   *	trajectory.points[1].positions[2] = M_PI/2; // set joint 2 to pi/2
   *	trajectory.points[1].time_from_start = ros::Duration(5); // 3 seconds for the transition
   * @endcode
   * 
   * @param trajectory JointTrajectory message containing the new positions (non-const, since velocity could be limited: warnings appear)
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJointTrajectory(trajectory_msgs::JointTrajectory& trajectory, bool blocking=true);
  
  /**
   * @brief Follow a given joint trajectory
   * 
   * See setJointTrajectory(const trajectory_msgs::JointTrajectory& trajectory, bool blocking) for details on
   * how to specify the underlying trajectory.
   * @param trajectory Trajectory given in FollowJointTrajectoryGoal format (non-const, since velocity could be limited: warnings appear)
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing
   */
  void setJointTrajectory(control_msgs::FollowJointTrajectoryGoal& trajectory, bool blocking=true);
  
  //@}
  
  
  /** @name Kinematics and Differential Kinematics */
  //@{
  
  /**
   * @brief Get the transformation matrix to the gripper(TCP) expressed in the robot base System
   * @param[out] base_T_gripper Eigen::Affine3d transformation type
   */
  void getEndeffectorState(Eigen::Affine3d& base_T_gripper);
  
  /**
   * @brief Get the transformation matrix to the gripper(TCP) expressed in the robot base System
   * @param[out] base_T_gripper tf transformation type
   */
  void getEndeffectorState(tf::StampedTransform& base_T_gripper);
  
  
  /**
   * @brief Set the endeffector/gripper to a given pose w.r.t. to the robot base frame
   * 
   * Since the robot has only 4-DOF, all 6D poses cannot be represtend by a set of joint angles.
   * The underlying inverse kinematics tries to set the translation and the pitch angle of the desired pose.
   * Since we have only one degree of freedom to set a yaw angle, yaw-orientation of the gripper should 
   * coincide with the distance vector to the gripper. Otherwise a warning is displayed. The yaw angle is interesting
   * in case of the arm singularity in which infinite solutions appear for the first joint value (for the position-IK).
   * The implementation accounts for joint limits.
   * If no solution within the joint limits is found, the algorihm tries to set q1=q1+pi and solves the IK again
   * resulting in a similar gripper position but with switched sides (of the gripper itself).
   * @param desired_pose Desired 6D poses but with some limitations mentioned above.
   * @param duration duration for the transition to the new joint state.
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */ 
  void setEndeffectorPose(const Eigen::Affine3d& desired_pose, const ros::Duration& duration=ros::Duration(5), bool relative=false, bool blocking=true);
  
  /**
   * @brief Set the endeffector/gripper to a given pose w.r.t. to the robot base frame
   * @details Overload: see setEndeffectorPose(const Eigen::Affine3d& desired_pose, const ros::Duration& duration, bool relative, bool blocking)
   * @param desired_pose Desired 6D poses but with some limitations mentioned above.
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the final state.
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */ 
  void setEndeffectorPose(const Eigen::Affine3d& desired_pose, double speed, bool relative=false, bool blocking=true);
  
  /**
   * @brief Compute the joint angles that correspond to a given 4D pose w.r.t. to the robot base frame
   * @details This methods overloads a more generic function and limits the pose to the position part and a pitch angle.
   * @param desired_xyz Desired [x,y,z] coordinates in the base frame
   * @param desired_pitch Desired pitch angle in the base frame (<e> endeffector points upwards for -pi/2 [rad] and downwards for +pi/2 [rad] </e>)
   * @param duration duration for the transition to the new joint state.
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   * @return \c true if a solution was found, \c false otherwise.
   */
  bool setEndeffectorPose(const Eigen::Ref<const Eigen::Vector3d>& desired_xyz, double desired_pitch, const ros::Duration& duration=ros::Duration(5), bool relative=false, bool blocking=true);
  
  /**
   * @brief Compute the joint angles that correspond to a given 4D pose w.r.t. to the robot base frame
   * @details This methods overloads a more generic function and limits the pose to the position part and a pitch angle.
   * @param desired_xyz Desired [x,y,z] coordinates in the base frame
   * @param desired_pitch Desired pitch angle in the base frame (<e> endeffector points upwards for -pi/2 [rad] and downwards for +pi/2 [rad] </e>)
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the final state.
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   * @return \c true if a solution was found, \c false otherwise.
   */
  bool setEndeffectorPose(const Eigen::Ref<const Eigen::Vector3d>& desired_xyz, double desired_pitch, double speed, bool relative=false, bool blocking=true);
  
  /**
   * @brief Compute the joint angles that correspond to a given 4D pose w.r.t. to the robot base frame
   * @details This methods overloads a more generic function and limits the pose to the position part and a pitch angle.
   * @param desired_xyz Desired [x,y,z] coordinates in the base frame (as std::vector< double >)
   * @param desired_pitch Desired pitch angle in the base frame (<e> endeffector points upwards for -pi/2 [rad] and downwards for +pi/2 [rad] </e>)
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the final state.
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   * @return \c true if a solution was found, \c false otherwise.
   */
  bool setEndeffectorPose(const std::vector<double>& desired_xyz, double desired_pitch, double speed, bool relative=false, bool blocking=true);
  
  
    /**
   * @brief Incrementally change the position of the endeffector / gripper (in world coordinates).
   * @details This function simplifies changing the relative position, that is also possibe with the more generic method 
   * 	      setEndeffectorPose(const Eigen::Ref< const Eigen::Vector3d >& desired_xyz, double desired_pitch, double speed, bool relative, bool blocking),
   * 	      by setting relative to true and pitch to 0
   * @param dx increment of the x coordinate
   * @param dy increment of the y coordinate
   * @param dz increment of the z coordinate
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the final state.
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   * @return \c true if a solution was found, \c false otherwise.
   */
  bool setEndeffectorPoseInc(double dx, double dy, double dz, double speed, bool blocking=true)
  {
    return setEndeffectorPose(Eigen::Vector3d(dx,dy,dz), 0, speed, true, blocking);
  }
  
  
  /**
   * @brief Compute the geometric jacobian of the endeffector/gripper velocity w.r.t. to the base frame.
   * 
   * The geometric jacobian denotes the linear relationship between the endeffector velocity and the joint angle 
   * velocities w.r.t. to the robot base frame: v = J*qdot.
   * Each row corresponds to the joint velocities and each row correspond to the endeffector velocity.
   * The resulting matrix is therefore 6x4.
   * The rotational component (lower 3x4 submatrix) expresses the rotational component of the end effector velocity
   * in terms of angular velocity (angle and axis representation). 
   * @see KinematicModel::computeJacobian
   * @remarks This is not the analytic robot jacobian!
   * @param[out] jacobian The 6x4 jacobian will be written into this matrix.
   */
  void getJacobian(RobotJacobian& jacobian);
  
  /**
   * @brief Compute reduced robot jacobian of the endeffector/gripper velocity w.r.t. to the base frame (x,y,z,pitch).
   * 
   * This matrix corresponds to the geometric and analytic jacobian, since only the pitch angle is considered
   * for the orientation part:
   *     | dx/dq1       dx/dq2        dx/dq3       dx/dq4  |    
   * J = | dy/dq1       dy/dq2        dy/dq3       dy/dq4  |
   *     | dz/dq1       dz/dq2        dz/dq3       dz/dq4  |
   *     | 0            1             1            1       |
   * @sa computePoseError
   * @param joint_values Joint configuration q=[q1,q2,q3,q4] in which the jacobian should be computed
   * @param[out] jacobian4d The 4x4 reduced jacobian will be written into this matrix.
   */  
  void getJacobianReduced(RobotJacobianReduced& jacobian4d);
  
  
  /**
   * @brief Access the kinematics model of the robot
   * 
   * Use the kinematics model in order to calculate/simulate forward/differential/inverse kinematics
   * for arbitrary joint states
   * @return KinematicModel object (read-only)
   */
  const KinematicModel& kinematics() const {return _kinematics;}
  
  
  //! A small test script that drives into a predefined set of joint configuratoins in order to test the underlying KinematicModel
  bool testKinematicModel();
  //! Drives to a joint configuration and verify the forward kinematics of the underlying KinematicModel
  bool testKinematicModel(const Eigen::Ref<const JointVector>& joint_values);
  
  //@}
  
  
  /** @name Gripper methods */
  //@{ 
  
  /**
   * @brief Open or close the gripper.
   * 
   * The distance between both gripper fingers is set according to a percentage.
   * A value of 0 means that the gripper is closed. A value of 100 corresponds to 
   * the maximum separation (open).
   * @warning Be careful if you use the gripper to pick things. We have no feedbeck
   *          if the gripper is already grasping. A too small value could damage the servo.
   *          Reduce the param \c percent_open only slighly!!!
   * @param percent_open Desired separation in percent [0..100] (0 = closed)
   * @param blocking  if \c true, wait until the action is completed or the timeout is exceeded before continuing 
   */
  void setGripperJoint(int percent_open, bool blocking = true);
  
  
  /**
   * @brief Set the gripper joint angle directly.
   * 
   * You may get the raw joint value using getGripperJointAngle().
   * @warning Be careful if you use the gripper to pick things. We have no feedbeck
   *          if the gripper is already grasping. A too small value could damage the servo.
   * @param joint_value Actual joint angle of the servo
   * @param blocking  if \c true, wait until the action is completed or the timeout is exceeded before continuing 
   */  
  void setGripperRawJointAngle(double joint_value, bool blocking = true);
  
  /** 
   * @brief Return the current joint angle of the gripper
   * @return current joint angle
   */
  double getGripperJointAngle();
  
 
  //@}
  
  
  /** @name Utility methods */
  //@{ 

  /**
   * @brief Check if a given joint vector exceeds joint limits
   * @param joint_values vector of joint values q=[q1,q2,q3,q4]^T
   * @return \c true if the joint vector exceeds limits, \c false otherwise
   */
  bool isExceedingJointLimits(const Eigen::Ref<const JointVector>& joint_values);
    
  /**
   * @brief Check if a given set of joint values leads to a self-collision.
   * 
   * Currently we test only, that the end-effector does not collide with
   * the robot base. The robot base is modeled as a rectangular with infinite depth (into the ground).
   * Additionally, the fourth joint is tested.
   * @todo Improve collision checking.
   * @todo Parameters of the rectangle are hardcoded now. Declare them as config paramteres.
   * @todo We have some problems with velocity control here, that need to be fixed.
   *	   But for now we bypass this problem by deactivating the collision check in case of velocity control.
   * @param joint_values vector of joint values q=[q1,q2,q3,q4]^T
   * @return \c true if the joint vector does not self-collide, \c false otherwise
   */
  bool checkSelfCollision(const Eigen::Ref<const JointVector>& joint_values);
    
  /**
   * @brief Construct a point-to-point transition in joint_space with individual velocity profiles.
   * 
   * This method creates a joint space trajectory between \c start_conf and \c goal_conf such that
   * each joint drives to the \c goal_conf with it's individual velocity.
   * The transition is completed, after each joint arrives at the \c goal_conf.
   * @param start_conf start joint configuration
   * @param goal_conf goal joint configuration
   * @param speed individual joint velocities
   * @param[out] trajectory joint trajectory that contains subgoals in order to achieve the behavior desribed above.
   */
  void createP2PTrajectoryWithIndividualVel(const Eigen::Ref<const JointVector>& start_conf,
                                                   const Eigen::Ref<const JointVector>& goal_conf,
                                                   const Eigen::Ref<const JointVector>& speed,
                                                   trajectory_msgs::JointTrajectory& trajectory);
  
  
  /**
   * @brief Construct a point-to-point transition in joint_space with individual velocity profiles.
   * 
   * This method creates a joint space trajectory between \c start_conf and \c goal_conf such that
   * each joint drives to the \c goal_conf with it's individual velocity.
   * The transition is completed, after each joint arrives at the \c goal_conf.
   * @param start_conf start joint configuration given as std::vector of doubles
   * @param goal_conf goal joint configuration given as std::vector of doubles
   * @param speed individual joint velocities given as std::vector of doubles
   * @param[out] trajectory joint trajectory that contains subgoals in order to achieve the behavior desribed above.
   */  
  void createP2PTrajectoryWithIndividualVel(const std::vector<double>& start_conf,
                                                   const std::vector<double>& goal_conf,
                                                   const std::vector<double>& speed,
                                                   trajectory_msgs::JointTrajectory& trajectory);
  
  
  
  /**
   * @brief Compute a point-to-point joint space trajectory using a quintic polynomial
   * @detail By default zero velocity and acceleration are assumed at the start and goal point.
   * Optionally, you can provide desired velocities.
   * @param q0 Start joint configuration
   * @param qf Goal joint configuration
   * @param n Desired number of samples
   * @param dt temporal resolution (time between consecutive samples)
   * @param[out] trajectory Resulting trajectory
   * @param v0 Initial joint velocity vector (optional)
   * @param vf Final joint velocity vector (optional)
   */    
  void createQuinticPolynomialJointTrajectory(const Eigen::Ref<const JointVector>& q0, const Eigen::Ref<const JointVector>& qf,
                                                      unsigned int n, double dt, trajectory_msgs::JointTrajectory& trajectory, 
                                                      const JointVector* v0 = nullptr, const JointVector* vf = nullptr);
  
  /**
   * @brief Stop any running transition.
   */
  void stopMoving();
  
  /**
   * @brief Relax all servos (deactivate torque)
   */
  bool relaxServos();
  
  /**
   * @brief Signal handler that terminates all actions before calling ros::shutdown().
   * 
   * The signal handler will be automatically overwritten in the initialize method.
   */
  static void phantomXSigHandler(int sig)
  {
    // cancel the all goals
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_arm("/arm_controller/follow_joint_trajectory", true);
    ac_arm.waitForServer(ros::Duration(0.1));
    ac_arm.cancelAllGoals();
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac_gripper("/gripper_controller/gripper_action", true);
    ac_gripper.waitForServer(ros::Duration(0.1));
    ac_gripper.cancelAllGoals();
    // Default ros sigint handler
    ros::shutdown();
  } 
  
  /**
   * @brief Print the position and velocity porfile of a given trajectory on the screen.
   */
  static void printTrajectory(const trajectory_msgs::JointTrajectory& trajectory);
  
  //! Access lower joint limits (read-only)
  const JointVector& getLowerJointLimits() const {return _joint_lower_bounds;}
  //! Access upper joint limits (read-only)
  const JointVector& getUpperJointLimits() const {return _joint_upper_bounds;}
  //! Access maximum (absolute) joint velocities (read-only)
  const JointVector& getMaxJointSpeeds() const {return _joint_max_speeds;}
  
  //@}
  


  
  
protected:
  
  /**
   * @brief Check and adapt trajectory in order to satisfy joint restrictions.
   * @param trajectory parameter to be checked and modified
   * @return \c true if trajectory is feasible (some velocity adjustments could take place, check warnings),
   *	     \c false if joint angle limits are exceeded.
   */
  bool verifyTrajectory(trajectory_msgs::JointTrajectory& trajectory);
  
  
private:
    
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  
  
  std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> _arm_action; //!< Action client for trajectory following
  std::unique_ptr<actionlib::SimpleActionClient<control_msgs::GripperCommandAction>> _gripper_action; //!< Action client for gripper actions
  
  KinematicModel _kinematics;
  
  ros::Subscriber _joints_sub;
  ros::CallbackQueue* _joints_sub_queue = nullptr; // seems to be deleted by ros
  std::unique_ptr<ros::AsyncSpinner> _joints_sub_spinner;
  bool _joint_values_received = false;
  tf::TransformListener _tf;
  
  std::vector< ros::ServiceClient > _joint_relax_services;
  
  Eigen::Affine3d _base_T_j1 = Eigen::Affine3d::Identity(); //!< Coordinate transformation from the base to the first joint
  Eigen::Affine3d _j1_T_base = Eigen::Affine3d::Identity(); //!< Coordinate transformation from the first joint to the base
  
  std::mutex _joints_mutex;
  JointVector _joint_angles = JointVector::Zero();
  JointVector _joint_velocities = JointVector::Zero();
  double _gripper_value = 0;
  
  JointVector _joint_lower_bounds;
  JointVector _joint_upper_bounds;
  JointVector _joint_max_speeds;
  
  double _gripper_lower_bound = 0;
  double _gripper_upper_bound = 0;
//   double _gripper_neutral = 0;
//   double _gripper_max_speed = 0;
  std::string _gripper_joint_name;
  
  std::map<std::string, int> _map_joint_to_index;
  
  std::vector<std::string> _joint_names_arm; //!< Store names for all joints of the arm
  
  bool _collision_check_enabled = true; //! Workaround, this variable is set to false in case of velocity control 
  
  bool _initialized = false;
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace phantomx


#endif /* PHANTOMX_INTERFACE_H_ */
