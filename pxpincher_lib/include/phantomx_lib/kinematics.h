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

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

// ros stuff
#include <ros/ros.h>

// own stuff
#include <phantomx_lib/types.h>
#include <phantomx_lib/misc.h>




namespace phantomx
{
  


/**
 * @class KinematicsModel
 * @brief This class provides methods regarding the kinematics of the 4-DOF PhantomX pincher robot arm.
 * 
 * This class allows the user to e.g. simulate end effector poses for varying joint states, inverse kinematics
 * and differential kinematics. It is currently based on the Denavit-Hartenberg parameters of the robot.
 * Unfortunately, we cannot apply DH parameters since the turtlebot_arm package does not define the coordinate systems
 * in the robot URDF according to the convention. E.g. all rotations are around the y-axis.
 * 
 * If you want to simulate the kinematics model in Matlab using Peter Corke's Robotics Toolbox, use the following model:
 * @code
 *      l1 = Link('revolute','d',0.065,'a',0,'alpha',-pi/2);
 *      l2 = Link('revolute','d',0,'a',0.1015,'alpha',0,'offset',-pi/2);
 *      l3 = Link('revolute','d',0,'a',0.1015,'alpha',0);
 *      l4 = Link('revolute','d',0,'a',0.0880,'alpha',0);
 *      arm = SerialLink([l1,l2,l3,l4]);
 *      arm.tool = troty(-pi/2) * trotx(pi) * trotz(-pi/2);
 * @endcode
 * But only the relation between the arm_base_link and gripper frame are identically. The individual joint 
 * coordinate systems differ from each other (since the robotics toolbox relies on DH paramters)
 * 
 * @todo Make templated for arbitrary robots
 * @todo Extract trasnformations from URDF/Mesh files of the robot.
 * @remarks We restrict ourselves to revolute joints only.
 */
class KinematicModel
{
public:
  
  /**
   * @brief Construct the class
   */
  KinematicModel() {};
  
  /**
   * @brief Destruct the class
   */
  virtual ~KinematicModel() {};
  
  //! set the coordinate transformation from the base coordinate system to the system of joint 1 (for <b>q=0</b>)
  void setBaseToJoint1Transform(const Eigen::Affine3d& base_T_j1)
  {
    _base_T_j1 = base_T_j1;
    _j1_T_base = base_T_j1.inverse();
  };
  //! set the coordinate transformation from joint 1 to joint 2 for <b>q=0</b> (joint 1 rotates around y)
  void setJoint1ToJoint2Transform(const Eigen::Affine3d& j1_T_j2)
  {
    _j1_T_j2 = j1_T_j2;
    _j2_T_j1 = j1_T_j2.inverse();
  };
  //! set the coordinate transformation from joint 2 to joint 3 for <b>q=0</b> (joint 2 rotates around y)
  void setJoint2ToJoint3Transform(const Eigen::Affine3d& j2_T_j3) {_j2_T_j3 = j2_T_j3;};
  //! set the coordinate transformation from joint 3 to joint 4 for <b>q=0</b> (joint 3 rotates around y)
  void setJoint3ToJoint4Transform(const Eigen::Affine3d& j3_T_j4) {_j3_T_j4 = j3_T_j4;};
  //! set the coordinate transformation from joint 4 to the gripper (TCP) coordiante system for <b>q=0</b> (joint 3 rotates around y)
  void setJoint4ToGripperTransform(const Eigen::Affine3d& j4_T_gripper) {_j4_T_gripper = j4_T_gripper;};
  //! set lower and upper joint limits
  void setLowerAndUpperJointLimits(const Eigen::Ref<const JointVector> lower_bounds, const Eigen::Ref<const JointVector> upper_bounds)
  {
    _joint_lower_bounds = lower_bounds;
    _joint_upper_bounds = upper_bounds;
  }
  
  /**
   * @brief Compute the forward kinematics according to the specified joint angles.
   * @param joint_values Vector of joint values q=[q1,q2,q3,q4]^T
   * @param up_to_index Specify final frame: \c 0 -> joint1 frame, \c 1 -> joint2 frame, \c 2 -> joint3 frame
   *                                         \c 3 -> joint 4 frame, \c 4 -> gripper / TCP frame
   *                    Be careful if you use this for other purposes than computing the jacobian, since
   *                    the rotation of a joint angle defines rotation around y to the subsequent frame instead of the current one:
   *                    E.g. joint1 frame is independent of the angle of joint1. This does not correspond to the actual
   *                    transformation provided by tf!
   * @return Transformation from the base coordinate system to the gripper (tool-center-point) coordinate system
   */
  Eigen::Affine3d computeForwardKinematics(const Eigen::Ref<const JointVector>& joint_values, int up_to_index=4) const;
  
  /**
   * @brief Compute the geometric jacobian of the endeffector/gripper velocity w.r.t. to the base frame.
   * 
   * The geometric jacobian denotes the linear relationship between the endeffector velocity and the joint angle 
   * velocities w.r.t. to the robot base frame: v = J*qdot.
   * Each row corresponds to the joint velocities and each row correspond to the endeffector velocity.
   * The resulting matrix is therefore 6x4.
   * The rotational component (lower 3x4 submatrix) expresses the rotational component of the end effector velocity
   * in terms of angular velocity (angle and axis representation). 
   * @remarks This is not the analytic robot jacobian!
   * @sa computePoseError
   * @param joint_values Joint configuration q=[q1,q2,q3,q4] in which the jacobian should be computed
   * @param[out] jacobian The 6x4 jacobian will be written into this matrix.
   */
  void computeJacobian(const Eigen::Ref<const JointVector>& joint_values, RobotJacobian& jacobian) const;
  
  
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
  void computeJacobianReduced(const Eigen::Ref<const JointVector>& joint_values, RobotJacobianReduced& jacobian4d) const;
  
  
  /**
   * @brief Compute the joint angles that correspond to a given pose w.r.t. to the robot base frame
   * 
   * Since the robot has only 4-DOF, all 6D poses cannot be represtend by a set of joint angles.
   * The underlying inverse kinematics tries to set the translation and the pitch angle of the desired pose.
   * Since we have only one degree of freedom to set a yaw angle, yaw-orientation of the gripper should 
   * coincide with the distance vector to the gripper. Otherwise a warning is displayed. The yaw angle is interesting
   * in case of the arm singularity in which infinite solutions appear for the first joint value (for the position-IK).
   * The implementation accounts for joint limits.
   * If no solution within the joint limits is found, the algorihm tries to set q1=q1+pi and solves the IK again
   * resulting in a similar gripper position but with switched sides (of the gripper itself).
   * @test This method requires more testing.
   * @param desired_pose Desired 6D poses but with some limitations mentioned above.
   * @param[out] joint_values the corresponding joint values. Initit them with the current joint values, in order to choose
   *                          either the elbow up or elbow down solution depending on the current angular distance.
   * @return \c true if a solution was found, \c false otherwise.
   */  
  bool computeInverseKinematics(const Eigen::Affine3d& desired_pose, Eigen::Ref<JointVector> joint_values) const;
  
  /**
   * @brief Compute the joint angles that correspond to a given 4D pose w.r.t. to the robot base frame
   * @details This methods overloads a more generic function and limits the pose to the position part and a pitch angle.
   * @param desired_xyz Desired [x,y,z] coordinates in the base frame
   * @param desired_pitch Desired pitch angle in the base frame (<e> endeffector points upwards for -pi/2 [rad] and downwards for +pi/2 [rad] </e>)
   * @param[out] joint_values the corresponding joint values. Initit them with the current joint values, in order to choose
   *                          either the elbow up or elbow down solution depending on the current angular distance.
   * @return \c true if a solution was found, \c false otherwise.
   */
  bool computeInverseKinematics(const Eigen::Ref<const Eigen::Vector3d>& desired_xyz, double desired_pitch, Eigen::Ref<JointVector> joint_values) const;
  
  /**
   * @brief Compute the joint angles that correspond to a given 4D pose w.r.t. to the robot base frame
   * @details This methods overloads a more generic function and limits the pose to the position part and a pitch angle.
   * @param desired_xyz Desired [x,y,z] coordinates in the base frame (std::vector< double >)
   * @param desired_pitch Desired pitch angle in the base frame (<e> endeffector points upwards for -pi/2 [rad] and downwards for +pi/2 [rad] </e>)
   * @param[out] joint_values the corresponding joint values. Initit them with the current joint values, in order to choose
   *                          either the elbow up or elbow down solution depending on the current angular distance.
   * @return \c true if a solution was found, \c false otherwise.
   */
  bool computeInverseKinematics(const std::vector<double>& desired_xyz, double desired_pitch, Eigen::Ref<JointVector> joint_values) const;

protected:
  
  //! Helper method to compute the inverse kinematics of joint 2-4, interpreted as 3-link-planar arm.
  bool computeIk3LinkPlanar(const Eigen::Affine3d& j2_T_pose, Eigen::Ref<Eigen::Vector3d> values, bool elbow_up) const;
  //! Helper method to compute both the elbow up and elbow down pose of the 3-link-planar part of the arm and choosing the closer solution.
  bool computeIk3LinkPlanarElbowUpAndDown(const Eigen::Affine3d& j2_T_pose, Eigen::Ref<Eigen::Vector3d> values) const;
  
private:
    
  Eigen::Affine3d _base_T_j1 = Eigen::Affine3d::Identity();
  Eigen::Affine3d _j1_T_j2 = Eigen::Affine3d::Identity();
  Eigen::Affine3d _j2_T_j3 = Eigen::Affine3d::Identity();
  Eigen::Affine3d _j3_T_j4 = Eigen::Affine3d::Identity();
  Eigen::Affine3d _j4_T_gripper = Eigen::Affine3d::Identity();
  
  Eigen::Affine3d _j1_T_base = Eigen::Affine3d::Identity(); // the inverse of _base_T_j1, computed in the accessor method
  Eigen::Affine3d _j2_T_j1 = Eigen::Affine3d::Identity(); // the inverse of _j1_T_j2, computed in the accessor method
  
  JointVector _joint_lower_bounds = JointVector::Constant(-M_PI);
  JointVector _joint_upper_bounds = JointVector::Constant(M_PI);
  
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace phantomx


#endif /* KINEMATICS_H_ */
