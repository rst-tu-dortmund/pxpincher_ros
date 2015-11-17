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

#ifndef MISC_H_
#define MISC_H_

#include <memory>
#include <math.h>
#include <type_traits>

#include <phantomx_lib/types.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Geometry>

  

namespace phantomx
{
  
  
  /**
   * @brief Convert angle from degree to radiant
   * @param angle_deg angle given in degree
   * @return angle in radiant [0, 2pi]
   */
  inline double deg_to_rad(double angle_deg)
  {
    return angle_deg * 0.01745329251994329576; // pi/180
  }
  
  /**
   * @brief Convert angle from radiant to degree
   * @param angle_rad angle given in degree
   * @return angle in degree [0, 360]
   */
  inline double rad_to_deg(double angle_rad)
  {
    return angle_rad * 57.29577951308232087721; // 180/M_PI
  }
  
  /**
   * @brief Normalize angle in radiant to the interval (-pi,2]
   * @param angle angle given in radiant
   * @return angle in radiant (-pi, pi]
   */  
  inline double normalize_angle_rad(double angle)
  {
    if (angle >= -M_PI && angle < M_PI)
      return angle;
    
    double mu = floor(angle / (2 * M_PI));
    angle = angle - mu * 2 * M_PI;
    
    if (angle >= M_PI)
      angle -= 2*M_PI;
    
    if (angle < -M_PI)
      angle += 2*M_PI;
    
    return angle;
  }
  
  /**
   * @brief Normalize angle in degree to the interval [0, 360)
   * @param angle angle given in degree
   * @return angle in degree [0, 360)
   */  
  inline double normalize_angle_deg_360(double angle)
  {
    if (angle >= 0 && angle < 360)
      return angle;
        
    return fmod(angle, 360.);
  }
  
  /** 
   * @brief Check if two doubles are similar
   * @param a first value
   * @param b second value
   * @return \c true if <c> fabs(a-b)< threshold </c>, otherwise \c false
   */
  inline bool is_approx(double a, double b, double threshold=1e-6)
  {
    return fabs(a-b) < threshold;
  }
  
  /**
   * @brief Check if x is inside the interval [l, u]
   * @param l lower bound
   * @param x value to be checked
   * @param u upper bound
   * @tparam T numerical/scalar type
   * @return \c true if inside, otherwise \c false
   */
  template <typename T, typename std::enable_if<std::is_scalar<T>::value>::type* = nullptr>
  inline bool isInsideInterval(T l, T x, T u) 
  {
    if (x < l)
      return false;
    if (x > u)
      return false;
    return true;
  }
  
  /**
   * @brief Check if each component of vector x is inside the corresponding interval [vec_l, vec_u]
   * @param l lower bound vector
   * @param x vector to be checked
   * @param u upper bound vector
   * @return \c true if inside, otherwise \c false
   */
  inline bool isInsideInterval(const Eigen::Ref<const Eigen::VectorXd>& l, const Eigen::Ref<const Eigen::VectorXd>& x, const Eigen::Ref<const Eigen::VectorXd>& u) 
  {
    if ( (x.array() < l.array()).all() )
      return false;
    if ( (x.array() > u.array()).all() )
      return false;
    return true;
  }
  
  
  /**
   * @brief Bound x to the interval [l, u]
   * @param l lower bound
   * @param x value to be bounded
   * @param u upper bound
   * @tparam T numerical type
   * @return bounded value
   */
  template <typename T, typename std::enable_if<std::is_scalar<T>::value>::type* = nullptr>
  inline T bound(T l, T x, T u) 
  {
    if (x < l)
      return l;
    if (x > u)
      return u;
    return x;
  }
  
  /**
   * @brief Bound vector x to the interval [vector_l, vector_u]
   * @param l lower bound
   * @param x vector to be bounded
   * @param u upper bound
   * @return bounded value
   */
  inline Eigen::VectorXd bound(const Eigen::Ref<const Eigen::VectorXd>& l, const Eigen::Ref<const Eigen::VectorXd>& x, const Eigen::Ref<const Eigen::VectorXd>& u) 
  {
      return (x.array() < l.array()).select( l, (x.array()>u.array()).select(u, x) );
  }
  
  /**
   * @brief Apply upper bound such that x<=u
   * @param x value to be bounded
   * @param u upper bound
   * @tparam T numerical type
   * @return bounded value
   */
  template <typename T, typename std::enable_if<std::is_scalar<T>::value>::type* = nullptr>
  inline T upper_bound(T x, T u) 
  {
    if (x > u)
      return u;
    return x;
  }
  
  
  /**
   * @brief Apply lower bound such that x>=u
   * @param l lower bound
   * @param x value to be bounded
   * @tparam T numerical type
   * @return bounded value
   */
  template <typename T, typename std::enable_if<std::is_scalar<T>::value>::type* = nullptr>
  inline T lower_bound(T l, T x) 
  {
    if (x < l)
      return l;
    return x;
  }    
    
  /**
   * @brief Convert roll-pitch-yaw angles to a 3d rotation matrix.
   * @param rpy vector containing [roll,pitch,yaw] angles
   * @return 3x3 rotation matrix
   */  
  inline Eigen::Matrix3d convertRpyToRotMat(const Eigen::Ref<const RpyVector>& rpy)
  {    
    return (Eigen::AngleAxisd(rpy.coeffRef(2), Eigen::Vector3d::UnitZ())
	  * Eigen::AngleAxisd(rpy.coeffRef(1), Eigen::Vector3d::UnitY()) 
	  * Eigen::AngleAxisd(rpy.coeffRef(0), Eigen::Vector3d::UnitX())).toRotationMatrix();
  }
  
  
  /**
   * @brief Convert a rotation matrix to roll-pitch-yaw angles
   * @param rot_mat 3x3 rotation matrix
   * @return vector contining [roll,pitch,yaw] angles
   */
  inline RpyVector convertRotMatToRpy(const Eigen::Ref<const Eigen::Matrix3d>& rot_mat)
  {    
    // see http://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel
    RpyVector rpy;
    rpy.coeffRef(2) = std::atan2( rot_mat.coeffRef(1,0), rot_mat.coeffRef(0,0) ); //atan2(r21,r11)
    rpy.coeffRef(1) = std::atan2( -rot_mat.coeffRef(2,0), sqrt( rot_mat.coeffRef(0,0)*rot_mat.coeffRef(0,0) + rot_mat.coeffRef(1,0)*rot_mat.coeffRef(1,0) ) ); //atan2(-r31,sqrt(r11^2 + r21^2))
    rpy.coeffRef(0) = std::atan2( rot_mat.coeffRef(2,1), rot_mat.coeffRef(2,2)  ); //atan2(r32,r33)
    
    // check for singularities
    if (fabs(rpy.coeffRef(1)-M_PI/2)<1e-4)
    {
      rpy.coeffRef(2) = 0;
      rpy.coeffRef(0) = std::atan2( rot_mat.coeffRef(0,1), rot_mat.coeffRef(1,1)  ); //atan2(r12,r22)
    } 
    else if (fabs(rpy.coeffRef(1)+M_PI/2)<1e-4)
    {
      rpy.coeffRef(2) = 0;
      rpy.coeffRef(0) = -std::atan2( rot_mat.coeffRef(0,1), rot_mat.coeffRef(1,1)  ); //-atan2(r12,r22)
    }
    return rpy;
  }
  
  /**
   * @brief Convert 6D pose [x,y,z,roll,pitch,yaw] to a transformation matrix.
   * @param pose_6d  vector containing [x,y,z,roll,pitch,yaw]
   * @return 3x3 rotation matrix
   */  
  inline Eigen::Affine3d convert6dPoseToMat(const Eigen::Ref<const Eigen::Matrix<double,6,1>>& pose_6d)
  {    
    Eigen::Affine3d mat;
    mat.matrix().topRightCorner<3,1>() = pose_6d.head<3>();
    mat.matrix().topLeftCorner<3,3>() = convertRpyToRotMat(pose_6d.tail<3>());
    mat.matrix().bottomLeftCorner<1,3>().setZero();
    mat.matrix().coeffRef(3,3) = 1.0;
    return mat;
  }
  
  
  /**
   * @brief Create a transformation matrix from a position part and a pitch angle
   * @remarks This function might be helpful for "4d poses" related to the phantomX robot
   * @param pos desired 3d position [x,y,z]^T
   * @param pitch desired pitch angle [rad]
   * @return transformation matrix representing the pose
   */
  inline Eigen::Affine3d createPoseFromPosAndPitch(const Eigen::Ref<const Eigen::Vector3d>& pos, double pitch)
  {
    Eigen::Affine3d pose;
    pose.linear() = Eigen::AngleAxisd( pitch, Eigen::Vector3d::UnitY() ).toRotationMatrix();
    pose.translation() = pos;
    return pose;
  }
    
  /**
   * @brief Convert a skew symmetric matrix to the underlying vector
   * @param skew_mat 3x3 skew mat with the following layout:
   * 			| 0   -vz  vy|
   *			| vz   0  -vx|
   *           		|-vy   vx  0 |
   * @remarks Each component [vx,vy,vz] is determined by the mean value of the related matrix elements
   * @return 3D vector [vx, vy, vz]^T
   */
  inline Eigen::Vector3d convertSkewSymMatToVec(const Eigen::Ref<const Eigen::Matrix3d>& skew_mat)
  {
    return 0.5*Eigen::Vector3d(skew_mat.coeffRef(2,1)-skew_mat.coeffRef(1,2),
			       skew_mat.coeffRef(0,2)-skew_mat.coeffRef(2,0),
			       skew_mat.coeffRef(1,0)-skew_mat.coeffRef(0,1));
  }
    
  /**
   * @brief Compute the orientation error / differential motion between two 3d rotation matrices.
   * @details The function computes an approximation of the differential motion from \c rot1 to \c rot2
   * @param rot1 3x3 rotation matrix
   * @param rot2 3x3 rotation matrix
   * @return differential motion [dRx, dRy, dRz]^T as approximation to the average spatial velocity multiplied by time
   */  
  inline Eigen::Vector3d computeOrientationError(const Eigen::Ref<const Eigen::Matrix3d>& rot1, const Eigen::Ref<const Eigen::Matrix3d>& rot2)
  {
    return convertSkewSymMatToVec(rot2 * rot1.transpose() - Eigen::Matrix3d::Identity());
  }
    
  /**
   * @brief Compute the pose error / differential motion between two transformation matrices.
   * @details The function computes an approximation of the differential motion from \c pose1 to \c pose2
   * @param pose1 transformation matrix representing the first pose
   * @param pose2 transformation matrix representing the second pose
   * @return differential motion [dx, dy, dz, dRx, dRy, dRz]^T as approximation to the average spatial velocity multiplied by time
   */     
  inline Pose6D computePoseError(const Eigen::Affine3d& pose1, const Eigen::Affine3d& pose2)
  {
    Pose6D diff;
    diff.head(3) = pose2.translation()-pose1.translation();
    diff.tail(3) = computeOrientationError(pose1.linear(), pose2.linear());
    return diff;
  }
  
  /**
   * @brief Compute the pose error / differential motion between two transformation matrices (4D version).
   * @details The function computes an approximation of the differential motion from \c pose1 to \c pose2
   * @param pose1 transformation matrix representing the first pose
   * @param pose2 transformation matrix representing the second pose
   * @return differential motion [dx, dy, dz, dRy]^T as approximation to the average spatial velocity multiplied by time
   */     
  inline Pose4D computePoseError4D(const Eigen::Affine3d& pose1, const Eigen::Affine3d& pose2)
  {
    Pose4D diff;
    diff.head(3) = pose2.translation()-pose1.translation();
    diff.coeffRef(3) = computeOrientationError(pose1.linear(), pose2.linear()).coeffRef(1);
    return diff;
  }
    
  
  /**
   * @brief Compute the relative transformation between two poses
   * @param from_pose first transformation
   * @param to_pose second transformation
   * @return relative transformation between \c from_pose and \c to_pose
   */
  inline Eigen::Affine3d computeRelativeTransform(const Eigen::Affine3d& from_pose, const Eigen::Affine3d& to_pose)
  {
    Eigen::Affine3d rel;
    rel.linear() = ( Eigen::Quaterniond( from_pose.linear() ) * Eigen::Quaterniond( to_pose.linear() ).conjugate() ).toRotationMatrix(); // conj = inv for unit-quaternions
    rel.translation() = to_pose.translation() - from_pose.translation();
    return rel;
  }
  
  
  /**
   * @brief Linearly interpolate line given by a start and a goal point
   * @param x0 Start of the line
   * @param xf End of the line
   * @param[out] path_out the interpolated line will be written to this container [resulting size: n]
   * @param n desired number of samples including start and end point.
   * @tparam VecType A vector or scalar type that overloads usual math operators (such as Eigen::VectorXd)
   * @tparam VecTypeContainer A STL compatible container of \c VecType such like std::vector<VecType>
   */
  template <typename VecType, typename VecTypeContainer>
  inline void interpolateLineLinear(const VecType& x0, const VecType& xf, VecTypeContainer& path_out, unsigned int n)
  {
    VecType diff = xf-x0;
    double step_length = 1 / double(n-1);
    path_out.clear();
    for (unsigned int i=0; i<n; ++i)
    {
        path_out.emplace_back( x0 + double(i)*step_length*diff );
    }
  }
  
  /**
   * @brief Get velocity profile of a given path
   * @param path Container of path points [Arbitrary vectors]
   * @param[out] vel_out The corresponding velocity profile
   * @param dt sampling time (duration between consecutive path points)
   * @tparam VecType A vector or scalar type that overloads usual math operators (such as Eigen::VectorXd)
   * @tparam VecTypeContainer A STL compatible container of \c VecType such like std::vector<VecType>
   */
  template <typename VecType, typename VecTypeContainer>
  inline void getVelocityProfile(const VecTypeContainer& path, VecTypeContainer& vel_out, double dt)
  {
    if (path.empty())
    {
        ROS_WARN("getVelocityProfile(): Cannot compute velocity profile since path is empty.");
        return;
    }
    ROS_ASSERT_MSG(dt>0, "getVelocityProfile(): dt must be greater than zero");
    vel_out.clear();
    for (int i=0; i<path.size()-1; ++i)
    {
        vel_out.emplace_back( ( path[i+1] - path[i] )/dt  );
    }
  }
    
    
  /**
   * @brief Compute a point-to-point trajectory using a quintic polynomial
   * @detail By default zero velocity and acceleration are assumed at the start and goal point.
   * Optionally, you can provide desired velocities.
   * The transition time is scaled to the interval [0,1].
   * The implementaion is based on http://www.petercorke.com/RTB/r9/html/jtraj.html.
   * @param x0 Start point [Mx1 vector]
   * @param xf Goal point [Mx1 vector]
   * @param n Desired number of samples
   * @param[out] pos_out Resulting trajectory as a container of \c VecType with \c n elements
   * @param v0 Initial velocity vector (optional)
   * @param vf Final velocity vector (optional)
   * @param[out] vel_out Resulting velocity profile as a container of \c VecType with \c n elements (optional)
   * @param[out] acc_out Resulting acceleration profile as a container of \c VecType with \c n elements (optional)
   * @tparam VecType A vector or scalar type that overloads usual math operators (such as Eigen::VectorXd)
   * @tparam VecTypeContainer A STL compatible container of \c VecType such like std::vector<VecType>
   */    
   template <typename VecType, typename VecTypeContainer> 
   inline void createQuinticPolynomialTrajectory(const VecType& x0, const VecType& xf,
                                               unsigned int n,
                                               VecTypeContainer& pos_out, 
                                               const VecType* v0 = nullptr, const VecType* vf = nullptr,
                                               VecTypeContainer* vel_out = nullptr, 
                                               VecTypeContainer* acc_out = nullptr)
                                               
   {
       // check initial and final constraints
       VecType v0_;
       if (v0) v0_ = *v0;
       else v0_.setZero();
       
       VecType vf_;
       if (vf) vf_ = *vf;
       else vf_.setZero();
       
       pos_out.clear();
       if (vel_out) vel_out->clear();
       if (acc_out) acc_out->clear();
              
      
       // compute coefficients
       Eigen::VectorXd a = 6*(xf-x0) - 3*(vf_+v0_);
       Eigen::VectorXd b = -15*(xf-x0) + 8*v0_ + 7*vf_;
       Eigen::VectorXd c = 10*(xf-x0) - 6*v0_ - 4*vf_;
       Eigen::VectorXd d = Eigen::VectorXd::Zero(x0.rows());
       Eigen::VectorXd e = v0_;
       Eigen::VectorXd f = x0;
      
       // each row corresponds to a joint vector component (after transposition)
       Eigen::MatrixXd coeff_mat_pos(x0.rows(), 6);
       coeff_mat_pos << a, b, c, d, e, f;
       coeff_mat_pos.transposeInPlace();
       Eigen::MatrixXd coeff_mat_vel(x0.rows(), 6);
       if (vel_out)
       {
        coeff_mat_vel << d, 5*a, 4*b, 3*c, d, e;
        coeff_mat_vel.transposeInPlace();
       }
       Eigen::MatrixXd coeff_mat_acc(x0.rows(), 6);
       if (acc_out)
       {
        coeff_mat_acc << d, d, 20*a, 12*b, 6*c, d;
        coeff_mat_acc.transposeInPlace();
       }
       
       double dt = 1/(double(n)-1);
       
       double t = 0;
       for (int i=0; i<n; ++i)
       {
           Eigen::Matrix<double,1,6> t_vec;
           t_vec << t*t*t*t*t, t*t*t*t, t*t*t, t*t, t, 1;
           pos_out.emplace_back( t_vec * coeff_mat_pos );
           
           if (vel_out)
               vel_out->emplace_back( t_vec * coeff_mat_vel );
           if (acc_out)
               acc_out->emplace_back( t_vec * coeff_mat_acc );
      
           t += dt;
       }
   }
       
  
  
  /**
   * @brief Convert Eigen::Vector to a std::vector
   * @param eig_vec input type: eigen vector
   * @param[out] stl_vec reference to the output std::vector< double >
   * @tparam T type of data (usually double, float, int or bool)
   */
  template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* = nullptr>
  inline void convertEigenVectorToSTL(const Eigen::Ref<const Eigen::Matrix<T,-1, 1>>& eig_vec, std::vector<T>& stl_vec)
  {
      stl_vec.assign(eig_vec.data(), eig_vec.data()+eig_vec.rows());
  }
  
  /**
    * @brief Constructs an object of type T and wraps it in a std::unique_ptr. 
    * 
    * Until C++14 is not widespread, we define our own make_unique.
    * @param args Arbitrary argument list that is compatible with the constructor list required by type \c T
    * @return std::unique_ptr of type \c T constructed with \c args
   */
  template<typename T, typename... Args>
  inline std::unique_ptr<T> make_unique(Args&&... args)
  {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }
  
 

} // namespace phantomx


#endif /* MISC_H_ */
