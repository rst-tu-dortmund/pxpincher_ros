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



// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "pxpincher_test");
  ros::NodeHandle n("~");
  
 
  pxpincher::PhantomXControl robot;
  robot.initialize();

  robot.setJoints({0.8, 0.6, 0.9, 1.6});
  robot.setGripperJoint(0);
  //robot.setGripperJoint(100);
  robot.setGripperJoint(20);
  
  ROS_INFO_STREAM(std::setprecision(2) << "Current joint configuration q=[" << robot.getJointAngles().transpose() << "]");
 
  robot.setEndeffectorPoseInc(0, 0, -0.05, 0.1); // notice: blocking call
  
  Eigen::Affine3d tcp;
  robot.getEndeffectorState(tcp);
  ROS_INFO_STREAM(std::setprecision(2) << "TCP rotation matrix w.r.t. base:\n" << tcp.rotation());
  ROS_INFO_STREAM(std::setprecision(2) << "TCP translation vector w.r.t. base: [" << tcp.translation().transpose() << "]");
  
  robot.setEndeffectorPoseInc(0, -0.2, 0, 0.1, false); // notice: non-blocking call
  
  ros::Rate r(10);
  while (ros::ok())
  {
      robot.publishInformationMarker();
      ros::spinOnce();
      r.sleep();
  }
  

  
  
  return 0;
}

