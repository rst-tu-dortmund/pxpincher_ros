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


#include <termios.h>
char getch(); // allow to capture keyboard inputs without blocking the program (we use this to disable and enable torque / relax)



// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "pxpincher_teach");
  ros::NodeHandle n("~");
  
  ros::Publisher pub_workspace = n.advertise<sensor_msgs::PointCloud>("workspace", 100);
  
  pxpincher::PhantomXControl robot;
  robot.initialize();
  
  ROS_INFO("Sampling joint configurations to generate workspace pointcloud ...");
  sensor_msgs::PointCloud workspace;
  robot.visualizeWorkSpace(workspace, 0.5);
  ROS_INFO("Sampling finished.");
  
  robot.activateInteractiveJointControl();
  
  ros::Rate r(10);
  
  ROS_INFO("Press 'r' to (un)relax motors (set torque on/off)");
  ROS_INFO("Press 'd' to drive into default configuration");
  
  bool relaxed = false;
  
  while (ros::ok())
  {
      
      char c = getch();
      if (c == 'r' || c == 'R')
      {
        robot.stopMoving();
        
        if (!robot.isExceedingJointLimits(robot.getJointAngles()))
        {    
            robot.relaxServos(!relaxed);
            relaxed = !relaxed;
            ROS_INFO_STREAM(std::boolalpha << "Servos relaxed: " << relaxed);
        }
        else
        {
            ROS_WARN("Robot exceeds joint limits, cannot unrelax motors. Please move the robot into its workspace manually");
        }
      }
      if (c== 'd' || c == 'D')
      {
          if (!relaxed)
          {
            ROS_INFO_STREAM("Driving to default configuration");
            robot.setJointsDefault(0.5*robot.getMaxJointSpeeds(), false);
          }
          else
            ROS_WARN_STREAM("Cannot drive to default configuration, since servos are relaxed.");
      }

    
      // visualize work space
      workspace.header.stamp = ros::Time::now();
      pub_workspace.publish(workspace);
      
      robot.publishInformationMarker();
      
      ros::spinOnce();
      r.sleep();
  }
  

  
  return 0;
}



// source http://answers.ros.org/question/63491/keyboard-key-pressed/
char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 50;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if (rv != 0) // == 0 -> nothing selected
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}

