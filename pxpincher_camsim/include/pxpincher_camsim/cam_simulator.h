/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
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

#ifndef PXPINCHER_CAM_SIMULATOR_H_
#define PXPINCHER_CAM_SIMULATOR_H_

#include <ros/ros.h>
#include <pxpincher_camsim/visual_object.h>
#include <pxpincher_camsim/camera_model.h>
#include <image_transport/image_transport.h>

#include <tf/transform_datatypes.h>

namespace pxpincher
{
  
  
  
class CamSimulator
{
public:
  CamSimulator();
  ~CamSimulator() {};
  
  void initialize();
  
  void start();
  
protected:
  
  bool getObjectsFromParamServer();
  
  void visualize3D();
  
  
private:
  
  ros::NodeHandle nhandle_;
  
  ros::Publisher vis_pub_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  
  std::string map_frame_;
  std::string cam_topic_;
  double rate_;
  
  bool live_preview_;
  
  std::vector<VisualObject> objects_;
  
  CameraModel cam_;
  
  bool initialized_;
  
  
}; 
  
  
  
} // end namespace

#endif /* PXPINCHER_CAM_SIMULATOR_H_ */
