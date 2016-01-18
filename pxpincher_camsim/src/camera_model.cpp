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

#include <pxpincher_camsim/camera_model.h>

#include <opencv2/opencv.hpp>
#include <limits>


namespace pxpincher
{

CameraModel::CameraModel()
{
}


void CameraModel::renderImage(const std::vector<VisualObject>& objects, const std::string& map_frame, bool preview)
{
  
  
  cv::Mat image = cv::Mat::zeros(params_.rows, params_.cols, CV_8UC3);
  
  // get extrinsic transform
  tf::StampedTransform extr_transform;
  if ( !getExtrinsicTransformation(map_frame, extr_transform) )
    return;
  
  for (const VisualObject& object : objects)
  {
     if (object.shape() == VisualObject::Shape::CIRCLE)
     {
       drawCircleObject(image, object, extr_transform);
       continue;
     }
    
    
  }
    
  if (preview)
  {
    cv::imshow( params_.window_name.c_str(), image );
    cv::waitKey(1);
  }
}


void CameraModel::drawCircleObject(cv::Mat& image, const VisualObject& object, const tf::StampedTransform& extr_transform)
{ 
  // perform extrinsic transformation
  tf::Pose pose_map;
  object.getPose(pose_map);
  tf::Pose pose_cam;
  pose_cam = extr_transform * pose_map;
  
  if (pose_cam.getOrigin().getZ() <=0) // object is behind camera
    return;
  
  // check opening angle
  // u:
  if ( std::abs( std::acos( pose_cam.getOrigin().z() / std::sqrt( std::pow(pose_cam.getOrigin().x(),2) + 
                                                                    std::pow(pose_cam.getOrigin().y(),2) + 
                                                                    std::pow(pose_cam.getOrigin().z(),2) ) ) ) > params_.opening_angle_x)
    return;
  
  // v
  if ( std::abs( std::acos( pose_cam.getOrigin().y() / std::sqrt( std::pow(pose_cam.getOrigin().x(),2) + 
                                                                    std::pow(pose_cam.getOrigin().y(),2) + 
                                                                    std::pow(pose_cam.getOrigin().z(),2) ) ) - M_PI/2) > params_.opening_angle_y)
    return;
  
  
  
  // perform intrinsic transformation
  int u, v;
  getIntrinsicTransformation(pose_cam, u, v);
  
  //TODO:  We assume a camera that is not rotated around the x or y axis for now!
  
  // get horizontal and vertical size of the circle (might be an ellipse after transforming)
  tf::Pose top = pose_map;
  top.getOrigin().setZ( top.getOrigin().z() + object.height()/2 );
  int u_top, v_top;
  getIntrinsicTransformation(extr_transform * top, u_top, v_top);
  
  tf::Pose left = pose_map;
  left.getOrigin().setY( left.getOrigin().y() + object.width()/2 );
  int u_right, v_right;
  getIntrinsicTransformation(extr_transform * left, u_right, v_right);
    
  int height = std::abs(v_top-v); // just half of the size is required by opencv
  int width = std::abs(u_right-u); // just half of the size is required by opencv

  // draw
  //cv::circle( image, cv::Point(u, v), 5, cv::Scalar( object.color().b, object.color().g, object.color().r ), -1, 8 );  
  cv::ellipse( image, cv::Point(u, v), cv::Size( width, height ), 0, 0, 360,  cv::Scalar( object.color().b, object.color().g, object.color().r ),  -1, 8 );
  
  
}


void CameraModel::getIntrinsicTransformation(const tf::Pose& pose_camframe, int& u, int& v)
{
  if (pose_camframe.getOrigin().z() == 0)
  {
    ROS_WARN("Warning, cannot transform into camera frame, z=0");
    u = std::numeric_limits<int>::max();
    v = std::numeric_limits<int>::max();
    return;
  }
  double u_d = -params().focal_length / pose_camframe.getOrigin().z() * pose_camframe.getOrigin().x();
  double v_d = -params().focal_length / pose_camframe.getOrigin().z() * pose_camframe.getOrigin().y();
  u = (int) u_d + params().center_u;
  v = (int) v_d + params().center_v;
}

bool CameraModel::getExtrinsicTransformation(const std::string& map_frame, tf::StampedTransform& transform)
{
    try
    {
      listener_.waitForTransform(params_.camera_frame, map_frame, ros::Time(0), ros::Duration(1.0) );
      listener_.lookupTransform(params_.camera_frame, map_frame, ros::Time(0), transform);
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR("CameraModel::performExtrinsicTransformation: transform error:\n%s",ex.what());
      transform.setIdentity();
      return false;
    }        
    return true;
}



} // end namespace
