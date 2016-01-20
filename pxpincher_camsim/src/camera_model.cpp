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
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <limits>


namespace pxpincher
{

CameraModel::CameraModel() : rng_( 0xFFFFFFFF )
{
}


void CameraModel::renderImage(cv::Mat& image, const std::vector<VisualObject>& objects, const std::string& map_frame, bool preview)
{
  
  
  image = cv::Mat::zeros(params_.rows, params_.cols, CV_8UC3);
  
  // get extrinsic transform
  tf::StampedTransform extr_transform;
  if ( !getExtrinsicTransformation(map_frame, extr_transform) )
    return;
  
  if (!params_.rnd_objects_overlapping) 
  {
    drawRandomCircles(image);
    drawRandomLines(image);
  }
  
  
  for (const VisualObject& object : objects)
  {
     if (object.shape() == VisualObject::Shape::CIRCLE)
     {
       drawCircleObject(image, object, extr_transform);
       continue;
     }
     
     if (object.shape() == VisualObject::Shape::RECTANGLE)
     {
       drawRectangleObject(image, object, extr_transform);
       continue;
     }
    
    
  }
  
  if (params_.rnd_objects_overlapping) 
  {
    drawRandomCircles(image);
    drawRandomLines(image);
  }
      
  if (params_.blur_kernel_size) 
    blurImage(image);  
  
  
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
  
  if (pose_cam.getOrigin().getZ() <=0.01) // object is behind camera
    return;

  // check opening angle
  if ( getOpeningAngleX(pose_cam) > params_.opening_angle_x || getOpeningAngleY(pose_cam) > params_.opening_angle_y)
    return;

   
  
  // perform intrinsic transformation
  int u, v;
  getIntrinsicTransformation(pose_cam, u, v);
  
  //TODO:  We assume a camera that is not rotated around the x or y axis for now!
  
  // get horizontal and vertical size of the circle (might be an ellipse after transforming) 
  tf::Pose top = pose_map;
  tf::Vector3 unit_y(0,1,0);
  tf::Vector3 rotated_y = tf::quatRotate(object.orientation(), unit_y);
  top.getOrigin() += rotated_y * object.height()/2.0;
  int u_top, v_top;
  getIntrinsicTransformation(extr_transform * top, u_top, v_top);
  
  tf::Pose left = pose_map;
  tf::Vector3 unit_x(1,0,0);
  tf::Vector3 rotated_x = tf::quatRotate(object.orientation(), unit_x);
  left.getOrigin() += rotated_x * object.width()/2.0;
  int u_right, v_right;
  getIntrinsicTransformation(extr_transform * left, u_right, v_right);
    
//   ROS_INFO_STREAM("u: (" << u << "," << v << ")" << " top: (" << u_top << "," << v_top << ") right: (" << u_right << "," << v_right << ")" );
  int height = std::abs(u_top-u); // just half of the size is required by opencv
  int width = std::abs(v_right-v); // just half of the size is required by opencv

  // draw
  //cv::circle( image, cv::Point(u, v), 5, cv::Scalar( object.color().b, object.color().g, object.color().r ), -1, 8 );  
  cv::ellipse( image, cv::Point(u, v), cv::Size( width, height ), 0, 0, 360,  cv::Scalar( object.color().b, object.color().g, object.color().r ),  -1, 8 );
}


void CameraModel::drawRectangleObject(cv::Mat& image, const VisualObject& object, const tf::StampedTransform& extr_transform)
{ 
  // perform extrinsic transformation
  tf::Pose pose_map;
  object.getPose(pose_map);
  tf::Pose pose_cam;
  pose_cam = extr_transform * pose_map;
  
  if (pose_cam.getOrigin().getZ() <=0.01) // object is behind camera
    return;
    
  // check opening angle
  if ( getOpeningAngleX(pose_cam) > params_.opening_angle_x || getOpeningAngleY(pose_cam) > params_.opening_angle_y)
    return;
  
  //TODO:  We assume a camera that is not rotated around the x or y axis for now!
  
  std::array<cv::Point,4> edges;
  double w = object.width()/2;
  double h = object.height()/2;
  
  tf::Pose top_right = pose_map;
  tf::Vector3 dir1(w,h,0);
  tf::Vector3 rotated1 = tf::quatRotate(object.orientation(), dir1);
  top_right.getOrigin() += rotated1;
  tf::Pose cam_top_right = extr_transform * top_right;
  getIntrinsicTransformation(cam_top_right, edges[0].x, edges[0].y);
    
  tf::Pose top_left = pose_map;
  tf::Vector3 dir2(-w,h,0);
  tf::Vector3 rotated2 = tf::quatRotate(object.orientation(), dir2);
  top_left.getOrigin() += rotated2;
  tf::Pose cam_top_left = extr_transform * top_left;
  getIntrinsicTransformation(cam_top_left, edges[1].x, edges[1].y);
  
  tf::Pose bottom_left = pose_map;
  tf::Vector3 dir3(-w,-h,0);
  tf::Vector3 rotated3 = tf::quatRotate(object.orientation(), dir3);
  bottom_left.getOrigin() += rotated3;
  tf::Pose cam_bottom_left = extr_transform * bottom_left;
  getIntrinsicTransformation(cam_bottom_left,  edges[2].x, edges[2].y);
  
  tf::Pose bottom_right = pose_map;
  tf::Vector3 dir4(w,-h,0);
  tf::Vector3 rotated4 = tf::quatRotate(object.orientation(), dir4);
  bottom_right.getOrigin() += rotated4;
  tf::Pose cam_bottom_right = extr_transform * bottom_right;
  getIntrinsicTransformation(cam_bottom_right, edges[3].x, edges[3].y);

    
  std::vector<double> angles_x;
  angles_x.push_back( getOpeningAngleX(cam_top_right) );
  angles_x.push_back( getOpeningAngleX(cam_top_left) );
  angles_x.push_back( getOpeningAngleX(cam_bottom_left) );
  angles_x.push_back( getOpeningAngleX(cam_bottom_right) );

//   // check opening angle x
//   if ( *std::min_element(angles_x.begin(),angles_x.end() ) > params_.opening_angle_x)
//     return;
//   
//   std::vector<double> angles_y;
//   angles_y.push_back( getOpeningAngleY(cam_top_right) );
//   angles_y.push_back( getOpeningAngleY(cam_top_left) );
//   angles_y.push_back( getOpeningAngleY(cam_bottom_left) );
//   angles_y.push_back( getOpeningAngleY(cam_bottom_right) );
//   
//   // check opening angle y
//   if ( *std::min_element(angles_y.begin(),angles_y.end() ) > params_.opening_angle_y)
//     return;

  
  // draw
  cv::fillConvexPoly(image, edges.data(), (int)edges.size(), cv::Scalar( object.color().b, object.color().g, object.color().r ), 8, 0);
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

double CameraModel::getOpeningAngleX(const tf::Pose& pose_camframe) const
{
    return std::abs( std::acos( pose_camframe.getOrigin().z() / std::sqrt( std::pow(pose_camframe.getOrigin().x(),2) + 
                                                                           std::pow(pose_camframe.getOrigin().y(),2) + 
                                                                           std::pow(pose_camframe.getOrigin().z(),2) ) ) );
}
double CameraModel::getOpeningAngleY(const tf::Pose& pose_camframe) const
{
     return std::abs( std::acos( pose_camframe.getOrigin().y() / std::sqrt( std::pow(pose_camframe.getOrigin().x(),2) + 
                                                                            std::pow(pose_camframe.getOrigin().y(),2) + 
                                                                            std::pow(pose_camframe.getOrigin().z(),2) ) ) - M_PI/2); 
}


void CameraModel::blurImage(cv::Mat& image)
{
  for ( int i = 1; i < params_.blur_kernel_size; i = i + 2 )
  {
    //cv::blur( image, image, cv::Size( i, i ), cv::Point(-1,-1) );
    cv::GaussianBlur( image, image, cv::Size( i, i ), 0, 0 );
  }
}


void CameraModel::drawRandomCircles(cv::Mat& image)
{
  cv::Point center;

  for( int i = 0; i < params_.no_random_circles; i++ )
  {
    center.x = rng_.uniform( 0, params_.cols );
    center.y = rng_.uniform( 0, params_.rows );
    
    int radius = rng_.uniform(1, params_.max_radius_rnd_circles);

    cv::circle( image, center, radius, randomColor(), -1, 8 );
  }
}

void CameraModel::drawRandomLines(cv::Mat& image)
{
  cv::Point pt1, pt2;

  for( int i = 0; i < params_.no_random_lines; i++ )
  {
   pt1.x = rng_.uniform( 0, params_.cols );
   pt1.y = rng_.uniform( 0, params_.rows );
   pt2.x = rng_.uniform( 0, params_.cols );
   pt2.y = rng_.uniform( 0, params_.rows );

   int thickness = rng_.uniform(1, params_.max_thickness_rnd_lines);
   
   cv::line( image, pt1, pt2, randomColor(), thickness, 8 );
  }
}


} // end namespace
