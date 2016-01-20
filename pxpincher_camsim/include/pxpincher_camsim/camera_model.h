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

#ifndef PXPINCHER_CAMERA_MODEL_H_
#define PXPINCHER_CAMERA_MODEL_H_

#include <ros/ros.h>
#include <pxpincher_camsim/visual_object.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>

namespace pxpincher
{
  
struct CameraParameters
{
  std::string camera_frame = "camera_link";
  
  std::string window_name = "Image";
  int rows = 500;
  int cols = 500;
  int center_u = 250;
  int center_v = 250;
  double focal_length = 1;
  double opening_angle_x = M_PI/3;
  double opening_angle_y = M_PI/3;
  int blur_kernel_size = 0;
  
  int no_random_circles = 0;
  int max_radius_rnd_circles = 20;
  int no_random_lines = 0;
  int max_thickness_rnd_lines = 10;
  bool rnd_objects_overlapping = false;
};
  
class CameraModel
{
  
public:
  CameraModel();
  ~CameraModel() {}

  void renderImage(cv::Mat& image, const std::vector<VisualObject>& objects, const std::string& map_frame, bool preview);
  
  CameraParameters& params() {return params_;}
  const CameraParameters& params() const {return params_;}
  
protected:
  
  bool getExtrinsicTransformation(const std::string& map_frame, tf::StampedTransform& transform);
  void getIntrinsicTransformation(const tf::Pose& pose_camframe, int& u, int& v);
  
  /**
   * @todo We assume a camera that is not rotated around the x or y axis for now!
   * @todo the viewing angle is currently calculated according to the center (change to boundary)
   */
  void drawCircleObject(cv::Mat& image, const VisualObject& object, const tf::StampedTransform& extr_transform);
  
  /**
   * @todo We assume a camera that is not rotated around the x or y axis for now!
   */
  void drawRectangleObject(cv::Mat& image, const VisualObject& object, const tf::StampedTransform& extr_transform);
  
  double getOpeningAngleX(const tf::Pose& pose_camframe) const;
  double getOpeningAngleY(const tf::Pose& pose_camframe) const;
  
  
  void blurImage(cv::Mat& image);
  
  void drawRandomCircles(cv::Mat& image);
  void drawRandomLines(cv::Mat& image);
  
  cv::Scalar randomColor()
  {
    int icolor = (unsigned) rng_;
    return cv::Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
  }
  
private:
  
  CameraParameters params_;
  tf::TransformListener listener_;
  
  cv::RNG rng_; // opencv random number generator
  
}; 
  
  
  
} // end namespace

#endif /* PXPINCHER_VISUAL_OBJECT_H_ */
