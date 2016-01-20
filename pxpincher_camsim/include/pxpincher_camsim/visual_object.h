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

#ifndef PXPINCHER_VISUAL_OBJECT_H_
#define PXPINCHER_VISUAL_OBJECT_H_

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

namespace pxpincher
{
  
  
class VisualObject
{
  
public:
  
  enum class Shape {CIRCLE, RECTANGLE};
  
  struct Color
  {
    int r = 0;
    int g = 0;
    int b = 0;
  };
  
  VisualObject() {};
  VisualObject(const std::string& name, const tf::Vector3& position, const tf::Quaternion& orientation, Shape shape, const Color& color) 
     : name_(name), position_(position), orientation_(orientation), shape_(shape), color_(color) {};
     
  ~VisualObject() {};
  
  std::string& name() {return name_;}
  const std::string& name() const {return name_;}
  
  tf::Vector3& position() {return position_;}
  const tf::Vector3& position() const {return position_;}
  
  tf::Quaternion& orientation() {return orientation_;}
  const tf::Quaternion& orientation() const {return orientation_;}
  
  Shape& shape() {return shape_;}
  const Shape& shape() const {return shape_;}
  
  Color& color() {return color_;}
  const Color& color() const {return color_;}
  
  double& width() {return width_;}
  const double& width() const {return width_;}
  
  double& height() {return height_;}
  const double& height() const {return height_;}
  
  void getPose(tf::Pose& pose) const
  {
    pose.setOrigin(position_);
    pose.setRotation(orientation_);
  }
  
private:
  
  std::string name_;
  tf::Vector3 position_;
  tf::Quaternion orientation_;
  Shape shape_ = Shape::CIRCLE;
  Color color_;
  double width_ = 1.0;
  double height_ = 1.0;
  
}; 
  
  
  
} // end namespace

#endif /* PXPINCHER_VISUAL_OBJECT_H_ */
