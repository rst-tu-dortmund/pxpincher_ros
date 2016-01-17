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


#include <pxpincher_camsim/cam_simulator.h>

#include <visualization_msgs/Marker.h>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

namespace pxpincher
{

  
CamSimulator::CamSimulator() : nhandle_("~"), initialized_(false)
{
 
}

  
void CamSimulator::initialize()
{
    getObjectsFromParamServer();
    
    map_frame_ = "/map";
    nhandle_.param("map_frame", map_frame_, map_frame_);
    
    rate_ = 10;
    nhandle_.param("rate", rate_, rate_);
    
    nhandle_.param("cam_frame", cam_.params().camera_frame, cam_.params().camera_frame);
    nhandle_.param("focal_length", cam_.params().focal_length, cam_.params().focal_length);
    nhandle_.param("image_width", cam_.params().cols, cam_.params().cols);
    nhandle_.param("image_height", cam_.params().rows, cam_.params().rows);
    nhandle_.param("image_center_x", cam_.params().center_x, cam_.params().center_x);
    nhandle_.param("image_center_y", cam_.params().center_y, cam_.params().center_y);
    
    vis_pub_ = nhandle_.advertise<visualization_msgs::Marker>("object_markers",0);
    
    initialized_ = true;
    ROS_INFO("CamSimulator initialized.");
}


void CamSimulator::start()
{
  if (!initialized_)
  {
    ROS_ERROR("CamSimulator not initialized. Please call initialize() first.");
    return;
  }
  
  ROS_INFO("CamSimulator started.");
  ros::Rate r(rate_);
  while (ros::ok())
  {
    cam_.renderImage(objects_, map_frame_);
    visualize3D();
    r.sleep();
  }
}


void CamSimulator::visualize3D()
{
  for (int i=0; i<(int)objects_.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = "Circular objects";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(2);
    marker.pose.position.x = objects_[i].position().x();
    marker.pose.position.y = objects_[i].position().y();
    marker.pose.position.z = objects_[i].position().z();
    tf::quaternionTFToMsg(objects_[i].orientation(), marker.pose.orientation);
    marker.scale.x = objects_[i].size();
    marker.scale.y = objects_[i].size();
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = float(objects_[i].color().r)/ 255.0;
    marker.color.g = float(objects_[i].color().g)/ 255.0;
    marker.color.b = float(objects_[i].color().b)/ 255.0;
    vis_pub_.publish( marker );
  }
}
  
  
  
bool CamSimulator::getObjectsFromParamServer()
{
  XmlRpc::XmlRpcValue param_yaml;

  if (!nhandle_.getParam("objects", param_yaml))
  {
      ROS_ERROR("Cannot read 'objects' from parameter server");
      return false;
  }
  if(param_yaml.getType() != XmlRpc::XmlRpcValue::TypeArray) // list of objects
  {
      ROS_ERROR("'objects' struct is not correct.");
      return false;
  }
  
  bool retval = true;
  
  objects_.clear();
  
  
  // XmlRpc does not support an implicit conversion between int and double. 
  // Therefore we define a small function for cases in which conversion is desired!
  auto convDouble = [] (XmlRpc::XmlRpcValue& val) -> double
  {
    if (val.getType() == XmlRpc::XmlRpcValue::TypeInt) // XmlRpc cannot cast int to double
      return int(val);
    return val; // if not double, an exception is thrown;
  };
  
  // iterate objects
  for (int i = 0; i < param_yaml.size(); ++i)
  {
      if(param_yaml[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR_STREAM("invalid parameter struct 'objects' found");
        retval = false;
      }
              
      for (XmlRpc::XmlRpcValue::iterator it_obj=param_yaml[i].begin(); it_obj!=param_yaml[i].end(); ++it_obj) // object name + struct
      {
        // Create new empty object
        VisualObject object;
        object.name() = it_obj->first;
        
        if (it_obj->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
          ROS_ERROR_STREAM("Syntax of object '" << object.name() << "' is invalid. Skipping object...");
          retval = false;
          continue;
        }
        
        // iterate elements of the struct
        for (XmlRpc::XmlRpcValue::iterator it_elem=it_obj->second.begin(); it_elem!=it_obj->second.end(); ++it_elem) // object name + struct
        {
            // check parameter pose
            if (it_elem->first == "pose")
            {
                // the second paretmer must be a double array of size 6
                if (it_elem->second.getType() == XmlRpc::XmlRpcValue::TypeArray && it_elem->second.size() == 6)
                {
                    try
                    {
                      object.position().setX( convDouble(it_elem->second[0]) );
                      object.position().setY( convDouble(it_elem->second[1]) );
                      object.position().setZ( convDouble(it_elem->second[2]) );
                      object.orientation() = tf::createQuaternionFromRPY( convDouble(it_elem->second[3]),
                                                                          convDouble(it_elem->second[4]),
                                                                          convDouble(it_elem->second[5]) );
                    }
                    catch (const XmlRpc::XmlRpcException& ex)
                    {
                      ROS_ERROR_STREAM("Cannot add pose element to object '" << object.name() << "': " << ex.getMessage());
                      retval = false;
                    }
                }
                else
                {
                  ROS_ERROR_STREAM("Object '" << object.name() << "' does not define a correct 6d pose");
                  retval = false;
                }
                continue;
            }
            
            // check parameter shape
            if (it_elem->first == "shape")
            {
              try
              {
                if (it_elem->second == "circle")
                {
                  object.shape() = VisualObject::Shape::CIRCLE;
                }
                else
                {
                  ROS_ERROR_STREAM("Shape '" << it_elem->second << "' is not supported. Supported shapes are: 'circle'. ");
                }
              }
              catch (const XmlRpc::XmlRpcException& ex)
              {
                ROS_ERROR_STREAM("Cannot add shape to object '" << object.name() << "': " << ex.getMessage());
                retval = false;
              }
              continue;
            }
            
            // check parameter size
            if (it_elem->first == "size")
            {
              try
              {
                object.size() = convDouble(it_elem->second);
              }
              catch (const XmlRpc::XmlRpcException& ex)
              {
                ROS_ERROR_STREAM("Cannot read size of object '" << object.name() << "': " << ex.getMessage());
                retval = false;
              }
              continue;
            }
            
            // check parameter color
            if (it_elem->first == "color")
            {
                // the second paretmer must be an int array of size 3
                if (it_elem->second.getType() == XmlRpc::XmlRpcValue::TypeArray && it_elem->second.size() == 3)
                {
                    try
                    {
                      object.color().r = it_elem->second[0];
                      object.color().g = it_elem->second[1];
                      object.color().b = it_elem->second[2];
                    }
                    catch (const XmlRpc::XmlRpcException& ex)
                    {
                      ROS_ERROR_STREAM("Cannot read color of object '" << object.name() << "': " << ex.getMessage());
                      retval = false;
                    }
                }
                else
                {
                  ROS_ERROR_STREAM("Object '" << object.name() << "' does not define a correct color (must be a 3d array)");
                  retval = false;
                }
                continue;
            }
        }
        
        
        // add object to container
        objects_.push_back(object);
      }
      
  }

  return retval;

}


} // end namespace
