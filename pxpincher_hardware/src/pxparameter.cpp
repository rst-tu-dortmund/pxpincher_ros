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
 * Authors: Maximilian Krämer, Christoph Rösmann
 *********************************************************************/

#include "pxpincher_hardware/pxparameter.h"

#include <XmlRpcValue.h>
#include <ros/ros.h>

namespace pxpincher
{

PXParameter::PXParameter()
{
    update();
}

void PXParameter::update()
{
    ros::NodeHandle nHandle;

    nHandle.getParam("/pxpincher/baud",baud_);
    nHandle.getParam("/pxpincher/port",port_);
    nHandle.getParam("/pxpincher/rate",rate_);
    nHandle.getParam("/pxpincher/simulation",simulation_);
    
    
    // parse joint parameters
    
    names_.clear();
    ids_.clear();
    cwlimits_.clear();
    ccwlimits_.clear();
    speeds_.clear();
    offsets_.clear();
    
    XmlRpc::XmlRpcValue joints_yaml;
    if (ros::param::get("/pxpincher/joints", joints_yaml))
    {
        ROS_ASSERT(joints_yaml.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        for (XmlRpc::XmlRpcValue::iterator it=joints_yaml.begin(); it!=joints_yaml.end(); ++it) 
        {
            names_.push_back( it->first ); // add joint name
            
            // now parse individual values
            ROS_ASSERT( it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct ); // TODO: maybe exceptions or error case handling

            // id
            ROS_ASSERT(it->second["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
            ids_.push_back( (int) it->second["id"] );
            
            // cwlimits
            ROS_ASSERT(it->second["cwlimit"].getType() == XmlRpc::XmlRpcValue::TypeInt);
            cwlimits_.push_back( (int) it->second["cwlimit"] );
            
            // ccwlimits
            ROS_ASSERT(it->second["ccwlimit"].getType() == XmlRpc::XmlRpcValue::TypeInt);
            ccwlimits_.push_back( (int) it->second["ccwlimit"] );
            
            // speeds
            ROS_ASSERT(it->second["speed"].getType() == XmlRpc::XmlRpcValue::TypeInt);
            speeds_.push_back( (int) it->second["speed"] );      
            
            // angle offsets
            ROS_ASSERT(it->second["offset"].getType() == XmlRpc::XmlRpcValue::TypeInt);
            offsets_.push_back( (int) it->second["offset"] );
            
            hardware_modes_.push_back( HardwareMode::STOPPED );    
            
            names_idx_map_.emplace(names_.back(), ids_.size()-1);
        }
    }
    else
    {
        ROS_ERROR("Cannot read joints form parameter server");
    }
}


} // end namespace pxpincher