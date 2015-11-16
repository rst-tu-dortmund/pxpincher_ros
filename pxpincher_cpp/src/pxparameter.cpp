#include "pxpincher_cpp/pxparameter.h"

#include <XmlRpcValue.h>
#include <ros/ros.h>

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
           
        }
    }
    else
    {
        ROS_ERROR("Cannot read joints form parameter server");
    }
}
