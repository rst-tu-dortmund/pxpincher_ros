#include "pxpincher_cpp/pxparameter.h"

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

    std::map<std::string,int> map;

    nHandle.getParam("/pxpincher/joints/J1",map);
    ids_.push_back((UBYTE)map["id"]);
    cwlimits_.push_back(map["cwlimit"]);
    ccwlimits_.push_back(map["ccwlimit"]);
    speeds_.push_back(map["speed"]);

    map.clear();

    nHandle.getParam("/pxpincher/joints/J2",map);
    ids_.push_back((UBYTE)map["id"]);
    cwlimits_.push_back(map["cwlimit"]);
    ccwlimits_.push_back(map["ccwlimit"]);
    speeds_.push_back(map["speed"]);

    map.clear();

    nHandle.getParam("/pxpincher/joints/J3",map);
    ids_.push_back((UBYTE)map["id"]);
    cwlimits_.push_back(map["cwlimit"]);
    ccwlimits_.push_back(map["ccwlimit"]);
    speeds_.push_back(map["speed"]);

    map.clear();

    nHandle.getParam("/pxpincher/joints/J4",map);
    ids_.push_back((UBYTE)map["id"]);
    cwlimits_.push_back(map["cwlimit"]);
    ccwlimits_.push_back(map["ccwlimit"]);
    speeds_.push_back(map["speed"]);

    map.clear();

    nHandle.getParam("/pxpincher/joints/J5",map);
    ids_.push_back((UBYTE)map["id"]);
    cwlimits_.push_back(map["cwlimit"]);
    ccwlimits_.push_back(map["ccwlimit"]);
    speeds_.push_back(map["speed"]);

}
