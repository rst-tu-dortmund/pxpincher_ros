#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "pxpincher_cpp/pxpincher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pxpincher_rst");

    PxPincher pincher;

    pincher.start();

    return 0;
}
