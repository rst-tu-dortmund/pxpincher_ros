#ifndef PXPARAMETER_H
#define PXPARAMETER_H

#include <vector>
#include <string>

#include "ros/ros.h"

#include "pxpincher_comm/codes.h"

class PXParameter
{
public:
    PXParameter();


    void update();

    int baud_;
    int rate_;
    std::string port_;
    bool simulation_;
    std::vector<int> cwlimits_,ccwlimits_,speeds_;
    std::vector<UBYTE> ids_;
    std::vector<std::string> names_;

};

#endif // PXPARAMETER_H
