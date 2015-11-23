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

#ifndef PXMISC_H
#define PXMISC_H

#include <cmath>
#include <limits>

namespace pxpincher
{
    

constexpr double PI = 3.141592653589793;
constexpr double conversionFactorPos = 5*PI/3066;
constexpr double conversionFactorSpeed = 19*PI/5115;
constexpr double INF = 0.8*std::numeric_limits< double >::max();

/**
* @brief Determine the sign of a specified value
* @param val value to be checked
* @return -1 if val<0; 0 if val==0; 1 if val>0
*/
template <typename T> int sign(T val) 
{
    return (T(0) < val) - (val < T(0));
}
    
inline double tick2rad(int position)
{
    return conversionFactorPos*position;
}


inline std::vector<double> tick2rad(const std::vector<int>& positions)
{
    std::vector<double> rads;
    rads.reserve(positions.size());
    
    for(int elem : positions){
        rads.push_back(tick2rad(elem));
    }
    return rads;
}

inline int rad2tick(double rad)
{
    return std::floor(rad/conversionFactorPos);
}

inline double tick2rads(int speed)
{
    return conversionFactorSpeed*speed;
}


inline std::vector<double> tick2rads(const std::vector<int>& speeds)
{
    std::vector<double> rads;
    rads.reserve(speeds.size());

    for(int elem : speeds){
        rads.push_back(tick2rads(elem));
    }
    return rads;
}

inline int rads2tick(double speed_rad)
{
    return std::floor(speed_rad/conversionFactorSpeed);
}

inline double convVoltage(int volt)
{
    return double(volt)/10.0;
}

inline std::vector<double> convVoltage(const std::vector<int>& volt)
{
    std::vector<double> new_volt;
    new_volt.reserve(volt.size());

    for(int elem : volt){
        new_volt.push_back(convVoltage(elem));
    }
    return new_volt;
}

} // end namespace pxpincher

#endif // PXMISC_H
