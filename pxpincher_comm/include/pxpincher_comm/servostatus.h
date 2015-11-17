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

#ifndef SERVOSTATUS_H
#define SERVOSTATUS_H

#include "codes.h"

namespace pxpincher
{

/**
 * @class ServoStatus
 * @brief Servo status object containing the present position, speed, load and further data of a specific servo
 * @see PXProtocol, ServoTarget
 */
class ServoStatus
{
public:
    
    /**
     * @brief Holistic constructor
     * @param id servo id
     * @param position position [0, 1023], unit: 0.29 degree, center point = 512.
     * @param speed speed [0, 2047], ccw: [0, 1023] cw: [1024, 2047], unit in joint mode: 0.111rpm 
     * @param load max torque [0, 1023], unit: 0.1%.
     * @param voltage current voltage, unit 10V
     * @param temperature current temperature, unit 1°C
     */
    ServoStatus(UBYTE id, int position, int speed, int load, UBYTE voltage, UBYTE temperature);
    
    /**
     * @brief Constructor with zero initialization
     * @param id servo id
     */
    ServoStatus(UBYTE id);

    /**
     * @brief Clear servo status
     */
    void clear();

    UBYTE id_; //!< servo id
    int position_; //!< position [0, 1023], unit: 0.29 degree, center point = 512.
    int speed_; //!< speed [0, 2047], ccw: [0, 1023] cw: [1024, 2047], unit in joint mode: 0.111rpm 
    int load_; //!< max torque [0, 1023], unit: 0.1%.
    UBYTE voltage_; //!< current voltage, unit 10V
    UBYTE temperature_; //!< current temperature, unit 1°C
};

} // end namespace pxpincher

#endif // SERVOSTATUS_H
