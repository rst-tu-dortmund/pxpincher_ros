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

#ifndef SERVOTARGET_H
#define SERVOTARGET_H

#include "codes.h"

namespace pxpincher
{

/**
 * @class ServoTarget
 * @brief Define a servo target containing a target/goal position, speed and maximum torque.
 * @see PXProtocol, ServoStatus
 */
class ServoTarget
{
public:
    
    /**
     * @brief Holistic constructor
     * @param id servo id
     * @param target_position target/goal position [0, 1023], unit: 0.29 degree, center point = 512.
     * @param target_speed target speed [0, 2047], ccw: [0, 1023] cw: [1024, 2047], unit in joint mode: 0.111rpm 
     * @param max_torque max torque [0, 1023], unit: 0.1%.
     */
    ServoTarget(UBYTE id, int target_position, int target_speed, int max_torque);
    
    /**
     * @brief Constructor with zero initialization
     * @param id servo id
     */
    ServoTarget(UBYTE id);

     /**
     * @brief Clear servo target
     */
    void clear();

    UBYTE id_; //!< servo id
    int target_position_; //!< target/goal position [0, 1023], unit: 0.29 degree, center point = 512.
    int target_speed_; //!< target speed [0, 2047], ccw: [0, 1023] cw: [1024, 2047], unit in joint mode: 0.111rpm 
    int max_torque_; //!< max torque [0, 1023], unit: 0.1%.
};


}

#endif // SERVOTARGET_H
