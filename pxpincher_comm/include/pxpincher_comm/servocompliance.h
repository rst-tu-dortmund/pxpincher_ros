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

#ifndef SERVOCOMPLIANCE_H
#define SERVOCOMPLIANCE_H

#include "codes.h"

namespace pxpincher
{

/**
 * @class ServoCompliance
 * @brief Object storing all compliance parameters of a servo
 * 
 * Refer to http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1A for details.
 * @see PXProtocol
 */
class ServoCompliance
{
public:
    
    /**
     * @brief Holistic constructor
     * @param id servo id
     * @param cw_margin clockwise margin
     * @param ccw_margin counter-clockwise margin
     * @param cw_slope clockwise slope
     * @param ccw_slope counter-clockwise slope
     * @param punch punch value
     */
    ServoCompliance(UBYTE id, UBYTE cw_margin, UBYTE ccw_margin, UBYTE cw_slope, UBYTE ccw_slope, UBYTE punch);
    
    /**
     * @brief Constructor with zero initialization
     * @param id servo id
     */
    ServoCompliance(UBYTE id);

    
    /**
     * @brief Clear servo status
     */
    void clear();

    UBYTE id_;
    UBYTE cw_margin_;
    UBYTE ccw_margin_;
    UBYTE cw_slope_;
    UBYTE ccw_slope_;
    UBYTE punch_;
};

} // end namespace pxpincher

#endif // SERVOCOMPLIANCE_H
