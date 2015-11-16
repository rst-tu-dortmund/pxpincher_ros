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

#ifndef PXPROTOCOL_H
#define PXPROTOCOL_H

#include "codes.h"
#include "serialcomm.h"
#include "ros/ros.h"

#include <stdio.h>
#include <algorithm> //std::max

#include "servocompliance.h"
#include "servotarget.h"
#include "servostatus.h"

//#define PRINT_BYTES


/**
 * @class PXProtocol
 * @brief Hardware interface to communicate with the arbotix board on the phantomx pincher robot
 * 
 * This class implements the low-level serial communication to the arbotix board on the PhantomX Pincher robot.
 * The arbotix board itself constitutes a gateway to the individual dynamixel servo joints.
 * 
 * The PhantomX Pincher can be either configured (baudrade, id, angle limits, voltage limits, temperature limits, ...)
 * or controlled (position of joints, maximum velocity).
 * Check the related class methods for further details.
 * 
 * For a list of the individual servo commands supported by the Dynamixel servos refer to:
 * http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm
 */
class PXProtocol
{

public:
    
    /**
     * @brief Default constructor
     */
    PXProtocol();

    /** @name Utilitiy methods  */
    //@{
    
    /**
     * @brief Read servo model number of a single servo
     * @param id specific servo id to read from
     * @param[out] number related model number
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readModelNumber(UBYTE id, int &number, SerialComm &comm);
    
    /**
     * @brief Read servo model numbers for a bunch of servos
     * @param ids servo id vector
     * @param[out] numbers resulting vector of model numbers
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readModelNumber(std::vector<UBYTE> ids, std::vector<int> &numbers, SerialComm &comm);

    
    /**
     * @brief Read servo firmware version of a single servo
     * @param id specific servo id to read from
     * @param[out] version related version number
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readFirmwareVersion(UBYTE id,int &version, SerialComm &comm);
    
    /**
     * @brief Read servo firmware versions for a bunch of servos
     * @param ids servo id vector
     * @param[out] versions resulting vector of firmware numbers
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readFirmwareVersion(std::vector<UBYTE> ids, std::vector<int> &versions, SerialComm &comm);

    
    /**
     * @brief Renew the servo id of a servo
     * @warning Each servo id must be unique within all connected servos!
     * @param id current id of the servo
     * @param new_id new servo id  (in range [0, 252])
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setID(UBYTE id, UBYTE new_id, SerialComm &comm);

    
    /**
     * @brief Read baudrate (serial connection) of a single servo
     * 
     * see http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_04
     * @param id specific servo id to read from
     * @param[out] baudrate related baudrate
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readBaudrate(UBYTE id, int &baudrate, SerialComm &comm);
    
    /**
     * @brief Read baudrate (serial connection) for a bunch of servos
     * 
     * see http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_04
     * @param ids servo id vector
     * @param[out] baudrates resulting vector of baudrates
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readBaudrate(std::vector<UBYTE> ids, std::vector<int> &baudrates, SerialComm &comm);
    
    /**
     * @brief Set baudrate (serial connection) of a single servo
     * 
     * see http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_04
     * @param id servo id
     * @param baudrate new baudrate
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setBaudrate(UBYTE id, UBYTE baudrate, SerialComm &comm);

    /**
     * @brief Set return delay of the servo communication
     * 
     * The communication type for the servos is half duplex.
     * After each instruction packet sent to a servo, the controller waits the specified amount of time (return-delay-time) 
     * before the status packet is sent.
     * The delay time per data value is 2 usec. E.g. in case of \c delay == 250, the resulting time is 0.5 ms.
     * See http://support.robotis.com/en/product/dynamixel/dxl_communication.htm
     * and http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_05
     * @param id servo id
     * @param delay new return-time-delay in range [0, 254]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setReturnDelay(UBYTE id, UBYTE delay, SerialComm &comm);
    
    /**
     * @brief Read return delay of the servo communication of a single servo
     * 
     * The delay time per data value is 2 usec. E.g. in case of \c delay == 250, the resulting time is 0.5 ms.
     * See http://support.robotis.com/en/product/dynamixel/dxl_communication.htm
     * and http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_05
     * @param id servo id
     * @param delay related return-time-delay
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readReturnDelay(UBYTE id,int &delay, SerialComm &comm);
    
    /**
     * @brief Read return delay of the servo communication for a bunch of servos
     * 
     * The delay time per data value is 2 usec. E.g. in case of \c delay == 250, the resulting time is 0.5 ms.
     * See http://support.robotis.com/en/product/dynamixel/dxl_communication.htm
     * and http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_05
     * @param ids servo id vector
     * @param delays related return-time-delay
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readReturnDelay(std::vector<UBYTE> ids, std::vector<int> &delays, SerialComm &comm);

    ///@}
    
    /** @name Angle limits */
    ///@{
        
    UBYTE readCWAngleLimit(UBYTE id,int &limit, SerialComm &comm);
    UBYTE readCWAngleLimit(std::vector<UBYTE> ids, std::vector<int> &limits, SerialComm &comm);
    UBYTE readCCWAngleLimit(UBYTE id,int &limit, SerialComm &comm);
    UBYTE readCCWAngleLimit(std::vector<UBYTE> ids, std::vector<int> &limits, SerialComm &comm);

    UBYTE setCWAngleLimit(UBYTE id,int limit, SerialComm &comm);
    UBYTE setCWAngleLimit(std::vector<UBYTE> ids, std::vector<int> limits, SerialComm &comm);

    UBYTE setCCWAngleLimit(UBYTE id,int limit, SerialComm &comm);
    UBYTE setCCWAngleLimit(std::vector<UBYTE> ids, std::vector<int> limits, SerialComm &comm);

    ///@}
    
    UBYTE readTemperatureLimit(UBYTE id,int &limit, SerialComm &comm);
    UBYTE readTemperatureLimit(std::vector<UBYTE> ids, std::vector<int> &limits, SerialComm &comm);
    UBYTE setTemperatureLimit(UBYTE id,UBYTE limit, SerialComm &comm);

    UBYTE readLowVoltageLimit(UBYTE id,int &limit, SerialComm &comm);
    UBYTE readLowVoltageLimit(std::vector<UBYTE> ids, std::vector<int> &limits, SerialComm &comm);
    UBYTE setLowVoltageLimit(UBYTE id,UBYTE limit, SerialComm &comm);

    UBYTE readHighVoltageLimit(UBYTE id,int &limit, SerialComm &comm);
    UBYTE readHighVoltageLimit(std::vector<UBYTE> ids, std::vector<int> &limits, SerialComm &comm);
    UBYTE setHighVoltageLimit(UBYTE id,UBYTE limit, SerialComm &comm);

    UBYTE readTorqueMax(UBYTE id,int &max, SerialComm &comm);
    UBYTE readTorqueMax(std::vector<UBYTE> ids, std::vector<int> &maxs, SerialComm &comm);
    UBYTE setTorqueMax(UBYTE id, int limit, SerialComm &comm);

    UBYTE setReturnLevel(UBYTE id, UBYTE level, SerialComm &comm);

    UBYTE setAlarmLED(UBYTE id, UBYTE level, SerialComm &comm);
    UBYTE readAlarmLED(UBYTE id, int &level, SerialComm &comm);
    UBYTE readAlarmLED(std::vector<UBYTE> ids, std::vector<int> &levels, SerialComm &comm);

    UBYTE setAlarmShutdown(UBYTE id, UBYTE level, SerialComm &comm);
    UBYTE readAlarmShutdown(UBYTE id, int &level, SerialComm &comm);
    UBYTE readAlarmShutdown(std::vector<UBYTE> ids, std::vector<int> &levels, SerialComm &comm);

    UBYTE readTorqueState(UBYTE id, int &state, SerialComm &comm);
    UBYTE readTorqueState(std::vector<UBYTE> ids, std::vector<int> &states, SerialComm &comm);
    UBYTE setTorqueState(UBYTE id, UBYTE state, SerialComm &comm);

    UBYTE readLEDState(UBYTE id, int &state, SerialComm &comm);
    UBYTE readLEDState(std::vector<UBYTE> ids, std::vector<int> &states, SerialComm &comm);
    UBYTE setLEDState(UBYTE id, UBYTE state, SerialComm &comm);

    UBYTE readCWComplianceMargin(UBYTE id, int &margin, SerialComm &comm);
    UBYTE readCWComplianceMargin(std::vector<UBYTE> ids, std::vector<int> &margins, SerialComm &comm);
    UBYTE setCWComplianceMargin(UBYTE id, UBYTE margin, SerialComm &comm);
    UBYTE readCCWComplianceMargin(UBYTE id, int &margin, SerialComm &comm);
    UBYTE readCCWComplianceMargin(std::vector<UBYTE> ids, std::vector<int> &margins, SerialComm &comm);
    UBYTE setCCWComplianceMargin(UBYTE id, UBYTE margin, SerialComm &comm);
    UBYTE readCWComplianceSlope(UBYTE id, int &slope, SerialComm &comm);
    UBYTE readCWComplianceSlope(std::vector<UBYTE> ids, std::vector<int> &slopes, SerialComm &comm);
    UBYTE setCWComplianceSlope(UBYTE id, UBYTE slope, SerialComm &comm);
    UBYTE readCCWComplianceSlope(UBYTE id, int &slope, SerialComm &comm);
    UBYTE readCCWComplianceSlope(std::vector<UBYTE> ids, std::vector<int> &slopes, SerialComm &comm);
    UBYTE setCCWComplianceSlope(UBYTE id, UBYTE slope, SerialComm &comm);
    UBYTE setComplianceMargin(UBYTE id, UBYTE cw, UBYTE ccw, SerialComm &comm);
    UBYTE setComplianceSlope(UBYTE id, UBYTE cw, UBYTE ccw, SerialComm &comm);

    
    
    /** @name Goal position and speed */
    //@{
    
    /**
     * @brief Read current goal position of a single servo
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1E
     * @param id servo id
     * @param delay current goal position [0, 1023]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readGoalPosition(UBYTE id, int &position, SerialComm &comm);
    
    /**
     * @brief Read current goal position for a bunch of servos
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * If goal position is out of the range, an Angle Limit Error Bit is returned and an Alarm is triggered as set in Alarm LED/Shutdown.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1E
     * @param ids servo ids
     * @param positions[out] current goal position [0, 1023]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readGoalPosition(std::vector<UBYTE> ids, std::vector<int> &positions, SerialComm &comm);
    
    /**
     * @brief Set goal position of a single servo
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1E
     * @param id servo id
     * @param position new goal position [0, 1023]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setGoalPosition(UBYTE id, int position, SerialComm &comm);
    
    /**
     * @brief Set goal position for a bunch of servos
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1E
     * @param ids servo id vector
     * @param positions new goal positions according to the servos stored in the servo \c ids vector
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setGoalPosition(std::vector<UBYTE> ids, std::vector<int> positions, SerialComm &comm);

    UBYTE readGoalSpeed(UBYTE id, int &speed, SerialComm &comm);
    UBYTE readGoalSpeed(std::vector<UBYTE> ids, std::vector<int> &speeds, SerialComm &comm);
    UBYTE setGoalSpeed(UBYTE id, int speed, SerialComm &comm);
    UBYTE setGoalSpeed(std::vector<UBYTE> ids, std::vector<int> speeds, SerialComm &comm);

    ///@}
    
    
    
    UBYTE readTorqueLimit(UBYTE id, int &limit, SerialComm &comm);
    UBYTE readTorqueLimit(std::vector<UBYTE> ids, std::vector<int> &limits, SerialComm &comm);
    UBYTE setTorqueLimit(UBYTE id, int limit, SerialComm &comm);

    UBYTE readPresentPosition(UBYTE id,int &position, SerialComm &comm);
    UBYTE readPresentPosition(std::vector<UBYTE> ids, std::vector<int> &positions, SerialComm &comm);

    UBYTE readPresentSpeed(UBYTE id, int &speed, SerialComm &comm);
    UBYTE readPresentSpeed(std::vector<UBYTE> ids, std::vector<int> &speeds, SerialComm &comm);

    UBYTE readPresentLoad(UBYTE id, int &load, SerialComm &comm);
    UBYTE readPresentLoad(std::vector<UBYTE> ids, std::vector<int> &loads, SerialComm &comm);

    UBYTE readPresentVoltage(UBYTE id,int &voltage, SerialComm &comm);
    UBYTE readPresentVoltage(std::vector<UBYTE> ids, std::vector<int> &voltage, SerialComm &comm);
    UBYTE readPresentTemperature(UBYTE id,int &temperature, SerialComm &comm);
    UBYTE readPresentTemperature(std::vector<UBYTE> ids, std::vector<int> &temperature, SerialComm &comm);

    UBYTE readRegistered(UBYTE id,int &registered, SerialComm &comm);
    UBYTE readRegistered(std::vector<UBYTE> ids, std::vector<int> &registereds, SerialComm &comm);

    UBYTE readMoving(UBYTE id,int &moving, SerialComm &comm);
    UBYTE readMoving(std::vector<UBYTE> ids, std::vector<int> &movings, SerialComm &comm);

    UBYTE readLock(UBYTE id,int &lock, SerialComm &comm);
    UBYTE readLock(std::vector<UBYTE> ids, std::vector<int> &locks, SerialComm &comm);
    UBYTE setLock(UBYTE id,UBYTE lock, SerialComm &comm);

    UBYTE readPunch(UBYTE id, int &punch, SerialComm &comm);
    UBYTE readPunch(std::vector<UBYTE> ids, std::vector<int> &punchs, SerialComm &comm);
    UBYTE setPunch(UBYTE id,int punch, SerialComm &comm);

    UBYTE ping(UBYTE id, SerialComm &comm);

    void printBytes(std::vector<UBYTE> &data);

    /*
     * These commands read several registers at once to reduce communication overhead.
     * In principle it is possible to combine arbitrary commands as long as their registers are in a row.
     * In this case only a few commands that can benefit from this technique are implemented.
     *
     * The servos or the arbotix board seem to have register borders that are not allowed to cross while writing
     * So it is only possible to write beyond one register when they belong together (conceptualy)
     */

    UBYTE readAngleLimit(UBYTE id, int &cw, int &ccw, SerialComm &comm);
    UBYTE readAngleLimit(std::vector<UBYTE> ids, std::vector<int> &cws, std::vector<int> &ccws, SerialComm &comm);

    UBYTE readServoCompliance(ServoCompliance &comp, SerialComm &comm);
    UBYTE readServoCompliance(std::vector<ServoCompliance> &comps, SerialComm &comm);

    UBYTE readServoTarget(ServoTarget &target, SerialComm &comm);
    UBYTE readServoTarget(std::vector<ServoTarget> &targets, SerialComm &comm);

    UBYTE readServoStatus(ServoStatus &status, SerialComm &comm);
    UBYTE readServoStatus(std::vector<ServoStatus> &status, SerialComm &comm);

private:

    bool checkChecksum(std::vector<UBYTE> &data);
    void checkError(UBYTE error);
    bool checkParameterRange(int &param, int low, int high, bool exception = true);
    bool checkParameterRange(UBYTE &param, int low, int high, bool exception = true);
    int calculateSpeed(UBYTE hiByte, UBYTE loByte);
    int calculateLoad(UBYTE hiByte, UBYTE loByte);

    void appendData(std::vector<UBYTE> &data, std::vector<int> rawData, bool isLowHigh);

    std::vector<UBYTE> makeSinglePackage(UBYTE id, UBYTE inst, std::vector<UBYTE> data);
    std::vector<UBYTE> makeMultiReadPackage(std::vector<UBYTE> ids, UBYTE addr, UBYTE len);
    std::vector<UBYTE> makeMultiWritePackage(std::vector<UBYTE> ids, UBYTE addr, std::vector<UBYTE> data, UBYTE nBytes);
};

#endif // PXPROTOCOL_H
