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

#include <chrono>
#include <thread>

//#define PRINT_BYTES


namespace pxpincher
{

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
    UBYTE readModelNumber(const std::vector<UBYTE>& ids, std::vector<int> &numbers, SerialComm &comm);

    
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
    UBYTE readFirmwareVersion(const std::vector<UBYTE>& ids, std::vector<int> &versions, SerialComm &comm);

    
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
    UBYTE readBaudrate(const std::vector<UBYTE>& ids, std::vector<int> &baudrates, SerialComm &comm);
    
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
     * @param delays return-time-delay vector according to the servos \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readReturnDelay(const std::vector<UBYTE>& ids, std::vector<int> &delays, SerialComm &comm);

    /**
     * @brief Read status flag, if the eeprom area of a single servo is locked
     * 
     * if \c lock == 0, the EEPROM area can be modified, otherwise not.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_2F
     * @param id servo id
     * @param lock[out] lock value [0: unlocked, 1: locked]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readLock(UBYTE id,int &lock, SerialComm &comm);
    
    /**
     * @brief Read status flag, if the eeprom areas of a bunch of servos are locked
     * 
     * if \c lock == 0, the EEPROM area can be modified, otherwise not.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_2F
     * @param ids servo id vector
     * @param locks[out] vector of lock values [0: unlocked, 1: locked]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readLock(const std::vector<UBYTE>& ids, std::vector<int> &locks, SerialComm &comm);
    
    /**
     * @brief Lock or unlock the status of an eeprom area
     * 
     * if \c lock == 0, the EEPROM area can be modified, otherwise not.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_2F
     * @warning If lock is set to 1, the power must be turned off and then turned on again to change into 0.
     * @param id servo id
     * @param lock lock value [0: unlocked, 1: locked]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setLock(UBYTE id, UBYTE lock, SerialComm &comm);

    /**
     * @brief Read punch (current to drive motor is at minimum) of a single servo
     * 
     * Value range [0x20, 0x3FF]
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_30
     * @param id servo id
     * @param punch[out] punch value
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readPunch(UBYTE id, int &punch, SerialComm &comm);
    
    /**
     * @brief Read punch (current to drive motor is at minimum) for a bunch of servos
     * 
     * Value range [0x20, 0x3FF]
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_30
     * @param ids servo id vector
     * @param punchs[out] punch value according to the servos in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readPunch(const std::vector<UBYTE>& ids, std::vector<int> &punchs, SerialComm &comm);
    
    /**
     * @brief Set punch (current to drive motor is at minimum) of a single servo
     * 
     * Value range [0x20, 0x3FF]
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_30
     * @param id servo id
     * @param punch punch value
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setPunch(UBYTE id, int punch, SerialComm &comm);
    
    
    /**
     * @brief Set status return level
     * 
     * Specify how to return a Status Packet:
     * Value 0: No return against all commands (Except PING)
     * Value 1: Return only for the READ command
     * Value 2: Return for all commands
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_10
     * @remarks In case of an instruction packet sent via the broadcast id,
     *          a status packet is not returned regardless of the status return level.
     * @param id servo id
     * @param level status return level [0,1,2]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setReturnLevel(UBYTE id, UBYTE level, SerialComm &comm);

    /**
     * @brief Set alarm led level of a single servo
     * 
     * See description of setAlarmShutdown() for details about the \c level
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param id servo id
     * @param level alarm level
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setAlarmLED(UBYTE id, UBYTE level, SerialComm &comm);
    
    /**
     * @brief Read alarm led level of a single servo
     * 
     * See description of readAlarmShutdown() for details about the \c level
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param id servo id
     * @param[out] level alarm level
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readAlarmLED(UBYTE id, int &level, SerialComm &comm);
    
    /**
     * @brief Read alarm led level for a bunch of servos
     * 
     * See description of readAlarmShutdown() for details about the \c level
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param ids servo id vector
     * @param[out] levels vector containins alarm levels for each servo provided in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readAlarmLED(const std::vector<UBYTE>& ids, std::vector<int> &levels, SerialComm &comm);

    /**
     * @brief Set alarm shutdown mode for a single servo
     * 
     * The dynamixel servos can protect themselves by detecting errors.
     * The errors can be set according to:
     * Bit 7: 0
     * Bit 6: Instruction Error - If undefined Instruction is transmitted or the Action command is delivered without the reg_write command
     * Bit 5: Overload Error - If the current load cannot be controller with the set maximum torque
     * Bit 4: Checksum Error - If the checksum of the transmitted instruction packet is invalid
     * Bit 3: Range Error - If the command is given beyond the range of usage
     * Bit 2: OverHeating Error - If the internal temperature is out of the range of operating temperature
     * Bit 1: Angle Limit Error - If Goal Position is outsite the interval specified by CW Angle Limit and CCW Angle Limit.
     * Bit 0: Input Voltage Error - If the applied voltage is out of range of the specified operating voltage.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param id servo id
     * @param level alarm levels
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setAlarmShutdown(UBYTE id, UBYTE level, SerialComm &comm);
    
    /**
     * @brief Read alarm shutdown mode of a single servo
     * 
     * The dynamixel servos can protect themselves by detecting errors.
     * Refer to readAlarmShutdown() for possible modes/levels.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param id servo id
     * @param[out] level alarm levels
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readAlarmShutdown(UBYTE id, int &level, SerialComm &comm);
    
    /**
     * @brief Read alarm shutdown mode for a bunch of servos
     * 
     * The dynamixel servos can protect themselves by detecting errors.
     * Refer to readAlarmShutdown() for possible modes/levels.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param ids servo id vector
     * @param[out] levels alarm levels for each servo specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readAlarmShutdown(const std::vector<UBYTE>& ids, std::vector<int> &levels, SerialComm &comm);

    /**
     * @brief Read torque state of a single servo (check if torque is enabled)
     * 
     * Two different states are possible:
     * State 0: Keeps toque from generating by interrupting the power of motor
     * State 1: Generates Torque by impressing the power to the motor.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param id servo id
     * @param[out] state torque state
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readTorqueState(UBYTE id, int &state, SerialComm &comm);
    
    /**
     * @brief Read torque state for a bunch of servos (check if torque is enabled)
     * 
     * Two different states are possible:
     * State 0: Keeps toque from generating by interrupting the power of motor
     * State 1: Generates Torque by impressing the power to the motor.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param ids servo id vector
     * @param[out] states torque states according to the specified servos in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readTorqueState(const std::vector<UBYTE>& ids, std::vector<int> &states, SerialComm &comm);
    
    /**
     * @brief Set torque state of a single servo
     * 
     * Two different states are possible:
     * State 0: Keeps toque from generating by interrupting the power of motor
     * State 1: Generates Torque by impressing the power to the motor.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param id servo id
     * @param state torque state
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setTorqueState(UBYTE id, UBYTE state, SerialComm &comm);


    /**
     * @brief Set torque state for a bunch of servos
     *
     * Two different states are possible:
     * State 0: Keeps toque from generating by interrupting the power of motor
     * State 1: Generates Torque by impressing the power to the motor.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param ids servo id vector
     * @param states state vector with torque states according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setTorqueState(const std::vector<UBYTE>& ids, std::vector<int> states, SerialComm &comm);

    
    /**
     * @brief Read led state of a single servo
     * 
     * The state is either 0 (off) or 1 (on).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param id servo id
     * @param[out] state led state (0,1) corresponding to (off, on)
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readLEDState(UBYTE id, int &state, SerialComm &comm);
    
    /**
     * @brief Read led state for a bunch of servos
     * 
     * The state is either 0 (off) or 1 (on).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param ids servo id vector
     * @param[out] states led states (0,1) corresponding to (off, on)
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readLEDState(const std::vector<UBYTE>& ids, std::vector<int> &states, SerialComm &comm);
    
    /**
     * @brief Set led state of a single servo
     * 
     * The state is either 0 (off) or 1 (on).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_11
     * @param id servo id
     * @param state led state (0,1) corresponding to (off, on)
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setLEDState(UBYTE id, UBYTE state, SerialComm &comm);
    
    /**
     * @brief Ping a single servo
     * 
     * @param id servo id
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE ping(UBYTE id, SerialComm &comm);

    /**
     * @brief Print a vector of bytes using the ros info console output.
     * @param data vector of bytes
     */
    static void printBytes(const std::vector<UBYTE> &data);
    
    /**
     * @brief Analyze an error byte and print related error messages (if occured)
     * @param error error byte
     * @param silent if \c false print error messages, otherwise supress.
     * @return \c false, if no error is found, \c true otherwise
     */
    static bool checkError(UBYTE error, bool silent = false);
    
    ///@}
    
    
    /** @name Limits */
    ///@{
        
    /**
     * @brief Read angle limits both clockwise and counter-clockwise of a single servo
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_06
     * @remarks This method is faster than reading the angles separately, since registers are read with a single read-call.
     * @param id servo id
     * @param[out] cw clockwise angle limit
     * @param[out] ccw counter-clockwise angle limit
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readAngleLimit(UBYTE id, int &cw, int &ccw, SerialComm &comm);
    
     /**
     * @brief Read angle limits both clockwise and counter-clockwise for a bunch of servos
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_06
     * @remarks This method is faster than reading the angles separately, since registers are read with a single read-call.
     * @param ids servo id vector
     * @param[out] cw clockwise angle limits according to servos specified in \c ids
     * @param[out] ccw counter-clockwise angle limits according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readAngleLimit(const std::vector<UBYTE>& ids, std::vector<int> &cws, std::vector<int> &ccws, SerialComm &comm);
        
    
    /**
     * @brief Read clockwise angle limit of a single servo
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_06
     * @param id servo id
     * @param[out] limit clockwise angle limit
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readCWAngleLimit(UBYTE id,int &limit, SerialComm &comm);
    
    /**
     * @brief Read clockwise angle limits for a bunch of servos
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_06
     * @param ids servo id vector
     * @param[out] limits clockwise angle limits according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readCWAngleLimit(const std::vector<UBYTE>& ids, std::vector<int> &limits, SerialComm &comm);
    
    /**
     * @brief Read counter-clockwise angle limit of a single servo
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_06
     * @param id servo id
     * @param[out] limit counter-clockwise angle limit
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readCCWAngleLimit(UBYTE id,int &limit, SerialComm &comm);
    
    /**
     * @brief Read counter-clockwise angle limits for a bunch of servos
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_06
     * @param ids servo id vector
     * @param[out] limits counter-clockwise angle limits according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readCCWAngleLimit(const std::vector<UBYTE>& ids, std::vector<int> &limits, SerialComm &comm);

    
    /**
     * @brief Set clockwise angle limit of a single servo
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_06
     * @param id servo id
     * @param limit clockwise angle limit
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setCWAngleLimit(UBYTE id, int limit, SerialComm &comm);
    
    /**
     * @brief Set clockwise angle limits for a bunch of servos
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_06
     * @param ids servo id vector
     * @param limits clockwise angle limits according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setCWAngleLimit(const std::vector<UBYTE>& ids, std::vector<int> limits, SerialComm &comm);

    
    /**
     * @brief Set counter-clockwise angle limit of a single servo
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_06
     * @param id servo id
     * @param limit counter-clockwise angle limit
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setCCWAngleLimit(UBYTE id, int limit, SerialComm &comm);
    
    /**
     * @brief Set counter-clockwise angle limits for a bunch of servos
     * 
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_06
     * @param ids servo id vector
     * @param limits counter-clockwise angle limits according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setCCWAngleLimit(const std::vector<UBYTE>& ids, std::vector<int> limits, SerialComm &comm);
   
    /**
     * @brief Read the maximum temperature limit of a single servo
     * 
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_0B
     * @param id servo id
     * @param[out] limit maximum temperature limit [°C]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readTemperatureLimit(UBYTE id, int &limit, SerialComm &comm);
    
    /**
     * @brief Read the maximum temperature limits for a bunch of servos
     * 
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_0B
     * @param ids servo id vector
     * @param[out] limits maximum temperature limits according to servos specified in \c ids  [°C]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readTemperatureLimit(const std::vector<UBYTE>& ids, std::vector<int> &limits, SerialComm &comm);
    
    /**
     * @brief Set the maximum temperature limit of a single servo
     * 
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_0B
     * @warning Do not set the temperature other than the default value!
     *          If the temperature alarm shutdown occurs, wait 20min to cool the temperature before re-use!
     * @param id servo id
     * @param limit maximum temperature limit  [°C]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setTemperatureLimit(UBYTE id, UBYTE limit, SerialComm &comm);

    /**
     * @brief Read low voltage limit of a single servo
     * The operation range is [50, 250] (0x32 ~ 0x96). The unit is 0.1V.
     * E.g. a value of 80 corresponds to 8V.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_0C
     * @param id servo id
     * @param[out] limit lower voltage limit
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readLowVoltageLimit(UBYTE id, int &limit, SerialComm &comm);
    
    /**
     * @brief Read low voltage limit for a bunch of servos
     * The operation range is [50, 250] (0x32 ~ 0x96). The unit is 0.1V.
     * E.g. a value of 80 corresponds to 8V.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_0C
     * @param ids servo id vector
     * @param[out] limits lower voltage limits according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readLowVoltageLimit(const std::vector<UBYTE>& ids, std::vector<int> &limits, SerialComm &comm);
    
    /**
     * @brief Set low voltage limit of a single servo
     * The operation range is [50, 250] (0x32 ~ 0x96). The unit is 0.1V.
     * E.g. a value of 80 corresponds to 8V.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_0C
     * @param ids servo id vector
     * @param limit lower voltage limit
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setLowVoltageLimit(UBYTE id, UBYTE limit, SerialComm &comm);

    /**
     * @brief Read upper/higher voltage limit of a single servo
     * The operation range is [50, 250] (0x32 ~ 0x96). The unit is 0.1V.
     * E.g. a value of 80 corresponds to 8V.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_0C
     * @param id servo id
     * @param[out] limit higher voltage limit
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readHighVoltageLimit(UBYTE id, int &limit, SerialComm &comm);
    
    /**
     * @brief Read upper/higher voltage limit for a bunch of servos
     * The operation range is [50, 250] (0x32 ~ 0x96). The unit is 0.1V.
     * E.g. a value of 80 corresponds to 8V.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_0C
     * @param ids servo id vector
     * @param[out] limits higher voltage limits according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readHighVoltageLimit(const std::vector<UBYTE>& ids, std::vector<int> &limits, SerialComm &comm);
    
    /**
     * @brief Set higher/upper voltage limit of a single servo
     * The operation range is [50, 250] (0x32 ~ 0x96). The unit is 0.1V.
     * E.g. a value of 80 corresponds to 8V.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_0C
     * @param ids servo id vector
     * @param limit higher voltage limit
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setHighVoltageLimit(UBYTE id, UBYTE limit, SerialComm &comm);

    /**
     * @brief Read torque value of maximum output of a single servo
     * The valid range is [0, 1023] (0x3FF). The unit is 0.1%.
     * E.g. data 1023 (0x3FF) means that Dynamixel will use 100% of the maximum torque it can produce, 512 corresponds to 50% etc.
     * When the power is turned on, Torque Limit (see readTorqueLimit()) uses this value as initial value. 
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_0E
     * @param id servo id
     * @param[out] max max torque limit [0, 1023]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readTorqueMax(UBYTE id, int &max, SerialComm &comm);
    
    /**
     * @brief Read torque value of maximum output for a bunch of servos
     * The valid range is [0, 1023] (0x3FF). The unit is 0.1%.
     * E.g. data 1023 (0x3FF) means that Dynamixel will use 100% of the maximum torque it can produce, 512 corresponds to 50% etc.
     * When the power is turned on, Torque Limit (see readTorqueLimit()) uses this value as initial value. 
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_0E
     * @param ids servo id vector
     * @param[out] maxs max torque limits [0, 1023] according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readTorqueMax(const std::vector<UBYTE>& ids, std::vector<int> &maxs, SerialComm &comm);
    
    /**
     * @brief Set torque value of maximum output of a single servo
     * The valid range is [0, 1023] (0x3FF). The unit is 0.1%.
     * E.g. data 1023 (0x3FF) means that Dynamixel will use 100% of the maximum torque it can produce, 512 corresponds to 50% etc.
     * When the power is turned on, Torque Limit (see readTorqueLimit()) uses this value as initial value. 
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_0E
     * @param id servo id
     * @param limit max torque limit [0, 1023]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setTorqueMax(UBYTE id, int limit, SerialComm &comm);
    
    /**
     * @brief Read maximum torque limit of a single servo
     * 
     * The allowed range is [0, 1023]. The unit is about 0.1%.
     * E.g. a value of 512 corresponds to 50%.
     * If the power is turned on, the value of TorqueMax will be used as initial value.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_22
     * @remarks If an alarm shutdown is triggered, the motor loses its torque. At this moment, if the value is changed to
     *          the value other than 0, the motor can be used again.
     * @param id servo id
     * @param[out] limit current torque limit
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readTorqueLimit(UBYTE id, int &limit, SerialComm &comm);
    
    /**
     * @brief Read maximum torque limit for a bunch of servos
     * 
     * The allowed range is [0, 1023]. The unit is about 0.1%.
     * E.g. a value of 512 corresponds to 50%.
     * If the power is turned on, the value of TorqueMax will be used as initial value.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_22
     * @remarks If an alarm shutdown is triggered, the motor loses its torque. At this moment, if the value is changed to
     *          the value other than 0, the motor can be used again.
     * @param ids servo id vector
     * @param[out] limits current torque limits according to the servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readTorqueLimit(const std::vector<UBYTE>& ids, std::vector<int> &limits, SerialComm &comm);
    
    /**
     * @brief Set maximum torque limit of a single servo
     * 
     * The allowed range is [0, 1023]. The unit is about 0.1%.
     * E.g. a value of 512 corresponds to 50%.
     * If the power is turned on, the value of TorqueMax will be used as initial value.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_22
     * @remarks If an alarm shutdown is triggered, the motor loses its torque. At this moment, if the value is changed to
     *          the value other than 0, the motor can be used again.
     * @param id servo id
     * @param limit current torque limit
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setTorqueLimit(UBYTE id, int limit, SerialComm &comm);
    
    ///@}    
    
    
    
    /** @name Compliance margin and slope 
      * @details For more information on compliance margins and slopes refer to
      *          http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1A
      */
    //@{
  
    UBYTE readServoCompliance(ServoCompliance &comp, SerialComm &comm);
    UBYTE readServoCompliance(std::vector<ServoCompliance> &comps, SerialComm &comm);
    UBYTE readCWComplianceMargin(UBYTE id, int &margin, SerialComm &comm);
    UBYTE readCWComplianceMargin(const std::vector<UBYTE>& ids, std::vector<int> &margins, SerialComm &comm);
    UBYTE setCWComplianceMargin(UBYTE id, UBYTE margin, SerialComm &comm);
    UBYTE readCCWComplianceMargin(UBYTE id, int &margin, SerialComm &comm);
    UBYTE readCCWComplianceMargin(const std::vector<UBYTE>& ids, std::vector<int> &margins, SerialComm &comm);
    UBYTE setCCWComplianceMargin(UBYTE id, UBYTE margin, SerialComm &comm);
    UBYTE readCWComplianceSlope(UBYTE id, int &slope, SerialComm &comm);
    UBYTE readCWComplianceSlope(const std::vector<UBYTE>& ids, std::vector<int> &slopes, SerialComm &comm);
    UBYTE setCWComplianceSlope(UBYTE id, UBYTE slope, SerialComm &comm);
    UBYTE readCCWComplianceSlope(UBYTE id, int &slope, SerialComm &comm);
    UBYTE readCCWComplianceSlope(const std::vector<UBYTE>& ids, std::vector<int> &slopes, SerialComm &comm);
    UBYTE setCCWComplianceSlope(UBYTE id, UBYTE slope, SerialComm &comm);
    UBYTE setComplianceMargin(UBYTE id, UBYTE cw, UBYTE ccw, SerialComm &comm);

    UBYTE setComplianceSlope(UBYTE id, UBYTE cw, UBYTE ccw, SerialComm &comm);
    UBYTE setComplianceSlope(std::vector<UBYTE> ids, std::vector<int> cws, std::vector<int> ccws, SerialComm &comm);



    //@}
    
    
    /** @name Goal position, speed and torque */
    ///@{
    
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
    UBYTE readGoalPosition(const std::vector<UBYTE>& ids, std::vector<int> &positions, SerialComm &comm);
    
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
    UBYTE setGoalPosition(const std::vector<UBYTE>& ids, const std::vector<int>& positions, SerialComm &comm);


    /**
     * @brief Set goal position and speed for a bunch of servos
     *
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * The goal speed is encoded in the range [0, 2047].
     * If the value is in [0, 1023], the motor rotates to the CCW direction.
     * If the value is in [1024, 2047], the motor rotates to the CW direction.
     * That is, the 10th bit becomes the direction bit to control the direction. 0 and 1024 are equal.
     * The unit of this value varies depending on operating mode:
     *  - Joint mode: Unit is about 0.111rpm
     *  - Wheel mode: Unit is about 0.1% (wheel mode is not suited for robot arms).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1E
     * @param ids servo id vector
     * @param positions new goal positions according to the servos stored in the servo \c ids vector
     * @param speeds new goal speeds according to the servos stored in the servo \c ids vector
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setGoalPositionAndSpeed(const std::vector<UBYTE>& ids, const std::vector<int>& positions,const std::vector<int>& speeds, SerialComm &comm);

    /**
     * @brief Set goal position and speed for a single servo
     *
     * The goal position is encoded in the range [0, 1023] (corresponding to [0°, 300°]).
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * The goal speed is encoded in the range [0, 2047].
     * If the value is in [0, 1023], the motor rotates to the CCW direction.
     * If the value is in [1024, 2047], the motor rotates to the CW direction.
     * That is, the 10th bit becomes the direction bit to control the direction. 0 and 1024 are equal.
     * The unit of this value varies depending on operating mode:
     *  - Joint mode: Unit is about 0.111rpm
     *  - Wheel mode: Unit is about 0.1% (wheel mode is not suited for robot arms).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1E
     * @param id servo id
     * @param position new goal position
     * @param speeds new goal speed
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setGoalPositionAndSpeed(UBYTE id, int position,int speed, SerialComm &comm);

    /**
     * @brief Read current goal speed of a single servo
     * 
     * The goal speed is encoded in the range [0, 2047].
     * If the value is in [0, 1023], the motor rotates to the CCW direction.
     * If the value is in [1024, 2047], the motor rotates to the CW direction.
     * That is, the 10th bit becomes the direction bit to control the direction. 0 and 1024 are equal.
     * The unit of this value varies depending on operating mode:
     *  - Joint mode: Unit is about 0.111rpm
     *  - Wheel mode: Unit is about 0.1% (wheel mode is not suited for robot arms).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1E
     * @param id servo id
     * @param[out] speed current goal speed [0, 2047]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readGoalSpeed(UBYTE id, int &speed, SerialComm &comm);
    
    /**
     * @brief Read current goal speed for a bunch of servos
     * 
     * The goal speed is encoded in the range [0, 2047].
     * If the value is in [0, 1023], the motor rotates to the CCW direction.
     * If the value is in [1024, 2047], the motor rotates to the CW direction.
     * That is, the 10th bit becomes the direction bit to control the direction. 0 and 1024 are equal.
     * The unit of this value varies depending on operating mode:
     *  - Joint mode: Unit is about 0.111rpm
     *  - Wheel mode: Unit is about 0.1% (wheel mode is not suited for robot arms).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1E
     * @param ids servo id vector
     * @param[out] speeds current goal speeds [0, 2047] according to the servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readGoalSpeed(const std::vector<UBYTE>& ids, std::vector<int> &speeds, SerialComm &comm);
    
    /**
     * @brief Set goal speed of a single servo
     * 
     * The goal speed is encoded in the range [0, 2047].
     * If the value is in [0, 1023], the motor rotates to the CCW direction.
     * If the value is in [1024, 2047], the motor rotates to the CW direction.
     * That is, the 10th bit becomes the direction bit to control the direction. 0 and 1024 are equal.
     * The unit of this value varies depending on operating mode:
     *  - Joint mode: Unit is about 0.111rpm
     *  - Wheel mode: Unit is about 0.1% (wheel mode is not suited for robot arms).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1E
     * @param id servo id
     * @param speed current goal speed [0, 2047]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setGoalSpeed(UBYTE id, int speed, SerialComm &comm);
    
    /**
     * @brief Set goal speed for a bunch of servos
     * 
     * The goal speed is encoded in the range [0, 2047].
     * If the value is in [0, 1023], the motor rotates to the CCW direction.
     * If the value is in [1024, 2047], the motor rotates to the CW direction.
     * That is, the 10th bit becomes the direction bit to control the direction. 0 and 1024 are equal.
     * The unit of this value varies depending on operating mode:
     *  - Joint mode: Unit is about 0.111rpm
     *  - Wheel mode: Unit is about 0.1% (wheel mode is not suited for robot arms).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_1E
     * @param ids servo id vector
     * @param[out] speed current goal speeds [0, 2047] according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE setGoalSpeed(const std::vector<UBYTE>& ids, const std::vector<int>& speeds, SerialComm &comm);

    ///@}
    
    
    
    /** @name present states */
    ///@{
    
    /**
     * @brief Read current target/goal of a single servo
     * 
     * @see ServoTarget
     * @param[in,out] target ServoTarget object (the ID must be provided within status)
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readServoTarget(ServoTarget &target, SerialComm &comm);
    
    /**
     * @brief Read current target/goal for a bunch of servos
     * 
     * @see ServoTarget
     * @param[in,out] targets vector of ServoTarget objects (a single target for each servo);  (the ID must be provided for each status)
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readServoTarget(std::vector<ServoTarget> &targets, SerialComm &comm);

    /**
     * @brief Read current status of a single servo
     * 
     * @see ServoStatus
     * @param[in,out] status ServoStatus object (the ID must be provided within status)
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readServoStatus(ServoStatus &status, SerialComm &comm);
    
    /**
     * @brief Read current status for a bunch of servos
     * 
     * @see ServoStatus
     * @param[in,out] status vector of ServoStatus objects (a single status for each servo); (the ID must be provided for each status)
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readServoStatus(std::vector<ServoStatus> &status, SerialComm &comm);
    
    /**
     * @brief Read current position of a single servo
     * 
     * The position is encoded in the range [0, 1023].
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_24
     * @param id servo id
     * @param[out] position current servo position [0, 1023]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readPresentPosition(UBYTE id,int &position, SerialComm &comm);
    
    /**
     * @brief Read current positions for a bunch of servos
     * 
     * The position is encoded in the range [0, 1023].
     * The unit is 0.29 degree. The center point is at 512 (150°).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_24
     * @param ids servo id vector
     * @param[out] positions current servo positions [0, 1023] according to the servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readPresentPosition(const std::vector<UBYTE>& ids, std::vector<int> &positions, SerialComm &comm);

     /**
     * @brief Read current speed of a single servo
     * 
     * The speed is encoded in the range [0, 2047].
     * If the value is in [0, 1023], the motor rotates to the CCW direction.
     * If the value is in [1024, 2047], the motor rotates to the CW direction.
     * That is, the 10th bit becomes the direction bit to control the direction. 0 and 1024 are equal.
     * The unit of this value varies depending on operating mode:
     *  - Joint mode: Unit is about 0.111rpm
     *  - Wheel mode: Unit is about 0.1% (wheel mode is not suited for robot arms).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_26
     * @param id servo id
     * @param[out] speed current servo speed [0, 2047]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readPresentSpeed(UBYTE id, int &speed, SerialComm &comm);
    
    /**
     * @brief Read current speeds for a bunch of servos
     * 
     * The speed is encoded in the range [0, 2047].
     * If the value is in [0, 1023], the motor rotates to the CCW direction.
     * If the value is in [1024, 2047], the motor rotates to the CW direction.
     * That is, the 10th bit becomes the direction bit to control the direction. 0 and 1024 are equal.
     * The unit of this value varies depending on operating mode:
     *  - Joint mode: Unit is about 0.111rpm
     *  - Wheel mode: Unit is about 0.1% (wheel mode is not suited for robot arms).
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_26
     * @param ids servo id vector
     * @param[out] speeds current servo speeds [0, 2047] according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readPresentSpeed(const std::vector<UBYTE>& ids, std::vector<int> &speeds, SerialComm &comm);

    /**
     * @brief Read currently applied load of a single servo
     * 
     * The load is encoded in the range [0, 2047]. The unit is about 0.1%.
     * If the value is in [0, 1023], the load works to the CCW direction.
     * If the value is in [1024, 2047], the load works to the CW direction.
     * That is, the 10th bit becomes the direction bit to control the direction. 0 and 1024 are equal.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_28
     * @param id servo id
     * @param[out] load current servo load [0, 2047]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readPresentLoad(UBYTE id, int &load, SerialComm &comm);
    
    /**
     * @brief Read currently applied load for a bunch of servos
     * 
     * The load is encoded in the range [0, 2047]. The unit is about 0.1%.
     * If the value is in [0, 1023], the load works to the CCW direction.
     * If the value is in [1024, 2047], the load works to the CW direction.
     * That is, the 10th bit becomes the direction bit to control the direction. 0 and 1024 are equal.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_28
     * @param ids servo id vector
     * @param[out] loads current servo loads [0, 2047] according to the servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readPresentLoad(const std::vector<UBYTE>& ids, std::vector<int> &loads, SerialComm &comm);

    /**
     * @brief Read current voltage of a single servo
     * 
     * The value is 10 times larger than the actual voltage. E.g. 10V corresponds to a value of 100.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_2A
     * @param id servo id
     * @param[out] voltage current servo voltage
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readPresentVoltage(UBYTE id,int &voltage, SerialComm &comm);
    
    /**
     * @brief Read current voltages for a bunch of servos
     * 
     * The value is 10 times larger than the actual voltage. E.g. 10V corresponds to a value of 100.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_2A
     * @param ids servo id vector
     * @param[out] voltages current servo voltages according to the servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readPresentVoltage(const std::vector<UBYTE>& ids, std::vector<int> &voltages, SerialComm &comm);
    
    /**
     * @brief Read current temperature of a single servo
     * 
     * The value is identical to the actual temperature in Celsius. E.g. 85°C corresponds to a value of 85.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_2B
     * @param id servo id
     * @param[out] temperature current servo temperature [°C]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readPresentTemperature(UBYTE id, int &temperature, SerialComm &comm);
    
    /**
     * @brief Read current temperature for a bunch of servos
     * 
     * The value is identical to the actual temperature in Celsius. E.g. 85°C corresponds to a value of 85.
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_2B
     * @param ids servo id vector
     * @param[out] temperatures current servo temperature according to the servos specified in \c ids [°C]
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readPresentTemperature(const std::vector<UBYTE>& ids, std::vector<int> &temperatures, SerialComm &comm);
    
    /**
     * @brief Check if there is any registered instruction for a single servo
     * 
     * Two possible states:
     *  - Value 0: There are not commands transmitted by REG_WRITE
     *  - Value 1: There are commands transmitted by REG_WRITE
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_2C
     * @param id servo id
     * @param[out] registered current registered state (0,1)
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readRegistered(UBYTE id, int &registered, SerialComm &comm);
    
    /**
     * @brief Check if there is any registered instruction for a bunch of servos
     * 
     * Two possible states for each servo:
     *  - Value 0: There are not commands transmitted by REG_WRITE
     *  - Value 1: There are commands transmitted by REG_WRITE
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_2C
     * @param ids servo ids
     * @param[out] registereds current registered states (0,1) according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readRegistered(const std::vector<UBYTE>& ids, std::vector<int> &registereds, SerialComm &comm);

    /**
     * @brief Check if there is any movement of a single servo
     * 
     * Two possible states:
     *  - Value 0: Goal position command execution is completed
     *  - Value 1: Goal position command execution is in progress
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_2E
     * @param id servo id
     * @param[out] moving current moving state (0,1)
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readMoving(UBYTE id,int &moving, SerialComm &comm);
    
    /**
     * @brief Check if there is any movement for a bunch of servos
     * 
     * Two possible states for each servo:
     *  - Value 0: Goal position command execution is completed
     *  - Value 1: Goal position command execution is in progress
     * See http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm#Actuator_Address_2E
     * @param ids servo id vector
     * @param[out] moving current moving state (0,1) according to servos specified in \c ids
     * @param comm reference to the related serial communication object
     * @return error/checksum byte
     */
    UBYTE readMoving(const std::vector<UBYTE>& ids, std::vector<int> &movings, SerialComm &comm);

    ///@}
    
    

private:

    /**
     * @brief Check the checksum of a given byte array
     * @param data vector of bytes
     */
    bool checkChecksum(const std::vector<UBYTE> &data) const;
    
    /**
     * @brief Check parameter range and bound param to the range (int version)
     * @todo exception implementation
     * @param param parameter to be checked and modified
     * @param low lower bound
     * @param high upper bound
     * @param exception if \c true, throw an exception
     * @return \c true, if parameter was in the range, \c false otherwise.
     */
    bool checkParameterRange(int &param, int low, int high, bool exception = true) const;
    
    /**
     * @brief Check parameter range and bound param to the range (ubyte version)
     * @todo exception implementation
     * @param param parameter to be checked and modified
     * @param low lower bound
     * @param high upper bound
     * @param exception if \c true, throw an exception
     * @return \c true, if parameter was in the range, \c false otherwise.
     */
    bool checkParameterRange(UBYTE &param, int low, int high, bool exception = true) const;
    
    /**
     * @brief Calculate speed value from 2 bytes
     * @param param parameter to be checked and modified
     * @param hi_byte higher byte
     * @param lo_byte lower byte
     * @return speed value
     */
    int calculateSpeed(UBYTE hi_byte, UBYTE lo_byte) const;
    
    /**
     * @brief Calculate load value from 2 bytes
     * @param param parameter to be checked and modified
     * @param hi_byte higher byte
     * @param lo_byte lower byte
     * @return load value
     */
    int calculateLoad(UBYTE hi_byte, UBYTE lo_byte) const;

    /**
     * @brief Append vector raw_data to data vector containing only bytes
     * @param[in,out] data data vector of bytes which is extended
     * @param raw_data new data vector that should be added to \c data.
     * @param is_low_high set to \c true, if the data is a 16bit value and therefore must be splitted into two bytes.
     */
    void appendData(std::vector<UBYTE> &data, const std::vector<int>& raw_data, bool is_low_high) const;

    /**
     * @brief Construct a message/package for a single servo
     * @param id servo id
     * @param inst byte representing the desired instruction
     * @param data actual data vector
     * @return byte vector of the complete package
     */
    std::vector<UBYTE> makeSinglePackage(UBYTE id, UBYTE inst, const std::vector<UBYTE>& data) const;
    
    /**
     * @brief Construct a message/package for reading from multiple servos
     * @param ids servo id vector
     * @param addr addresses of the registers to read from
     * @param len length in bytes starting from \c addr.
     * @return byte vector of the complete package
     */
    std::vector<UBYTE> makeMultiReadPackage(const std::vector<UBYTE>& ids, UBYTE addr, UBYTE len) const;
    
    /**
     * @brief Construct a message/package for writing to multiple servos
     * @param ids servo id vector
     * @param addr addresses of the registers to write to
     * @param data data vector (bytes)
     * @param n_bytes number of bytes
     * @return byte vector of the complete package
     */
    std::vector<UBYTE> makeMultiWritePackage(const std::vector<UBYTE>& ids, UBYTE addr, const std::vector<UBYTE>& data, UBYTE n_bytes) const;
};


} // end namespace pxpincher

#endif // PXPROTOCOL_H
