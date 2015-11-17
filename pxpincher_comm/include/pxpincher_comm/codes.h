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

#ifndef CODES_H
#define CODES_H

namespace pxpincher
{


//Typedef
using UBYTE = unsigned char; //8bit unsigned
using SBYTE = signed char; //8bit signed
using UWORD = unsigned short int; //16bit unsigned
using SWORD = signed short int; //16bit signed
using UDWORD = unsigned int; //32bit unsigned
using SDWORD = signed int; //32bit signed
using ULONG = unsigned long int; //64bit unsigned
using SLONG = signed long int; //64bit signed


//USB Device

#define ARBOTIX_VENDOR_ID 0x0403
#define ARBOTIX_PRODUCT_ID 0x6001

//Instructions

#define DYNAMIXEL_PREFIX 0xFF

#define DYNAMIXEL_PING 0x01
#define DYNAMIXEL_READ_DATA 0x02
#define DYNAMIXEL_WRITE_DATA 0x03
#define DYNAMIXEL_SYNC_READ 0x84
#define DYNAMIXEL_SYNC_WRITE 0x83

#define DYNAMIXEL_BROADCAST 0xFE

#define DYNAMIXEL_NO_DATA_RESPONSE 6

//Registers (EEPROM)

#define DYNAMIXEL_MODEL_NUMBER_L 0x00
#define DYNAMIXEL_MODEL_NUMBER_H 0x01
#define DYNAMIXEL_FIRMWARE 0x02
#define DYNAMIXEL_SERVO_ID 0x03
#define DYNAMIXEL_BAUDRATE 0x04
#define DYNAMIXEL_RETURN_DELAY 0x05

#define DYNAMIXEL_CW_LIMIT_L 0x06
#define DYNAMIXEL_CW_LIMIT_H 0x07
#define DYNAMIXEL_CCW_LIMIT_L 0x08
#define DYNAMIXEL_CCW_LIMIT_H 0x09

#define DYNAMIXEL_TEMPERATURE_LIMIT 0x0B
#define DYNAMIXEL_VOLTAGE_LOW_LIMIT 0x0C
#define DYNAMIXEL_VOLTAGE_HIGH_LIMIT 0x0D

#define DYNAMIXEL_MAX_TORQUE_L 0x0E
#define DYNAMIXEL_MAX_TORQUE_H 0x0F

#define DYNAMIXEL_RETURN_LEVEL 0x10
#define DYNAMIXEL_ALARM_LED 0x11
#define DYNAMIXEL_ALARM_SHUTDOWN 0x12

// Registers (RAM)

#define DYNAMIXEL_TORQUE_ENDABLE 0x18
#define DYNAMIXEL_LED_ENABLE 0x19

#define DYNAMIXEL_CW_COMP_MARGIN 0x1A
#define DYNAMIXEL_CCW_COMP_MARGIN 0x1B
#define DYNAMIXEL_CW_COMP_SLOPE 0x1C
#define DYNAMIXEL_CCW_COMP_SLOPE 0x1D

#define DYNAMIXEL_GOAL_POSITION_L 0x1E
#define DYNAMIXEL_GOAL_POSITION_H 0x1F
#define DYNAMIXEL_GOAL_SPEED_L 0x20
#define DYNAMIXEL_GOAL_SPEED_H 0x21

#define DYNAMIXEL_TORQUE_LIMIT_L 0x22
#define DYNAMIXEL_TORQUE_LIMIT_H 0x23

#define DYNAMIXEL_PRESENT_POSITION_L 0x24
#define DYNAMIXEL_PRESENT_POSITION_H 0x25
#define DYNAMIXEL_PRESENT_SPEED_L 0x26
#define DYNAMIXEL_PRESENT_SPEED_H 0x27
#define DYNAMIXEL_PRESENT_LOAD_L 0x28
#define DYNAMIXEL_PRESENT_LOAD_H 0x29

#define DYNAMIXEL_PRESENT_VOLTAGE 0x2A
#define DYNAMIXEL_PRESENT_TEMPERATUR 0x2B

#define DYNAMIXEL_REGISTERED 0x2C
#define DYNAMIXEL_MOVING 0x2E

#define DYNAMIXEL_LOCK_EEPROM 0x2F

#define DYNAMIXEL_PUNCH_L 0x30
#define DYNAMIXEL_PUNCH_H 0x31

// Error codes
#define ERROR_VOLTAGE_EXCEPTION 1<<0
#define ERROR_OUT_OF_LIMITS 1<<1
#define ERROR_SERVO_TOO_HOT 1<<2
#define ERROR_INSTRUCTION_OUT_OF_RANGE 1<<3
#define ERROR_CHECKSUM_ERROR 1<<4
#define ERROR_TOO_MUCH_LOAD 1<<5
#define ERROR_UNDEFINED_INSTRUCTION 1<<6
#define ERROR_UNKNOWN_ERROR 1<<7

} // end namespace pxpincher

#endif // CODES_H
