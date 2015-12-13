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

#include "pxpincher_comm/pxprotocol.h"

namespace pxpincher
{

PXProtocol::PXProtocol()
{
}

UBYTE PXProtocol::readModelNumber(UBYTE id, int &number, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> numbers = {0};

    // Wrap to multi servo command
    UBYTE error = readModelNumber(ids,numbers,comm);

    // Remap data
    number = numbers.at(0);

    return error;
}

UBYTE PXProtocol::readModelNumber(const std::vector<UBYTE>& ids, std::vector<int> &numbers, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_MODEL_NUMBER_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    numbers.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        numbers.push_back((int) (response.at(i+6) << 8) | response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::readFirmwareVersion(UBYTE id, int &version, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> versions = {0};

    // Wrap to multi servo command
    UBYTE error = readFirmwareVersion(ids,versions,comm);

    // Remap data
    version = versions.at(0);

    return error;
}

UBYTE PXProtocol::readFirmwareVersion(const std::vector<UBYTE>& ids, std::vector<int> &versions, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_FIRMWARE;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    versions.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        versions.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setID(UBYTE id, UBYTE new_id, SerialComm &comm)
{
    if (!checkParameterRange(new_id,0,252)){
        ROS_INFO("Cannot set servo id apart from [0,254]. Skipping ...");
        return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_SERVO_ID;

    data.push_back(reg);
    data.push_back(new_id);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::readBaudrate(UBYTE id, int &baudrate, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> baudrates = {0};

    // Wrap to multi servo command
    UBYTE error = readBaudrate(ids,baudrates,comm);

    // Remap data
    baudrate = baudrates.at(0);

    return error;
}

UBYTE PXProtocol::readBaudrate(const std::vector<UBYTE>& ids, std::vector<int> &baudrates, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_BAUDRATE;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    baudrates.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        baudrates.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setBaudrate(UBYTE id, UBYTE baudrate, SerialComm &comm)
{
    if (!checkParameterRange(baudrate,0,254)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_BAUDRATE;

    data.push_back(reg);
    data.push_back(baudrate);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::setReturnDelay(UBYTE id, UBYTE delay, SerialComm &comm)
{
    if (!checkParameterRange(delay,0,254)){
        ROS_INFO("Cannot set a return delay apart from [0,254]. Bounding ...");
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_RETURN_DELAY;

    data.push_back(reg);
    data.push_back(delay);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::readReturnDelay(UBYTE id, int &delay, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> delays = {0};

    // Wrap to multi servo command
    UBYTE error = readReturnDelay(ids,delays,comm);

    // Remap data
    delay = delays.at(0);

    return error;
}

UBYTE PXProtocol::readReturnDelay(const std::vector<UBYTE>& ids, std::vector<int> &delays, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_RETURN_DELAY;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    delays.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        delays.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}


UBYTE PXProtocol::readLock(UBYTE id, int &lock, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> locks = {0};

    // Wrap to multi servo command
    UBYTE error = readLock(ids,locks,comm);

    // Remap data
    lock = locks.at(0);

    return error;
}

UBYTE PXProtocol::readLock(const std::vector<UBYTE>& ids, std::vector<int> &locks, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_LOCK_EEPROM;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    locks.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        locks.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setLock(UBYTE id, UBYTE lock, SerialComm &comm)
{
    if (!checkParameterRange(lock,0,1)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_LOCK_EEPROM;

    data.push_back(reg);
    data.push_back(lock);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::readPunch(UBYTE id, int &punch, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> punchs = {0};

    // Wrap to multi servo command
    UBYTE error = readPunch(ids,punchs,comm);

    // Remap data
    punch = punchs.at(0);

    return error;
}

UBYTE PXProtocol::readPunch(const std::vector<UBYTE>& ids, std::vector<int> &punchs, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_PUNCH_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    punchs.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        punchs.push_back((int) (response.at(i+6) << 8) | response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setPunch(UBYTE id, int punch, SerialComm &comm)
{
    if (!checkParameterRange(punch,32,1023)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_PUNCH_L;
    UBYTE punch_low = (UBYTE) (punch % 256);
    UBYTE punch_high = (UBYTE) (punch >> 8);

    data.push_back(reg);
    data.push_back(punch_low);
    data.push_back(punch_high);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::ping(UBYTE id, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    package = makeSinglePackage(id,DYNAMIXEL_PING,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    return response[4];
}


UBYTE PXProtocol::readCWAngleLimit(UBYTE id, int &limit, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> limits = {0};

    // Wrap to multi servo command
    UBYTE error = readCWAngleLimit(ids,limits,comm);

    // Remap data
    limit = limits.at(0);

    return error;
}

UBYTE PXProtocol::readCWAngleLimit(const std::vector<UBYTE>& ids, std::vector<int> &limits, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_CW_LIMIT_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    limits.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        limits.push_back((int) (response.at(i+6) << 8) | response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::readCCWAngleLimit(UBYTE id, int &limit, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> limits = {0};

    // Wrap to multi servo command
    UBYTE error = readCCWAngleLimit(ids,limits,comm);

    // Remap data
    limit = limits.at(0);

    return error;
}

UBYTE PXProtocol::readCCWAngleLimit(const std::vector<UBYTE>& ids, std::vector<int> &limits, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_CCW_LIMIT_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    limits.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        limits.push_back((int) (response.at(i+6) << 8) | response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setCWAngleLimit(UBYTE id, int limit, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> limits = {limit};

    // Wrap to multi servo command
    UBYTE error = setCWAngleLimit(ids,limits,comm);

    return error;
}

UBYTE PXProtocol::setCWAngleLimit(const std::vector<UBYTE>& ids, std::vector<int> limits, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_CW_LIMIT_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        appendData(data,limits,true);

        package = makeMultiWritePackage(ids,reg,data,nBytes);

        // Send package and recieve response
        comm.sendData(package);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

        return 0x00;

    }else{
        // Single Servo Mode
        data.push_back(reg);
        appendData(data,limits,true);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_WRITE_DATA,data);

        // Send package and recieve response
        comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

        // Check Checksum
        if(!checkChecksum(response)){
            /// @todo TODO Throw Exception
            ROS_INFO("Checksum mismatch");
        }

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

        // Return Error Byte
        checkError(response[4]);
        return response[4];
    }
}

UBYTE PXProtocol::setCCWAngleLimit(UBYTE id, int limit, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> limits = {limit};

    // Wrap to multi servo command
    UBYTE error = setCCWAngleLimit(ids,limits,comm);

    return error;
}

UBYTE PXProtocol::setCCWAngleLimit(const std::vector<UBYTE>& ids, std::vector<int> limits, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_CCW_LIMIT_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        appendData(data,limits,true);

        package = makeMultiWritePackage(ids,reg,data,nBytes);

        // Send package and recieve response
        comm.sendData(package);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

        return 0x00;

    }else{
        // Single Servo Mode
        data.push_back(reg);
        appendData(data,limits,true);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_WRITE_DATA,data);

        // Send package and recieve response
        comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

        // Check Checksum
        if(!checkChecksum(response)){
            /// @todo TODO Throw Exception
            ROS_INFO("Checksum mismatch");
        }

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

        // Return Error Byte
        checkError(response[4]);
        return response[4];
    }
}

UBYTE PXProtocol::readTemperatureLimit(UBYTE id, int &limit, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> limits = {0};

    // Wrap to multi servo command
    UBYTE error = readTemperatureLimit(ids,limits,comm);

    // Remap data
    limit = limits.at(0);

    return error;
}

UBYTE PXProtocol::readTemperatureLimit(const std::vector<UBYTE>& ids, std::vector<int> &limits, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_TEMPERATURE_LIMIT;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    limits.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        limits.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setTemperatureLimit(UBYTE id, UBYTE limit, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_TEMPERATURE_LIMIT;

    data.push_back(reg);
    data.push_back(limit);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::readLowVoltageLimit(UBYTE id, int &limit, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> limits = {0};

    // Wrap to multi servo command
    UBYTE error = readLowVoltageLimit(ids,limits,comm);

    // Remap data
    limit = limits.at(0);

    return error;
}

UBYTE PXProtocol::readLowVoltageLimit(const std::vector<UBYTE>& ids, std::vector<int> &limits, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_VOLTAGE_LOW_LIMIT;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    limits.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        limits.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setLowVoltageLimit(UBYTE id, UBYTE limit, SerialComm &comm)
{

    if (!checkParameterRange(limit,50,250)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_VOLTAGE_LOW_LIMIT;

    data.push_back(reg);
    data.push_back(limit);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::readHighVoltageLimit(UBYTE id, int &limit, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> limits = {0};

    // Wrap to multi servo command
    UBYTE error = readHighVoltageLimit(ids,limits,comm);

    // Remap data
    limit = limits.at(0);

    return error;
}

UBYTE PXProtocol::readHighVoltageLimit(const std::vector<UBYTE>& ids, std::vector<int> &limits, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_VOLTAGE_HIGH_LIMIT;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    limits.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        limits.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setHighVoltageLimit(UBYTE id, UBYTE limit, SerialComm &comm)
{

    if (!checkParameterRange(limit,50,250)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_VOLTAGE_HIGH_LIMIT;

    data.push_back(reg);
    data.push_back(limit);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::readTorqueMax(UBYTE id, int &max, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> maxs = {0};

    // Wrap to multi servo command
    UBYTE error = readTorqueMax(ids,maxs,comm);

    // Remap data
    max = maxs.at(0);

    return error;
}

UBYTE PXProtocol::readTorqueMax(const std::vector<UBYTE>& ids, std::vector<int> &maxs, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_MAX_TORQUE_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    maxs.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        maxs.push_back((int) (response.at(i+6) << 8) | response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setTorqueMax(UBYTE id, int limit, SerialComm &comm)
{
    if (!checkParameterRange(limit,0,1023)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_MAX_TORQUE_L;
    UBYTE limit_low = (UBYTE) (limit % 256);
    UBYTE limit_high = (UBYTE) (limit >> 8);

    data.push_back(reg);
    data.push_back(limit_low);
    data.push_back(limit_high);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::readPresentTemperature(UBYTE id, int &temperature, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> temps = {0};

    // Wrap to multi servo command
    UBYTE error = readPresentTemperature(ids,temps,comm);

    // Remap data
    temperature = temps.at(0);

    return error;
}

UBYTE PXProtocol::readPresentTemperature(const std::vector<UBYTE>& ids, std::vector<int> &temperatures, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_PRESENT_TEMPERATUR;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    temperatures.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        temperatures.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];

}

UBYTE PXProtocol::readRegistered(UBYTE id, int &registered, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> registereds = {0};

    // Wrap to multi servo command
    UBYTE error = readRegistered(ids,registereds,comm);

    // Remap data
    registered = registereds.at(0);

    return error;
}

UBYTE PXProtocol::readRegistered(const std::vector<UBYTE>& ids, std::vector<int> &registereds, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_REGISTERED;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    registereds.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        registereds.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::readMoving(UBYTE id, int &moving, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> movings = {0};

    // Wrap to multi servo command
    UBYTE error = readMoving(ids,movings,comm);

    // Remap data
    moving = movings.at(0);

    return error;
}

UBYTE PXProtocol::readMoving(const std::vector<UBYTE>& ids, std::vector<int> &movings, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_MOVING;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    movings.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        movings.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}



UBYTE PXProtocol::readPresentPosition(UBYTE id, int &position, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> positions = {0};

    // Wrap to multi servo command
    UBYTE error = readPresentPosition(ids,positions,comm);

    // Remap data
    position = positions.at(0);

    return error;
}

UBYTE PXProtocol::readPresentPosition(const std::vector<UBYTE>& ids, std::vector<int> &positions, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_PRESENT_POSITION_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    positions.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        positions.push_back((int) (response.at(i+6) << 8) | response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::readPresentSpeed(UBYTE id, int &speed, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> speeds = {0};

    // Wrap to multi servo command
    UBYTE error = readPresentSpeed(ids,speeds,comm);

    // Remap data
    speed = speeds.at(0);

    return error;
}

UBYTE PXProtocol::readPresentSpeed(const std::vector<UBYTE>& ids, std::vector<int> &speeds, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_PRESENT_SPEED_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    speeds.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        speeds.push_back((int) calculateSpeed( (response.at(i+6) << 8) , (response.at(i+5))));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::readPresentLoad(UBYTE id, int &load, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> loads = {0};

    // Wrap to multi servo command
    UBYTE error = readPresentLoad(ids,loads,comm);

    // Remap data
    load = loads.at(0);

    return error;
}

UBYTE PXProtocol::readPresentLoad(const std::vector<UBYTE>& ids, std::vector<int> &loads, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_PRESENT_LOAD_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    loads.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        loads.push_back( (int) calculateLoad( (response.at(i+6) << 8),(response.at(i+5)) ));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::readPresentVoltage(UBYTE id, int &voltage, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> volts = {0};

    // Wrap to multi servo command
    UBYTE error = readPresentVoltage(ids,volts,comm);

    // Remap data
    voltage = volts.at(0);

    return error;
}

UBYTE PXProtocol::readPresentVoltage(const std::vector<UBYTE>& ids, std::vector<int> &voltages, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_PRESENT_VOLTAGE;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    voltages.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        voltages.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setGoalPosition(UBYTE id, int position, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> positions = {position};

    // Wrap to multi servo command
    UBYTE error = setGoalPosition(ids,positions,comm);

    return error;
}

UBYTE PXProtocol::setGoalPosition(const std::vector<UBYTE>& ids, const std::vector<int>& positions, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_GOAL_POSITION_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        appendData(data,positions,true);

        package = makeMultiWritePackage(ids,reg,data,nBytes);

        // Send package and recieve response
        comm.sendData(package);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

        return 0x00;

    }else{
        // Single Servo Mode
        data.push_back(reg);
        appendData(data,positions,true);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_WRITE_DATA,data);

        // Send package and recieve response
        comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

        // Check Checksum
        if(!checkChecksum(response)){
            /// @todo TODO Throw Exception
            ROS_INFO("Checksum mismatch");
        }

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

        // Return Error Byte
        checkError(response[4]);
        return response[4];
    }
}

UBYTE PXProtocol::setGoalPositionAndSpeed(const std::vector<UBYTE> &ids, const std::vector<int> &positions, const std::vector<int> &speeds, SerialComm &comm)
{
    std::vector<UBYTE> package, data;

    UBYTE reg = DYNAMIXEL_GOAL_POSITION_L;
    UBYTE nBytes = 0x04;

    // Arrange data to be pos1 speed1 pos2 speed2 ... posN speedN
    std::vector<int> fusion;
    for(int i = 0; i < ids.size(); ++i){
        fusion.push_back(positions.at(i));
        fusion.push_back(speeds.at(i));
    }

    // append the actual information
    appendData(data,fusion,true);

    // make a package that performs a write to several servo ids
    package = makeMultiWritePackage(ids,reg,data,nBytes);

    // Send package and recieve response
    comm.sendData(package);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    return 0x00;
}

UBYTE PXProtocol::setGoalPositionAndSpeed(UBYTE id, int position, int speed, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> positions = {position};
    std::vector<int> speeds = {speed};

    // Wrap to multi servo command
    UBYTE error = setGoalPositionAndSpeed(ids,positions,speeds,comm);

    return error;
}

UBYTE PXProtocol::readGoalSpeed(UBYTE id, int &speed, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> speeds = {0};

    // Wrap to multi servo command
    UBYTE error = readGoalSpeed(ids,speeds,comm);

    // Remap data
    speed = speeds.at(0);

    return error;
}

UBYTE PXProtocol::readGoalSpeed(const std::vector<UBYTE>& ids, std::vector<int> &speeds, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_GOAL_SPEED_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    speeds.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        speeds.push_back((int) (response.at(i+6) << 8) | response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setGoalSpeed(UBYTE id, int speed, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> speeds = {speed};

    // Wrap to multi servo command
    UBYTE error = setGoalSpeed(ids,speeds,comm);

    return error;
}

UBYTE PXProtocol::setGoalSpeed(const std::vector<UBYTE>& ids, const std::vector<int>& speeds, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_GOAL_SPEED_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        appendData(data,speeds,true);

        package = makeMultiWritePackage(ids,reg,data,nBytes);

        // Send package and recieve response
        comm.sendData(package);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

        return 0x00;

    }else{
        // Single Servo Mode
        data.push_back(reg);
        appendData(data,speeds,true);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_WRITE_DATA,data);

        // Send package and recieve response
        comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

        // Check Checksum
        if(!checkChecksum(response)){
            /// @todo TODO Throw Exception
            ROS_INFO("Checksum mismatch");
        }

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

        // Return Error Byte
        checkError(response[4]);
        return response[4];
    }
}

UBYTE PXProtocol::readTorqueLimit(UBYTE id, int &limit, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> limits = {0};

    // Wrap to multi servo command
    UBYTE error = readTorqueLimit(ids,limits,comm);

    // Remap data
    limit = limits.at(0);

    return error;
}

UBYTE PXProtocol::readTorqueLimit(const std::vector<UBYTE>& ids, std::vector<int> &limits, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_TORQUE_LIMIT_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    limits.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        limits.push_back((int) (response.at(i+6) << 8) | response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setTorqueLimit(UBYTE id, int limit, SerialComm &comm)
{
    if (!checkParameterRange(limit,0,1023)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_TORQUE_LIMIT_L;
    UBYTE limit_low = (UBYTE) (limit % 256);
    UBYTE limit_high = (UBYTE) (limit >> 8);

    data.push_back(reg);
    data.push_back(limit_low);
    data.push_back(limit_high);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::setReturnLevel(UBYTE id, UBYTE level, SerialComm &comm)
{
    if (!checkParameterRange(level,0,2)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_RETURN_LEVEL;

    data.push_back(reg);
    data.push_back(level);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    return response[4];
}

UBYTE PXProtocol::setAlarmLED(UBYTE id, UBYTE level, SerialComm &comm)
{
    if (!checkParameterRange(level,0,255)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_ALARM_LED;

    data.push_back(reg);
    data.push_back(level);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::readAlarmLED(UBYTE id, int &level, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> levels = {0};

    // Wrap to multi servo command
    UBYTE error = readAlarmLED(ids,levels,comm);

    // Remap data
    level = levels.at(0);

    return error;
}

UBYTE PXProtocol::readAlarmLED(const std::vector<UBYTE>& ids, std::vector<int> &levels, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_ALARM_LED;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    levels.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        levels.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setAlarmShutdown(UBYTE id, UBYTE level, SerialComm &comm)
{

    if (!checkParameterRange(level,0,255)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_ALARM_SHUTDOWN;

    data.push_back(reg);
    data.push_back(level);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::readAlarmShutdown(UBYTE id, int &level, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> levels = {0};

    // Wrap to multi servo command
    UBYTE error = readAlarmShutdown(ids,levels,comm);

    // Remap data
    level = levels.at(0);

    return error;
}

UBYTE PXProtocol::readAlarmShutdown(const std::vector<UBYTE>& ids, std::vector<int> &levels, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_ALARM_SHUTDOWN;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    levels.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        levels.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::readTorqueState(UBYTE id, int &state, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> states = {0};

    // Wrap to multi servo command
    UBYTE error = readTorqueState(ids,states,comm);

    // Remap data
    state = states.at(0);

    return error;
}

UBYTE PXProtocol::readTorqueState(const std::vector<UBYTE>& ids, std::vector<int> &states, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_TORQUE_ENDABLE;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    states.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        states.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setTorqueState(UBYTE id, UBYTE state, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> states = {(int)state};

    // Wrap to multi servo command
    UBYTE error = setTorqueState(ids,states,comm);

    return error;
}

UBYTE PXProtocol::setTorqueState(const std::vector<UBYTE> &ids, std::vector<int> states, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_TORQUE_ENDABLE;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        appendData(data,states,true);

        package = makeMultiWritePackage(ids,reg,data,nBytes);

        // Send package and recieve response
        comm.sendData(package);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

        return 0x00;

    }else{
        // Single Servo Mode
        data.push_back(reg);
        appendData(data,states,true);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_WRITE_DATA,data);

        // Send package and recieve response
        comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

        // Check Checksum
        if(!checkChecksum(response)){
            /// @todo TODO Throw Exception
            ROS_INFO("Checksum mismatch");
        }

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

        // Return Error Byte
        checkError(response[4]);
        return response[4];
    }
}

UBYTE PXProtocol::readLEDState(UBYTE id, int &state, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> states = {0};

    // Wrap to multi servo command
    UBYTE error = readLEDState(ids,states,comm);

    // Remap data
    state = states.at(0);

    return error;
}

UBYTE PXProtocol::readLEDState(const std::vector<UBYTE>& ids, std::vector<int> &states, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_LED_ENABLE;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    states.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        states.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setLEDState(UBYTE id, UBYTE state, SerialComm &comm)
{
    if (!checkParameterRange(state,0,1)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_LED_ENABLE;

    data.push_back(reg);
    data.push_back(state);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::readCWComplianceMargin(UBYTE id, int &margin, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> margins = {0};

    // Wrap to multi servo command
    UBYTE error = readCWComplianceMargin(ids,margins,comm);

    // Remap data
    margin = margins.at(0);

    return error;
}

UBYTE PXProtocol::readCWComplianceMargin(const std::vector<UBYTE>& ids, std::vector<int> &margins, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_CW_COMP_MARGIN;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    margins.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        margins.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setCWComplianceMargin(UBYTE id, UBYTE margin, SerialComm &comm)
{
    if (!checkParameterRange(margin,0,255)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_CW_COMP_MARGIN;

    data.push_back(reg);
    data.push_back(margin);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::readCCWComplianceMargin(UBYTE id, int &margin, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> margins = {0};

    // Wrap to multi servo command
    UBYTE error = readCCWComplianceMargin(ids,margins,comm);

    // Remap data
    margin = margins.at(0);

    return error;
}

UBYTE PXProtocol::readCCWComplianceMargin(const std::vector<UBYTE>& ids, std::vector<int> &margins, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_CCW_COMP_MARGIN;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    margins.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        margins.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setCCWComplianceMargin(UBYTE id, UBYTE margin, SerialComm &comm)
{
    if (!checkParameterRange(margin,0,255)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_CCW_COMP_MARGIN;

    data.push_back(reg);
    data.push_back(margin);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::readCWComplianceSlope(UBYTE id, int &slope, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> slopes = {0};

    // Wrap to multi servo command
    UBYTE error = readCWComplianceSlope(ids,slopes,comm);

    // Remap data
    slope = slopes.at(0);

    return error;
}

UBYTE PXProtocol::readCWComplianceSlope(const std::vector<UBYTE>& ids, std::vector<int> &slopes, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_CW_COMP_SLOPE;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    slopes.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        slopes.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setCWComplianceSlope(UBYTE id, UBYTE slope, SerialComm &comm)
{
    if (!checkParameterRange(slope,0,255)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_CW_COMP_SLOPE;

    data.push_back(reg);
    data.push_back(slope);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::readCCWComplianceSlope(UBYTE id, int &slope, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> slopes = {0};

    // Wrap to multi servo command
    UBYTE error = readCCWComplianceSlope(ids,slopes,comm);

    // Remap data
    slope = slopes.at(0);

    return error;
}

UBYTE PXProtocol::readCCWComplianceSlope(const std::vector<UBYTE>& ids, std::vector<int> &slopes, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_CCW_COMP_SLOPE;
    UBYTE nBytes = 0x01;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    slopes.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        slopes.push_back((int) response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::setCCWComplianceSlope(UBYTE id, UBYTE slope, SerialComm &comm)
{
    if (!checkParameterRange(slope,0,255)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_CCW_COMP_SLOPE;

    data.push_back(reg);
    data.push_back(slope);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::setComplianceMargin(UBYTE id, UBYTE cw, UBYTE ccw, SerialComm &comm)
{
    if (!checkParameterRange(cw,0,255) || !checkParameterRange(ccw,0,255)){
        //return 0xFF;
    }

    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_CW_COMP_MARGIN;

    data.push_back(reg);
    data.push_back(cw);
    data.push_back(ccw);

    package = makeSinglePackage(id,DYNAMIXEL_WRITE_DATA,data);

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    //Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Return Error Byte
    checkError(response[4]);
    return response[4];
}

UBYTE PXProtocol::setComplianceSlope(UBYTE id, UBYTE cw, UBYTE ccw, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> cws = {cw};
    std::vector<int> ccws = {ccw};

    // Wrap to multi servo command
    UBYTE error = setComplianceSlope(ids,cws,ccws,comm);

    return error;
}

UBYTE PXProtocol::setComplianceSlope(std::vector<UBYTE> ids, std::vector<int> cws, std::vector<int> ccws, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_CW_COMP_SLOPE;
    UBYTE nBytes = 0x02;

    // Arrange data to be cw1 ccw1 cw2 ccw2 ... cwN ccwN
    std::vector<int> fusion;
    for(int i = 0; i < ids.size(); ++i){
        fusion.push_back(cws.at(i));
        fusion.push_back(ccws.at(i));
    }

    if(nServos > 1){
        // Multi Servo Mode
        appendData(data,fusion,false);

        package = makeMultiWritePackage(ids,reg,data,nBytes);

        // Send package and recieve response
        comm.sendData(package);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

        return 0x00;

    }else{
        // Single Servo Mode
        data.push_back(reg);
        appendData(data,fusion,false);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_WRITE_DATA,data);

        // Send package and recieve response
        comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE);

        // Check Checksum
        if(!checkChecksum(response)){
            /// @todo TODO Throw Exception
            ROS_INFO("Checksum mismatch");
        }

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

        // Return Error Byte
        checkError(response[4]);
        return response[4];
    }
}

UBYTE PXProtocol::readGoalPosition(UBYTE id, int &position, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> positions = {0};

    // Wrap to multi servo command
    UBYTE error = readGoalPosition(ids,positions,comm);

    // Remap data
    position = positions.at(0);

    return error;
}

UBYTE PXProtocol::readGoalPosition(const std::vector<UBYTE>& ids, std::vector<int> &positions, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_GOAL_POSITION_L;
    UBYTE nBytes = 0x02;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    positions.clear();
    for(int i = 0; i < nBytes * nServos; i+=nBytes){
        positions.push_back((int) (response.at(i+6) << 8) | response.at(i+5));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

void PXProtocol::printBytes(const std::vector<UBYTE> &data)
{
    int n = data.size();

    ROS_INFO("##### Start Printing Bytes #####");
    for(int i = 0; i<n; i++){
        ROS_INFO("Byte%d : %#X",i,data.at(i));
    }
    ROS_INFO("##### End Printing Bytes #####");
}


UBYTE PXProtocol::readAngleLimit(UBYTE id, int &cw, int &ccw, SerialComm &comm)
{
    std::vector<UBYTE> ids = {id};
    std::vector<int> cws = {0};
    std::vector<int> ccws = {0};

    // Wrap to multi servo command
    UBYTE error = readAngleLimit(ids,cws,ccws,comm);

    // Remap data
    cw = cws.at(0);
    ccw = ccws.at(0);

    return error;
}

UBYTE PXProtocol::readAngleLimit(const std::vector<UBYTE>& ids, std::vector<int> &cws, std::vector<int> &ccws, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response;

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_CW_LIMIT_L;
    UBYTE nBytes = 0x04;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    cws.clear();
    ccws.clear();
    for(int i = 0; i < nBytes * nServos; i += nBytes){
        cws.push_back((int) (response.at(i+6) << 8) | response.at(i+5));
        ccws.push_back((int) (response.at(i+8) << 8) | response.at(i+7));
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::readServoCompliance(ServoCompliance &comp, SerialComm &comm)
{
    ServoCompliance temp(comp.id_);
    std::vector<ServoCompliance> comps = {temp};

    // Wrap to multi servo command
    UBYTE error = readServoCompliance(comps,comm);

    // Remap data
    comp.cw_margin_ = (comps.at(0)).cw_margin_;
    comp.ccw_margin_ = (comps.at(0)).ccw_margin_;
    comp.cw_slope_ = (comps.at(0)).cw_slope_;
    comp.ccw_slope_ = (comps.at(0)).ccw_slope_;
    comp.punch_ = (comps.at(0)).punch_;

    return error;
}

UBYTE PXProtocol::readServoCompliance(std::vector<ServoCompliance> &comps, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response, ids;
    ids.reserve(comps.size());

    // Get the ids
    for(ServoCompliance elem : comps){
        ids.push_back(elem.id_);
    }

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_CW_COMP_MARGIN;
    UBYTE nBytes = 0x04;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Punch is not in a row with the other registers but belongs to the compliance parameters
    std::vector<int> punchs;
    readPunch(ids,punchs,comm);

    // Extract Data
    for(int i = 0; i < comps.size(); ++i){
        (comps.at(i)).cw_margin_ =     (int)  response.at(nBytes*i + 5);
        (comps.at(i)).ccw_margin_ =    (int)  response.at(nBytes*i + 6);
        (comps.at(i)).cw_slope_ =      (int)  response.at(nBytes*i + 7);
        (comps.at(i)).ccw_slope_ =     (int)  response.at(nBytes*i + 8);
        (comps.at(i)).punch_ =        (int)  punchs.at(i);
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary

    return response[4];
}

UBYTE PXProtocol::readServoTarget(ServoTarget &target, SerialComm &comm)
{
    ServoTarget temp(target.id_);
    std::vector<ServoTarget> targets = {temp};

    // Wrap to multi servo command
    UBYTE error = readServoTarget(targets,comm);

    // Remap data
    target.target_position_ = (targets.at(0)).target_position_;
    target.target_speed_ = (targets.at(0)).target_speed_;
    target.max_torque_ = (targets.at(0)).max_torque_;

    return error;
}

UBYTE PXProtocol::readServoTarget(std::vector<ServoTarget> &targets, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response, ids;
    ids.reserve(targets.size());

    // Get the ids
    for(const ServoTarget& elem : targets){
        ids.push_back(elem.id_);
    }

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_GOAL_POSITION_L;
    UBYTE nBytes = 0x06;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    for(int i = 0; i < targets.size(); ++i){
        (targets.at(i)).target_position_ =   (int) (response.at(nBytes*i + 6) << 8) | response.at(nBytes*i + 5);
        (targets.at(i)).target_speed_ =      (int) (response.at(nBytes*i + 8) << 8) | response.at(nBytes*i + 7);
        (targets.at(i)).max_torque_ =        (int) (response.at(nBytes*i + 10) << 8) | response.at(nBytes*i + 9);
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

UBYTE PXProtocol::readServoStatus(ServoStatus &status, SerialComm &comm)
{
    ServoStatus temp(status.id_);
    std::vector<ServoStatus> stati = {temp};

    // Wrap to multi servo command
    UBYTE error = readServoStatus(stati,comm);

    // Remap data
    status.position_ = (stati.at(0)).position_;
    status.speed_ = (stati.at(0)).speed_;
    status.load_ = (stati.at(0)).load_;
    status.voltage_ = (stati.at(0)).voltage_;
    status.temperature_ = (stati.at(0)).temperature_;

    return error;
}

UBYTE PXProtocol::readServoStatus(std::vector<ServoStatus> &status, SerialComm &comm)
{
    std::vector<UBYTE> package, data, response, ids;
    ids.reserve(status.size());

    // Get the ids
    for(const ServoStatus& elem : status){
        ids.push_back(elem.id_);
    }

    UBYTE nServos = ids.size();
    UBYTE reg = DYNAMIXEL_PRESENT_POSITION_L;
    UBYTE nBytes = 0x08;

    if(nServos > 1){
        // Multi Servo Mode
        package = makeMultiReadPackage(ids,reg,nBytes);

    }else{
        // Single Servo Mode
        data.push_back(reg);
        data.push_back(nBytes);

        package = makeSinglePackage(ids.at(0),DYNAMIXEL_READ_DATA,data);
    }

    // Send package and recieve response
    comm.sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes * nServos);

#ifdef PRINT_BYTES
    // Print out Bytes to the Console
    printBytes(package);
    printBytes(response);
#endif

    // Check Checksum
    if(!checkChecksum(response)){
        /// @todo TODO Throw Exception
        ROS_INFO("Checksum mismatch");
    }

    // Extract Data
    for(int i = 0; i < status.size(); ++i){
        (status.at(i)).position_ =      (int) (response.at(nBytes*i + 6) << 8) | response.at(nBytes*i + 5);
        (status.at(i)).speed_ =         (int) calculateSpeed((response.at(nBytes*i + 8) << 8),response.at(nBytes*i + 7));
        (status.at(i)).load_ =          (int) calculateLoad((response.at(nBytes*i + 10) << 8),response.at(nBytes*i + 9));
        (status.at(i)).voltage_ =       (int)  response.at(nBytes*i + 11);
        (status.at(i)).temperature_ =   (int)  response.at(nBytes*i + 12);
    }

    // Return Error Byte
    /// @todo TODO check error byte and throw exception if nescecary
    return response[4];
}

bool PXProtocol::checkChecksum(const std::vector<UBYTE> &data) const
{
    if (data.size() < 5){
        return true;
    }

    UBYTE id = data.at(2);
    UBYTE len = data.at(3);
    UBYTE err = data.at(4);
    UBYTE checksum = data.at(3+len);

    UBYTE sum = id + len + err;

    for(int i = 5; i < len+3; i++){
        sum += data.at(i);
    }
    UBYTE check = 255 - (sum % 256);
    //ROS_INFO("Calculated Checksum: %#X",check);
    //ROS_INFO("Recieved Checksum: %#X",checksum);

    return check == checksum;
}

bool PXProtocol::checkError(UBYTE error, bool silent)
{
    /// @todo TODO Throw single excpetions in case of single error

    /// @todo TODO Throw mixed-exception in case of multiple errors

    int c = 0;
    for(int i = 0; i < 8; i++){
        if((error >> i) & 1){
            c++;
        }
    }

    if(c == 0){
        return false;
    }else if(c > 1){
        //throw mixed excpetion
        ROS_INFO_COND(!silent, "Mixed Exception");
    }

    if((error >> 0) & 1){
        //Voltage out of range
        ROS_INFO_COND(!silent, "Voltage Exception");
    }

    if((error >> 1) & 1){
        //Desired joint position out of limits
        ROS_INFO_COND(!silent, "out of limits Exception");
    }

    if((error >> 2) & 1){
        //Servo too hot
        ROS_INFO_COND(!silent, "Servo too hot Exception");
    }

    if((error >> 3) & 1){
        //Instruction out of range
        ROS_INFO_COND(!silent, "Instruction out of range Exception");
    }

    if((error >> 4) & 1){
        //Checksum error
        ROS_INFO_COND(!silent, "Checksum error Exception: %X",error);
    }

    if((error >> 5) & 1){
        //Too much load for the maximum torque
        ROS_INFO_COND(!silent, "Too much load Exception");
    }

    if((error >> 6) & 1){
        //Undefined instruction
        ROS_INFO_COND(!silent, "Undefined instruction Exception");
    }

    if((error >> 7) & 1){
        ROS_INFO_COND(!silent, "Unknow Error");
    }
    return true;
}

bool PXProtocol::checkParameterRange(int &param, int low, int high, bool exception) const
{
    bool retval = true;
    if (param > high || param < low){
        retval = false;
        if (exception)
        {
            /// @todo TODO Throw parameter out of range exception
        }
    }

    param = std::max(low,param);
    param = std::min(high,param);
    return retval;
}

bool PXProtocol::checkParameterRange(UBYTE &param, int low, int high, bool exception) const
{
    bool retval = true;
    if (param > high || param < low){
        retval = false;
        if (exception)
        {
            /// @todo TODO Throw parameter out of range exception
        }
    }

    param = std::max(low,(int) param);
    param = std::min(high,(int) param);
    return retval;
}

int PXProtocol::calculateSpeed(UBYTE hi_byte, UBYTE lo_byte) const
{
    int speed = hi_byte | lo_byte;
    speed = speed & 1023;

    if( (hi_byte >> 2) > 0 ){
        return -1*speed;
    }else{
        return speed;
    }
}

int PXProtocol::calculateLoad(UBYTE hi_byte, UBYTE lo_byte) const
{
    return calculateSpeed(hi_byte,lo_byte);
}

void PXProtocol::appendData(std::vector<UBYTE> &data, const std::vector<int>& raw_data, bool is_low_high) const
{
    for(int i = 0; i < raw_data.size(); ++i){

        int elem = raw_data.at(i);

        if(is_low_high){
            UBYTE low = (UBYTE) (elem % 256);
            UBYTE high = (UBYTE) (elem >> 8);

            data.push_back(low);
            data.push_back(high);
        }else{
            data.push_back((UBYTE) elem);
        }
    }
}

std::vector<UBYTE> PXProtocol::makeSinglePackage(UBYTE id, UBYTE inst, const std::vector<UBYTE>& data) const
{
    std::vector<UBYTE> package;

    // Length is data length plus 2 (id,inst)
    UBYTE len = data.size() + 2;

    // Build up the package
    package.push_back(DYNAMIXEL_PREFIX);
    package.push_back(DYNAMIXEL_PREFIX);
    package.push_back(id);
    package.push_back(len);
    package.push_back(inst);

    // Append the data vector
    package.insert(package.end(),data.begin(),data.end());

    // Get the sum of the data elements
    int sum = 0;
    for(UBYTE element : data){
        sum += (int) element;
    }

    // Calculate and add checksum
    UBYTE checksum = 255 - ((id + len + inst + sum) % 256);
    package.push_back(checksum);

    return package;
}

std::vector<UBYTE> PXProtocol::makeMultiReadPackage(const std::vector<UBYTE>& ids, UBYTE addr, UBYTE len) const
{
    std::vector<UBYTE> data;
    data.push_back(addr);
    data.push_back(len);
    data.insert(data.end(),ids.begin(),ids.end());

    return makeSinglePackage(DYNAMIXEL_BROADCAST,DYNAMIXEL_SYNC_READ,data);
}

std::vector<UBYTE> PXProtocol::makeMultiWritePackage(const std::vector<UBYTE>& ids, UBYTE addr, const std::vector<UBYTE>& data, UBYTE n_bytes) const
{
    std::vector<UBYTE> dataBytes, package;
    UBYTE id = DYNAMIXEL_BROADCAST;
    UBYTE inst = DYNAMIXEL_SYNC_WRITE;

    int N = ids.size();

    UBYTE len = (n_bytes+1) * N + 4;
    dataBytes.reserve((n_bytes+1)*N);

    for(int i = 0; i < N; ++i){
        dataBytes.push_back(ids.at(i));
        dataBytes.insert(dataBytes.end(),data.begin() + i*n_bytes, data.begin() + (i+1)*n_bytes);
    }

    // Get the sum of the data elements
    int sum = 0;
    for(UBYTE element : dataBytes){
        sum += (int) element;
    }

    package.push_back(DYNAMIXEL_PREFIX);
    package.push_back(DYNAMIXEL_PREFIX);
    package.push_back(id);
    package.push_back(len);
    package.push_back(inst);
    package.push_back(addr);
    package.push_back(n_bytes);

    // Append Data
    package.insert(package.end(),dataBytes.begin(),dataBytes.end());


    // Calculate and add checksum
    UBYTE checksum = 255 - ((id + len + inst + addr + n_bytes + sum) % 256);
    package.push_back(checksum);

    return package;
}


} // end namespace pxpincher
