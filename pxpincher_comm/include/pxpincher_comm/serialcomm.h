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

#ifndef SERIALCOMM_H
#define SERIALCOMM_H

#include "codes.h"                  //Register defines

#include "ros/ros.h"                //ROS include

#include <string>                   //std::string
#include "CerealPort.h"             //cereal::CerealPort
#include "boost/thread/mutex.hpp"   //boost::mutex

#include <thread> // for c++11 sleep
#include <chrono> // for c++11 sleep

namespace pxpincher
{

/**
 * @class SerialComm
 * @brief Helper class for performing the actual serial communication
 * 
 * Currently the serial communication is performed utilizing CerealPort by Gonçalo Cabrita.
 * @remarks This class assumes that one of the servos is assigned to id=1.
 */
class SerialComm
{

public:
    
    /**
     * @brief Construct SerialComm class
     */
    SerialComm();
    
    /**
     * @brief Construct SerialComm class and open device
     * @param device port name (e.g. "/dev/ttyUSB0")
     * @param baud baud rate of the serial port (e.g. 115200)
     */
    SerialComm(const std::string& device, unsigned long baud);
    
    /**
     * @brief Open device (if not already opened using a proper constructor)
     * @param device port name (e.g. "/dev/ttyUSB0")
     * @param baud baud rate of the serial port (e.g. 115200)
     * @return \c true, if port is open, otherwise \c false.
     */
    bool open(const std::string& device, unsigned long baud);
    
    /**
     * @brief Send vector of bytes via already initialized serial port
     * @param output_data vector of bytes that should that should be sent to the serial port
     * @param input_data (optional) provide if a response is demanded/expeced of the particular length \c resp_len (input_data will be cleared)
     * @param resp_len (optional) specify the expected response length; this parameter is only required if \c input_data is provided
     */
    void sendData(const std::vector<UBYTE>& output_data, std::vector<UBYTE>* input_data = nullptr, int resp_len = 0);
    
    /**
     * @brief Close serial port connection
     */
    void close();

private:
    
    /**
     * @brief Initialize connection to the servo joints
     * 
     * This method is called once as soon as the first data is sent to the servos using sendData().
     * It tries to send a small init message to one of the attached servos (e.g. servo with id 1) by calling readInitBytes() repeatedly.
     * If no timeout exception occures anymore, the state of this class and the related serial connection is set to initialized.     * 
     * @param default_id specify the servo that should be polled in order to test the connection
     */
    void initializeConnection(UBYTE default_id = 1);
    
    /**
     * @brief Read some initial bytes in order to test the connection to the arbotix board
     * 
     * @param id specify the target servo id
     */
    void readInitBytes(UBYTE id);
    
    

    cereal::CerealPort serialComm_; //!< Serial communication interface object
    bool opened_; //!< \c true, if the serialComm_ object is connected to a port
    boost::mutex mex_; //!< mux for serial communications
    bool initialized_; //!< \c true, if the connection to a specified test servo (id=1) was successful
    bool initializing_; //!< \c true, if initializeConnection() currently tries to connect to a specific servo

};

} // end namespace pxpincher

#endif // SERIALCOMM_H
