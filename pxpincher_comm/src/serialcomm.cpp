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

#include "pxpincher_comm/serialcomm.h"

namespace pxpincher
{

  
SerialComm::SerialComm():
    opened_(false),
    initialized_(false),
    initializing_(false)
{
}


SerialComm::SerialComm(const std::string& device, unsigned long baud):
    opened_(false),
    initialized_(false),
    initializing_(false)
{
    open(device, baud);
}

bool SerialComm::open(const std::string& device, unsigned long baud)
{
    if (opened_)
    {
      ROS_INFO("SerialPort already opened. Skipping...");
      return true;
    }
  
    boost::mutex::scoped_lock lock(mex_);
    
    serialComm_.open(device.c_str(),baud); //TODO Catch Exception

    if(serialComm_.portOpen()){ //TODO Catch Exception
        ROS_INFO("Serial port opened");
        opened_ = true;

    }else{
        ROS_INFO("Serial port not opened");
    }
    return opened_;
}

void SerialComm::sendData(const std::vector<UBYTE>& output_data, std::vector<UBYTE>* input_data, int resp_len)
{
    if(!opened_){
        return;
    }

    if(!initialized_ && !initializing_){
        initializeConnection();
    }

    const UBYTE* output_buffer = output_data.data();

    boost::mutex::scoped_lock lock(mex_);

    serialComm_.flush(); //TODO Catch Exception
    serialComm_.write((const char*)output_buffer,output_data.size()); //TODO Catch Exception


    if(input_data && resp_len>0){
      
        UBYTE input_buffer[resp_len];
        std::fill(input_buffer, input_buffer+resp_len, 0);
    
        serialComm_.readBytes((char*)input_buffer,resp_len,1000); //TODO Catch Exception
        serialComm_.flush();

        input_data->clear();
        input_data->assign(input_buffer,input_buffer+resp_len);
    }
}

void SerialComm::close()
{
    if(!opened_){
        return;
    }

    boost::mutex::scoped_lock lock(mex_);
    
    serialComm_.close(); //TODO Catch Exception

    opened_ = false;
    ROS_INFO("Serial port closed");
}

void SerialComm::initializeConnection(UBYTE default_id)
{
    if(initialized_){
        // Communication object already initialized
        ROS_INFO("Communication already initialized...");
        return;
    }
    bool success = false;
    initializing_ = true;

    ROS_INFO("Initializing...");

    while(!success && ros::ok()){
        try{
            readInitBytes(default_id);
            success = true;
        }catch(cereal::TimeoutException ex){
            ROS_INFO("Timeout...");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if(success){
        initialized_ = true;
        ROS_INFO("Initializing successfull...");
    }
    initializing_ = false;
}

void SerialComm::readInitBytes(UBYTE id)
{
    std::vector<UBYTE> package, data, response;

    UBYTE reg = DYNAMIXEL_MODEL_NUMBER_L;
    UBYTE nBytes = 0x02;

    data.push_back(reg);
    data.push_back(nBytes);

    // Length is data length plus 2 (id,inst)
    UBYTE len = data.size() + 2;
    UBYTE inst = DYNAMIXEL_READ_DATA;

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

    // Send package and receive response
    sendData(package, &response, DYNAMIXEL_NO_DATA_RESPONSE + nBytes);
}

} // end namespace pxpincher
