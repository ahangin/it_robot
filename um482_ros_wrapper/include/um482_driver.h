/*
 * File: um482_driver.h
 * Brief: Header file for UM482 GNSS RTK driver class
 * Description: This file contains the declaration of the UM482Driver class which handles 
 *              communication with the UniStrong UM482 GNSS receiver via serial port, 
 *              parses NMEA sentences, and publishes ROS messages.
 * Author: ahangin
 * Date: 2024-05-01
 * Copyright: Copyright (c) 2024 ahangin
 * License: MIT License
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * Modification Record:
 *   - 2024-05-01 | ahangin | v1.0.0 | Initial version, added file header comment
 */
#ifndef UM482_DRIVER_H
#define UM482_DRIVER_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nmea_msgs/Gpgga.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>

#define PI 3.141592653589793

class UM482Driver
{
public:
    UM482Driver(ros::NodeHandle& nh, ros::NodeHandle& param_nh);
    ~UM482Driver();

    bool initialize();
    void run();

private:
    // Serial communication
    serial::Serial ser_;
    
    // ROS publishers
    ros::Publisher gpgga_pub_;
    ros::Publisher odometry_pub_;
    ros::Publisher navsatfix_pub_;
    
    // ROS parameters
    std::string port_;
    int baudrate_;
    int timeout_;
    float gpgga_freq_;
    float gptra_freq_;
    float bestxyza_freq_;
    bool gpgga_enable_;
    bool gptra_enable_;
    bool bestxyza_enable_;
    bool unlogall_enable_;
    int update_rate_;  // 新增参数控制更新频率
    
    // Message objects
    nmea_msgs::Gpgga msg_gpgga_;
    nav_msgs::Odometry msg_gnssodometry_;
    sensor_msgs::NavSatFix msg_navsatfix_;

    // Helper functions
    template <typename Type>
    Type stringToNum(const std::string &str);

    template <typename Type>
    std::string numToString(const Type &num);

    void parseNmeaSentence(const std::string& sentence);
    void handleGpggaSentence(const std::string& sentence, std::vector<int>& separator_pos);
    void handleBestxyzSentence(const std::string& sentence, std::vector<int>& separator_pos);
    void handleGptraSentence(const std::string& sentence, std::vector<int>& separator_pos);
};

#endif // UM482_DRIVER_H