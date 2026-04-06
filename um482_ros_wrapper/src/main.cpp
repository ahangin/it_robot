/*
 * File: main.cpp
 * Brief: Main implementation file for UM482 GNSS RTK driver
 * Description: This file implements the UM482Driver class methods and contains the main function
 *              for the ROS node that communicates with the UniStrong UM482 GNSS receiver.
 *              It handles serial communication, NMEA parsing, and ROS message publishing.
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

#include "um482_driver.h"

template <typename Type>
Type UM482Driver::stringToNum(const std::string &str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

template <typename Type>
std::string UM482Driver::numToString(const Type &num)
{
    std::stringstream ss;
    std::string s;
    ss << num;
    s = ss.str();
    return s;
}

UM482Driver::UM482Driver(ros::NodeHandle& nh, ros::NodeHandle& param_nh)
{
    // Get parameters
    param_nh.param("gnss_port", port_, std::string("/dev/um482"));
    param_nh.param("gnss_Baudrate", baudrate_, 115200);
    param_nh.param("serial_timeout", timeout_, 10);
    param_nh.param("gpgga_enable", gpgga_enable_, true);
    param_nh.param("gptra_enable", gptra_enable_, true);
    param_nh.param("bestxyza_enable", bestxyza_enable_, true);
    param_nh.param("Unlogall_enable", unlogall_enable_, false);
    param_nh.param<float>("gpgga_freq", gpgga_freq_, 1);
    param_nh.param<float>("gptra_freq", gptra_freq_, 1);
    param_nh.param<float>("bestxyza_freq", bestxyza_freq_, 1);

    param_nh.param<int>("update_rate", update_rate_, 20);
    if(update_rate_ < 1) update_rate_ = 1;
    if(update_rate_ > 50) update_rate_ = 50;

    // Initialize publishers
    gpgga_pub_ = nh.advertise<nmea_msgs::Gpgga>("gpgga", 1);
    odometry_pub_ = nh.advertise<nav_msgs::Odometry>("navOdometry", 1);
    navsatfix_pub_ = nh.advertise<sensor_msgs::NavSatFix>("navSatFix", 1);
}

UM482Driver::~UM482Driver()
{
    if(ser_.isOpen())
    {
        ser_.close();
    }
}

bool UM482Driver::initialize()
{
    // Set up serial connection
    ser_.setPort(port_);
    ser_.setBaudrate(baudrate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout_);
    ser_.setTimeout(to);

    try
    {
        ser_.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port " << port_);
        return false;
    }

    if(!ser_.isOpen())
    {
        ROS_ERROR_STREAM("Could not open port " << port_);
        return false;
    }

    ROS_INFO_STREAM("Serial Port " << port_ << " initialized at " << baudrate_ << " baud");

    // Configure logging commands
    if(gpgga_enable_)
    {
        std::cout << "GPGGA enable" << std::endl;
        ser_.write("log com1 gpgga ontime " + numToString(gpgga_freq_) + "\r\n");
        std::cout << "log com1 gpgga ontime " + numToString(gpgga_freq_) + "\r\n" << std::endl;
    }

    if(gptra_enable_)
    {
        std::cout << "GPTRA enable" << std::endl;
        ser_.write("log com1 gptra onchanged\r\n");
    }

    if(bestxyza_enable_)
    {
        std::cout << "BESTXYZA enable" << std::endl;
        ser_.write("log com1 bestxyza ontime " + numToString(bestxyza_freq_) + "\r\n");
    }

    if(unlogall_enable_)
    {
        ser_.write("Unlogall\r\n");
    }

    ser_.flush();
    return true;
}

void UM482Driver::parseNmeaSentence(const std::string& sentence)
{
    // Reset message status for each new sentence
    msg_navsatfix_.status.status = 0;

    std::vector<int> separator_pos;
    separator_pos.push_back(sentence.find(",", 0));

    std::string serialHeader;
    if (separator_pos[0] != std::string::npos) {
        serialHeader.assign(sentence, 1, separator_pos[0] - 1);
    } else {
        // If no comma found, this is not a valid NMEA sentence
        return;
    }

    if (serialHeader == "GPGGA")
    {
        handleGpggaSentence(sentence, separator_pos);
    }
    else if (serialHeader == "BESTXYZA")
    {
        handleBestxyzSentence(sentence, separator_pos);
    }
    else if (serialHeader == "GPTRA")
    {
        handleGptraSentence(sentence, separator_pos);
    }
}

void UM482Driver::handleGpggaSentence(const std::string& sentence, std::vector<int>& separator_pos)
{
    struct SeparatorFormat {
        int first;
        int headerLength;
        int totalCommas;
    };
    SeparatorFormat gpgga;
    gpgga.first = 6;
    gpgga.totalCommas = 14;

    // Find all commas in the sentence
    for(int i = 1; i <= gpgga.totalCommas - 1; i++)
    {
        if (separator_pos[i-1] == std::string::npos) break;
        separator_pos.push_back(sentence.find(",", separator_pos[i-1] + 1));
    }

    if (separator_pos.size() < 13) return; // Not enough fields

    msg_gpgga_.header.stamp = ros::Time::now();
    msg_navsatfix_.header.stamp = ros::Time::now();

    std::string temp_gpgga;

    // UTC seconds
    if (separator_pos[0] < sentence.length() && separator_pos[1] > separator_pos[0])
    {
        temp_gpgga.assign(sentence, separator_pos[0] + 1, separator_pos[1] - separator_pos[0] - 1);
        msg_gpgga_.utc_seconds = stringToNum<double>(temp_gpgga);
    }

    // Latitude
    if (separator_pos[1] < sentence.length() && separator_pos[2] > separator_pos[1])
    {
        temp_gpgga.assign(sentence, separator_pos[1] + 1, separator_pos[2] - separator_pos[1] - 1);
        msg_gpgga_.lat = stringToNum<double>(temp_gpgga);
        double temp = msg_gpgga_.lat / 100;
        int integer = floor(temp);
        double decimals = temp - integer;
        msg_navsatfix_.latitude = integer + decimals / 0.6;
    }

    // Latitude direction
    if (separator_pos[2] < sentence.length() && separator_pos[3] > separator_pos[2])
    {
        msg_gpgga_.lat_dir = temp_gpgga.assign(sentence, separator_pos[2] + 1, separator_pos[3] - separator_pos[2] - 1);
    }

    // Longitude
    if (separator_pos[3] < sentence.length() && separator_pos[4] > separator_pos[3])
    {
        temp_gpgga.assign(sentence, separator_pos[3] + 1, separator_pos[4] - separator_pos[3] - 1);
        msg_gpgga_.lon = stringToNum<double>(temp_gpgga);
        double temp = msg_gpgga_.lon / 100;
        int integer = floor(temp);
        double decimals = temp - integer;
        msg_navsatfix_.longitude = integer + decimals / 0.6;
    }

    // Longitude direction
    if (separator_pos[4] < sentence.length() && separator_pos[5] > separator_pos[4])
    {
        msg_gpgga_.lon_dir = temp_gpgga.assign(sentence, separator_pos[4] + 1, separator_pos[5] - separator_pos[4] - 1);
    }

    // GPS quality indicator
    if (separator_pos[5] < sentence.length() && separator_pos[6] > separator_pos[5])
    {
        temp_gpgga.assign(sentence, separator_pos[5] + 1, separator_pos[6] - separator_pos[5] - 1);
        msg_gpgga_.gps_qual = stringToNum<int>(temp_gpgga);
    }

    // Number of satellites
    if (separator_pos[6] < sentence.length() && separator_pos[7] > separator_pos[6])
    {
        temp_gpgga.assign(sentence, separator_pos[6] + 1, separator_pos[7] - separator_pos[6] - 1);
        msg_gpgga_.num_sats = stringToNum<int>(temp_gpgga);
    }

    // HDOP
    if (separator_pos[7] < sentence.length() && separator_pos[8] > separator_pos[7])
    {
        temp_gpgga.assign(sentence, separator_pos[7] + 1, separator_pos[8] - separator_pos[7] - 1);
        msg_gpgga_.hdop = stringToNum<double>(temp_gpgga);
    }

    // Altitude
    if (separator_pos[8] < sentence.length() && separator_pos[9] > separator_pos[8])
    {
        temp_gpgga.assign(sentence, separator_pos[8] + 1, separator_pos[9] - separator_pos[8] - 1);
        msg_gpgga_.alt = stringToNum<double>(temp_gpgga);
        msg_navsatfix_.altitude = stringToNum<double>(temp_gpgga);
    }

    // Altitude units
    if (separator_pos[9] < sentence.length() && separator_pos[10] > separator_pos[9])
    {
        msg_gpgga_.altitude_units = temp_gpgga.assign(sentence, separator_pos[9] + 1, separator_pos[10] - separator_pos[9] - 1);
    }

    // Undulation
    if (separator_pos[10] < sentence.length() && separator_pos[11] > separator_pos[10])
    {
        temp_gpgga.assign(sentence, separator_pos[10] + 1, separator_pos[11] - separator_pos[10] - 1);
        msg_gpgga_.undulation = stringToNum<double>(temp_gpgga);
    }

    // Undulation units
    if (separator_pos[11] < sentence.length() && separator_pos[12] > separator_pos[11])
    {
        msg_gpgga_.undulation_units = temp_gpgga.assign(sentence, separator_pos[11] + 1, separator_pos[12] - separator_pos[11] - 1);
    }

    // Differential age
    if (separator_pos[12] < sentence.length() && separator_pos[13] > separator_pos[12])
    {
        temp_gpgga.assign(sentence, separator_pos[12] + 1, separator_pos[13] - separator_pos[12] - 1);
        msg_gpgga_.diff_age = stringToNum<int>(temp_gpgga);
    }

    // Station ID
    if (separator_pos[13] < sentence.length())
    {
        msg_gpgga_.station_id = temp_gpgga.assign(sentence, separator_pos[13] + 1, 4);
    }
}

void UM482Driver::handleBestxyzSentence(const std::string& sentence, std::vector<int>& separator_pos)
{
    struct SeparatorFormat {
        int first;
        int headerLength;
        int totalCommas;
    };
    SeparatorFormat bestxyza;
    bestxyza.first = 9;
    bestxyza.totalCommas = 36; // BESTXYZ NOTICE: 9,s + 1; +  27,s

    // Find all commas in the sentence
    for(int i = 1; i <= bestxyza.totalCommas - 1; i++)
    {
        if (separator_pos[i-1] == std::string::npos) break;
        separator_pos.push_back(sentence.find(",", separator_pos[i-1] + 1));
    }

    int header_separator = sentence.find(";", 0);

    msg_gnssodometry_.header.stamp = ros::Time::now();
    msg_gnssodometry_.header.frame_id = "gnss";

    std::string temp_bestxyza;

    if (header_separator != std::string::npos && separator_pos.size() > 9)
    {
        temp_bestxyza.assign(sentence, header_separator + 1, separator_pos[9] - header_separator - 1);  // Status field

        if(temp_bestxyza == "SOL_COMPUTED")
        {
            msg_navsatfix_.status.status = 1;
        }
        else
        {
            msg_navsatfix_.status.status = -1;
        }
    }

    // Process position data if we have enough separators
    if (separator_pos.size() > 24)
    {
        // X in ECEF
        temp_bestxyza.assign(sentence, separator_pos[10] + 1, separator_pos[11] - separator_pos[10] - 1);
        msg_gnssodometry_.pose.pose.position.x = stringToNum<double>(temp_bestxyza);

        // Y in ECEF
        temp_bestxyza.assign(sentence, separator_pos[11] + 1, separator_pos[12] - separator_pos[11] - 1);
        msg_gnssodometry_.pose.pose.position.y = stringToNum<double>(temp_bestxyza);

        // Z in ECEF
        temp_bestxyza.assign(sentence, separator_pos[12] + 1, separator_pos[13] - separator_pos[12] - 1);
        msg_gnssodometry_.pose.pose.position.z = stringToNum<double>(temp_bestxyza);

        // Standard deviation X
        temp_bestxyza.assign(sentence, separator_pos[13] + 1, separator_pos[14] - separator_pos[13] - 1);
        double std_x = stringToNum<double>(temp_bestxyza);
        msg_gnssodometry_.pose.covariance[0] = std_x * std_x;
        msg_navsatfix_.position_covariance[0] = std_x * std_x;

        // Standard deviation Y
        temp_bestxyza.assign(sentence, separator_pos[14] + 1, separator_pos[15] - separator_pos[14] - 1);
        double std_y = stringToNum<double>(temp_bestxyza);
        msg_gnssodometry_.pose.covariance[7] = std_y * std_y;
        msg_navsatfix_.position_covariance[4] = std_y * std_y;

        // Standard deviation Z
        temp_bestxyza.assign(sentence, separator_pos[15] + 1, separator_pos[16] - separator_pos[15] - 1);
        double std_z = stringToNum<double>(temp_bestxyza);
        msg_gnssodometry_.pose.covariance[14] = std_z * std_z;
        msg_navsatfix_.position_covariance[8] = std_z * std_z;

        // VX in ECEF
        temp_bestxyza.assign(sentence, separator_pos[18] + 1, separator_pos[19] - separator_pos[18] - 1);
        msg_gnssodometry_.twist.twist.linear.x = stringToNum<double>(temp_bestxyza);

        // VY in ECEF
        temp_bestxyza.assign(sentence, separator_pos[19] + 1, separator_pos[20] - separator_pos[19] - 1);
        msg_gnssodometry_.twist.twist.linear.y = stringToNum<double>(temp_bestxyza);

        // VZ in ECEF
        temp_bestxyza.assign(sentence, separator_pos[20] + 1, separator_pos[21] - separator_pos[20] - 1);
        msg_gnssodometry_.twist.twist.linear.z = stringToNum<double>(temp_bestxyza);

        // Standard deviation VX
        temp_bestxyza.assign(sentence, separator_pos[21] + 1, separator_pos[22] - separator_pos[21] - 1);
        double std_vx = stringToNum<double>(temp_bestxyza);
        msg_gnssodometry_.twist.covariance[0] = std_vx * std_vx;

        // Standard deviation VY
        temp_bestxyza.assign(sentence, separator_pos[22] + 1, separator_pos[23] - separator_pos[22] - 1);
        double std_vy = stringToNum<double>(temp_bestxyza);
        msg_gnssodometry_.twist.covariance[7] = std_vy * std_vy;

        // Standard deviation VZ
        temp_bestxyza.assign(sentence, separator_pos[23] + 1, separator_pos[24] - separator_pos[23] - 1);
        double std_vz = stringToNum<double>(temp_bestxyza);
        msg_gnssodometry_.twist.covariance[14] = std_vz * std_vz;

        msg_navsatfix_.position_covariance_type = 3;
    }
}

void UM482Driver::handleGptraSentence(const std::string& sentence, std::vector<int>& separator_pos)
{
    struct SeparatorFormat {
        int first;
        int headerLength;
        int totalCommas;
    };
    SeparatorFormat gptra;
    gptra.first = 6;
    gptra.totalCommas = 8;

    // Find all commas in the sentence
    for(int i = 1; i <= gptra.totalCommas - 1; i++)
    {
        if (separator_pos[i-1] == std::string::npos) break;
        separator_pos.push_back(sentence.find(",", separator_pos[i-1] + 1));
    }

    msg_gnssodometry_.header.stamp = ros::Time::now();
    msg_gnssodometry_.header.frame_id = "gnss";

    if (separator_pos.size() > 4)
    {
        std::string temp_gptra;
        
        // Heading
        temp_gptra.assign(sentence, separator_pos[1] + 1, separator_pos[2] - separator_pos[1] - 1);
        double gnss_heading = stringToNum<double>(temp_gptra) / 180 * PI;

        // Pitch
        temp_gptra.assign(sentence, separator_pos[2] + 1, separator_pos[3] - separator_pos[2] - 1);
        double gnss_pitch = stringToNum<double>(temp_gptra) / 180 * PI;

        // Roll
        temp_gptra.assign(sentence, separator_pos[3] + 1, separator_pos[4] - separator_pos[3] - 1);
        double gnss_roll = stringToNum<double>(temp_gptra) / 180 * PI;

        Eigen::Vector3d ea0(gnss_heading, gnss_pitch, gnss_roll);
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());

        Eigen::Quaterniond q;
        q = R;

        msg_gnssodometry_.pose.pose.orientation.x = q.x();
        msg_gnssodometry_.pose.pose.orientation.y = q.y();
        msg_gnssodometry_.pose.pose.orientation.z = q.z();
        msg_gnssodometry_.pose.pose.orientation.w = q.w();
    }
}

void UM482Driver::run()
{
    ros::Rate loop_rate(update_rate_); // 使用参数控制的频率

    while(ros::ok())
    {
        if(ser_.available())
        {
            std::vector<std::string> sentences = ser_.readlines();

            for(const auto& sentence : sentences)
            {
                parseNmeaSentence(sentence);
            }

            // Only publish if we have a valid fix
            if(msg_navsatfix_.status.status == 1)
            {
                gpgga_pub_.publish(msg_gpgga_);
                odometry_pub_.publish(msg_gnssodometry_);
                navsatfix_pub_.publish(msg_navsatfix_);
            }

            ser_.flush();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "um482_driver");
    
    ros::NodeHandle nh;
    ros::NodeHandle param_nh("~");

    UM482Driver driver(nh, param_nh);

    if (!driver.initialize())
    {
        ROS_ERROR("Failed to initialize UM482 driver");
        return -1;
    }

    ROS_INFO("UM482 driver initialized successfully");
    driver.run();

    return 0;
}