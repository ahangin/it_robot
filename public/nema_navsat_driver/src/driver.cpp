/**
 * @file driver.cpp
 * @brief ROS NMEA GPS Driver Implementation - Main driver class implementation
 * 
 * @description
 * This file implements the RosNMEADriver class which processes NMEA GPS sentences
 * and publishes corresponding ROS messages. Key features include:
 * - GPS fix publication (sensor_msgs/NavSatFix)
 * - Velocity publication (geometry_msgs/TwistStamped)
 * - Heading publication (std_msgs/Float64)
 * - Time reference publication (sensor_msgs/TimeReference)
 * - Support for multiple GPS quality levels and EPE estimation
 * 
 * @author ahangin, AI
 * @email ahangin@todo.todo
 * @version 1.0.0
 * @date 2026-01-15
 * @copyright Copyright (c) 2024 Robot Hardware Team
 * 
 * @par Revision History:
 * - 2026-01-15, ahangin: Initial version
 * - 2026-03-10, ahangin: Enhanced fix type handling and covariance estimation
 * 
 * @see driver.h
 * @see parser.h
 */
#include "nema_navsat_driver/driver.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <string>
#include <map>
#include <vector>
#include <limits>  // for std::numeric_limits
#include "nema_navsat_driver/parser.h"


RosNMEADriver::RosNMEADriver()
{
    nh_ = ros::NodeHandle("~");
    
    // Initialize publishers
    fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("fix", 10);
    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("vel", 10);
    heading_pub_ = nh_.advertise<std_msgs::Float64>("heading", 10);
    time_ref_pub_ = nh_.advertise<sensor_msgs::TimeReference>("time_reference", 10);
    //fix_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("fix_velocity", 10);
    
    // Initialize parameters
    time_ref_source_ = "gps";
    use_RMC_ = false;
    use_GNSS_time_ = false;
    valid_fix_ = false;
    using_receiver_epe_ = false;
    
    // Default EPE values
    default_epe_quality0_ = 1000000;
    default_epe_quality1_ = 4.00;
    default_epe_quality2_ = 0.10;
    default_epe_quality4_ = 0.02;
    default_epe_quality5_ = 0.40;
    default_epe_quality9_ = 3.00;
    
    lon_std_dev_ = std::numeric_limits<double>::quiet_NaN();
    lat_std_dev_ = std::numeric_limits<double>::quiet_NaN();
    alt_std_dev_ = std::numeric_limits<double>::quiet_NaN();
    
    latest_utm_easting_ = 0.0;
    latest_utm_northing_ = 0.0;
    latest_heading_ = 0.0;
    
    initializeGPSQualities();
}

void RosNMEADriver::initializeGPSQualities()
{
    gps_qualities_[-1] = {0.00, 0.0, 2};
    gps_qualities_[0] = {default_epe_quality0_, 0.0, 2};
    gps_qualities_[1] = {default_epe_quality1_, 1.0, 2};
    gps_qualities_[2] = {default_epe_quality2_, 2.0, 2};
    gps_qualities_[4] = {default_epe_quality4_, 2.0, 2};
    gps_qualities_[5] = {default_epe_quality5_, 2.0, 2};
    gps_qualities_[9] = {default_epe_quality9_, 2.0, 2};
}

// Helper function to apply direction sign to coordinates
inline void applyDirectionSign(double& coordinate, char direction)
{
    if (direction == 'S' || direction == 'W')
    {
        coordinate = -coordinate;
    }
}

bool RosNMEADriver::addSentence(const std::string& nmea_string, const std::string& frame_id,
                                const ros::Time& timestamp)
{
    frame_id_ = frame_id;
    
    std::map<std::string, std::map<std::string, double>> parsed;
    if (!libnmea_navsat_driver::parse_nmea_sentence(nmea_string, parsed))
    {
        return false;
    }
    
    for (const auto& sentence : parsed)
    {
        std::string sentence_type = sentence.first;
        const auto& data = sentence.second;
        
        if (sentence_type == "GGA")
        {
            double fix_type = data.count("fix_type") ? data.at("fix_type") : 0;
            double latitude = data.count("latitude") ? data.at("latitude") : 0;
            double longitude = data.count("longitude") ? data.at("longitude") : 0;
            double altitude = data.count("altitude") ? data.at("altitude") : 0;
            double hdop = data.count("hdop") ? data.at("hdop") : 0;
            int num_sats = static_cast<int>(data.count("num_satellites") ? data.at("num_satellites") : 0);
            
            // Handle latitude/longitude direction using helper function
            if (data.count("latitude_direction"))
            {
                char lat_dir = static_cast<char>(data.at("latitude_direction"));
                applyDirectionSign(latitude, lat_dir);
            }
            
            if (data.count("longitude_direction"))
            {
                char lon_dir = static_cast<char>(data.at("longitude_direction"));
                applyDirectionSign(longitude, lon_dir);
            }
            
            ros::Time fix_time = ros::Time::now();
            if (data.count("utc_time"))
            {
                fix_time = ros::Time(data.at("utc_time"), data.at("utc_time_ns"));
            }
            
            publishFix(fix_time, frame_id_, latitude, longitude, altitude, hdop, 
                      static_cast<int>(fix_type), num_sats);
        }
        else if (sentence_type == "RTK")
        { 
            double heading = data.count("gps_heading") ? data.at("gps_heading") : 0;
            publishHeading(heading);
        }
        else if (sentence_type == "RMC")
        {
            double speed = data.count("speed") ? data.at("speed") : 0;
            double true_course = data.count("true_course") ? data.at("true_course") : 0;
            double latitude = data.count("latitude") ? data.at("latitude") : 0;
            double longitude = data.count("longitude") ? data.at("longitude") : 0;
            
            // Handle latitude/longitude direction using helper function
            if (data.count("latitude_direction"))
            {
                char lat_dir = static_cast<char>(data.at("latitude_direction"));
                applyDirectionSign(latitude, lat_dir);
            }
            
            if (data.count("longitude_direction"))
            {
                char lon_dir = static_cast<char>(data.at("longitude_direction"));
                applyDirectionSign(longitude, lon_dir);
            }

            ros::Time fix_time = ros::Time::now();
            if (data.count("utc_time"))
            {
                fix_time = ros::Time(data.at("utc_time"), data.at("utc_time_ns"));
            }
            publishTimeRef(fix_time, data.count("utc_time") ? data.at("utc_time") : 0);
            
            publishVel(speed, true_course, fix_time);
            
        }
        else if (sentence_type == "HDT")
        {
            double heading = data.count("heading") ? data.at("heading") : 0;
            //publishHeading(heading);
        }
    }
    
    return true;
}

void RosNMEADriver::publishFix(const ros::Time& fix_time, const std::string& frame_id,
                               double latitude, double longitude, double altitude,
                               double hdop, int fix_type, int num_sats)
{
    sensor_msgs::NavSatFix fix_msg;
    fix_msg.header.stamp = fix_time;
    fix_msg.header.frame_id = frame_id;
    
    fix_msg.latitude = latitude;
    fix_msg.longitude = longitude;
    fix_msg.altitude = altitude;
    
    // Set status
    if (fix_type == 0 || fix_type == 3)
    {
        fix_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }
    else if (fix_type == 1)
    { 
        fix_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    }
    else if (fix_type == 2 || fix_type == 5 || fix_type == 9)
    {
        fix_msg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
    }
    else if (fix_type == 4)
    {
        fix_msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    }
    
    if (gps_qualities_.count(fix_type))
    {
        fix_msg.status.service = gps_qualities_[fix_type][1];
    }
    else
    {
        fix_msg.status.service = 1;
    }
    
    // Set position covariance
    double epe = 0.0;
    if (gps_qualities_.count(fix_type))
    {
        epe = gps_qualities_[fix_type][0];
    }
    
    double cov[9] = {epe*epe, 0, 0, 0, epe*epe, 0, 0, 0, epe*epe};
    for (int i = 0; i < 9; i++)
    {
        fix_msg.position_covariance[i] = cov[i];
    }
    fix_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    
    fix_pub_.publish(fix_msg);
}

void RosNMEADriver::publishVel(double speed, double true_course, const ros::Time& timestamp)
{
    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header.stamp = timestamp;
    vel_msg.header.frame_id = frame_id_;
    
    vel_msg.twist.linear.x = speed;
    vel_msg.twist.linear.y = 0;
    vel_msg.twist.linear.z = 0;
    
    vel_msg.twist.angular.x = 0;
    vel_msg.twist.angular.y = 0;
    vel_msg.twist.angular.z = true_course;
    
    vel_pub_.publish(vel_msg);
}

void RosNMEADriver::publishHeading(double heading)
{
    std_msgs::Float64 heading_msg;
    heading_msg.data = heading;
    heading_pub_.publish(heading_msg);
}

void RosNMEADriver::publishTimeRef(const ros::Time& timestamp, double utc_time)
{
    sensor_msgs::TimeReference time_ref_msg;
    time_ref_msg.header.stamp = ros::Time::now();
    time_ref_msg.header.frame_id = frame_id_;
    time_ref_msg.time_ref = timestamp;
    time_ref_msg.source = time_ref_source_;
    
    time_ref_pub_.publish(time_ref_msg);
}
