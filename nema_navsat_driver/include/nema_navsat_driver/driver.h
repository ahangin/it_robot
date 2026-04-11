/**
 * @file driver.h
 * @brief ROS NMEA GPS Driver - Main driver class for processing NMEA GPS data
 * 
 * @description
 * This header file defines the RosNMEADriver class which handles NMEA GPS sentence
 * processing and publishes ROS messages for GPS fix, velocity, heading, and time
 * reference information. It supports multiple NMEA sentence types including GGA,
 * RMC, GST, HDT, and VTG.
 * 
 * @author ahangin, AI
 * @email ahangin@todo.todo
 * @version 1.0.0
 * @date 2026-01-15
 * @copyright Copyright (c) 2024 Robot Hardware Team
 * 
 * @par Revision History:
 * - 2026-01-15, ahangin: Initial version
 * - 2026-03-10, ahangin: Added support for multiple NMEA sentence types
 * 
 * @note This driver is compatible with ROS Kinetic and newer versions
 */
#ifndef LIBNMEA_NAVSAT_DRIVER_H
#define LIBNMEA_NAVSAT_DRIVER_H

#include <ros/ros.h>
#include <ros/param.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <string>
#include <map>
#include <vector>
#include <cmath>

class RosNMEADriver
{
private:
    ros::Publisher fix_pub_;
    ros::Publisher vel_pub_;
    ros::Publisher heading_pub_;
    ros::Publisher time_ref_pub_;
    ros::Publisher fix_vel_pub_;
    
    ros::NodeHandle nh_;
    
    std::string time_ref_source_;
    bool use_RMC_;
    bool use_GNSS_time_;
    bool valid_fix_;
    bool using_receiver_epe_;
    
    double default_epe_quality0_;
    double default_epe_quality1_;
    double default_epe_quality2_;
    double default_epe_quality4_;
    double default_epe_quality5_;
    double default_epe_quality9_;
    
    double lon_std_dev_;
    double lat_std_dev_;
    double alt_std_dev_;
    
    ros::Time last_valid_fix_time_;
    
    std::map<int, std::vector<double>> gps_qualities_;
    
    std::string frame_id_;
    
    double latest_utm_easting_;
    double latest_utm_northing_;
    double latest_heading_;

public:
    RosNMEADriver();
    
    void initializeGPSQualities();
    
    bool addSentence(const std::string& nmea_string, const std::string& frame_id, 
                     const ros::Time& timestamp = ros::Time());
    
    void publishFix(const ros::Time& fix_time, const std::string& frame_id,
                   double latitude, double longitude, double altitude,
                   double hdop, int fix_type, int num_sats);
    
    void publishVel(double speed, double true_course, const ros::Time& timestamp);
    
    void publishHeading(double heading);
    
    void publishTimeRef(const ros::Time& timestamp, double utc_time);
};

// Helper function declarations
std::string getFrameId();
void nmeaSentenceCallback(const std_msgs::String::ConstPtr& msg, RosNMEADriver* driver);

#endif // LIBNMEA_NAVSAT_DRIVER_H