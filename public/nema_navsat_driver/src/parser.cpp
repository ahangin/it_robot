/**
 * @file parser.cpp
 * @brief NMEA Sentence Parser Implementation - Parsing utility functions
 * 
 * @description
 * This file implements NMEA 0183 sentence parsing functions including:
 * - safe_float/safe_int: Safe string to number conversion with error handling
 * - convert_latitude/convert_longitude: NMEA coordinate to decimal degrees
 * - convert_time/convert_time_rmc: NMEA time to Unix timestamp conversion
 * - convert_status_flag: Parse GPS fix status flags
 * - convert_knots_to_mps: Speed unit conversion
 * - convert_deg_to_rads: Angle unit conversion
 * - check_nmea_checksum: NMEA checksum validation
 * - parse_nmea_sentence: Main parsing function for all supported sentence types
 * 
 * @author ahangin, AI
 * @email ahangin@todo.todo
 * @version 1.0.0
 * @date 2026-01-15
 * @copyright Copyright (c) 2024 Robot Hardware Team
 * 
 * @par Revision History:
 * - 2026-01-15, ahangin: Initial version
 * - 2026-03-10, ahangin: Added GST and VTG sentence parsing support
 * 
 * @see parser.h
 */
#include "nema_navsat_driver/parser.h"
#include <ros/ros.h>
#include <ctime>
#include <chrono>
#include <algorithm>
#include <sstream>

namespace libnmea_navsat_driver
{


double safe_float(const std::string& field)
{
    try
    {
        if (field.empty())
        {
            return std::nan("");
        }
        return std::stod(field);
    }
    catch (const std::exception& e)
    {
        return std::nan("");
    }
}

int safe_int(const std::string& field)
{
    try
    {
        if (field.empty())
        {
            return 0;
        }
        return std::stoi(field);
    }
    catch (const std::exception& e)
    {
        return 0;
    }
}

double convert_latitude(const std::string& field)
{
    if (field.length() < 2)
    {
        return std::nan("");
    }
    double degrees = safe_float(field.substr(0, 2));
    double minutes = safe_float(field.substr(2));
    return degrees + minutes / 60.0;
}

double convert_longitude(const std::string& field)
{
    if (field.length() < 3)
    {
        return std::nan("");
    }
    double degrees = safe_float(field.substr(0, 3));
    double minutes = safe_float(field.substr(3));
    return degrees + minutes / 60.0;
}

std::pair<double, double> convert_time(const std::string& nmea_utc)
{
    // If one of the time fields is empty, return NaN seconds
    if (nmea_utc.length() < 6 || 
        nmea_utc.substr(0, 2).empty() || 
        nmea_utc.substr(2, 2).empty() || 
        nmea_utc.substr(4, 2).empty())
    {
        return {std::nan(""), std::nan("")};
    }
    
    // Get current time in UTC for date information
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* utc_time = std::gmtime(&now_time);
    
    int hours = safe_int(nmea_utc.substr(0, 2));
    int minutes = safe_int(nmea_utc.substr(2, 2));
    int seconds = safe_int(nmea_utc.substr(4, 2));
    long nanosecs = 0;
    
    // If the seconds includes a decimal portion, convert it to nanoseconds
    if (nmea_utc.length() > 7)
    {
        std::string frac = nmea_utc.substr(7);
        nanosecs = safe_int(frac) * std::pow(10, 9 - frac.length());
    }
    
    // Resolve the ambiguity of day
    int day_offset = static_cast<int>((utc_time->tm_hour - hours) / 12.0);
    utc_time->tm_mday += day_offset;
    utc_time->tm_hour = hours;
    utc_time->tm_min = minutes;
    utc_time->tm_sec = seconds;
    
    std::time_t unix_secs = std::mktime(utc_time);
    return {static_cast<double>(unix_secs), static_cast<double>(nanosecs)};
}

std::pair<double, double> convert_time_rmc(const std::string& date_str, const std::string& time_str)
{
    // If one of the time fields is empty, return NaN seconds
    if (date_str.length() < 6 || time_str.length() < 6 ||
        date_str.substr(0, 6).empty() ||
        time_str.substr(0, 2).empty() ||
        time_str.substr(2, 2).empty() ||
        time_str.substr(4, 2).empty())
    {
        return {std::nan(""), std::nan("")};
    }
    
    // Get current year for century resolution
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::gmtime(&now_time);
    int pc_year = local_time->tm_year + 1900;
    
    // Resolve the ambiguity of century
    int utc_year = safe_int(date_str.substr(4, 2));
    int years = pc_year + static_cast<int>((pc_year % 100 - utc_year) / 50.0);
    
    int months = safe_int(date_str.substr(2, 2));
    int days = safe_int(date_str.substr(0, 2));
    
    int hours = safe_int(time_str.substr(0, 2));
    int minutes = safe_int(time_str.substr(2, 2));
    int seconds = safe_int(time_str.substr(4, 2));
    long nanosecs = 0;
    
    if (time_str.length() > 7)
    {
        std::string frac = time_str.substr(7);
        nanosecs = safe_int(frac) * std::pow(10, 9 - frac.length());
    }
    
    std::tm time_struct = {};
    time_struct.tm_year = years - 1900;
    time_struct.tm_mon = months - 1;
    time_struct.tm_mday = days;
    time_struct.tm_hour = hours;
    time_struct.tm_min = minutes;
    time_struct.tm_sec = seconds;
    time_struct.tm_isdst = 0;
    
    std::time_t unix_secs = std::mktime(&time_struct);
    return {static_cast<double>(unix_secs), static_cast<double>(nanosecs)};
}

bool convert_status_flag(const std::string& status_flag)
{
    if (status_flag == "A")
    {
        return true;
    }
    else if (status_flag == "V")
    {
        return false;
    }
    else
    {
        return false;
    }
}

double convert_knots_to_mps(double knots)
{
    return knots * 0.514444444444;
}

double convert_deg_to_rads(double degs)
{
    return degs * M_PI / 180.0;
}

bool check_nmea_checksum(const std::string& nmea_sentence)
{
    // Split sentence by '*' character (same as Python's split('*'))
    size_t checksum_pos = nmea_sentence.find('*');
    if (checksum_pos == std::string::npos)
    {
        // No checksum bytes were found... improperly formatted/incomplete NMEA data?
        return false;
    }
    
    // Extract transmitted checksum (after '*')
    std::string transmitted_checksum = nmea_sentence.substr(checksum_pos + 1);
    // Remove any trailing characters (like carriage return)
    transmitted_checksum.erase(std::remove_if(transmitted_checksum.begin(), 
                                               transmitted_checksum.end(), 
                                               [](char c){ return !std::isalnum(c); }), 
                               transmitted_checksum.end());
    
    if (transmitted_checksum.length() != 2)
    {
        return false;
    }
    
    // Remove the '$' at the front (same as Python's [1:])
    std::string data_to_checksum = nmea_sentence.substr(1, checksum_pos - 1);
    
    // Calculate checksum by XORing all characters (same as Python's ord(c))
    unsigned char checksum = 0;
    for (char c : data_to_checksum)
    {
        checksum ^= static_cast<unsigned char>(c);
    }
    
    // Compare checksums (same as Python's "%02X" % checksum)
    char calculated_checksum_str[3];
    std::snprintf(calculated_checksum_str, sizeof(calculated_checksum_str), "%02X", checksum);
    
    // Convert transmitted checksum to uppercase for comparison
    std::transform(transmitted_checksum.begin(), transmitted_checksum.end(), 
                   transmitted_checksum.begin(), ::toupper);
    
    return std::string(calculated_checksum_str) == transmitted_checksum;
}

bool parse_nmea_sentence(const std::string& nmea_sentence,
                         std::map<std::string, std::map<std::string, double>>& parsed)
{
    // Check for valid NMEA sentence
    std::string trimmed = nmea_sentence;
    // Remove carriage return or new line
    trimmed.erase(std::remove(trimmed.begin(), trimmed.end(), '\r'), trimmed.end());
    trimmed.erase(std::remove(trimmed.begin(), trimmed.end(), '\n'), trimmed.end());
    
    std::regex nmea_regex(R"(^\$(GP|WT|GN|GL|IN).*\*[0-9A-Fa-f]{2}$)");
    if (!std::regex_match(trimmed, nmea_regex))
    {
        ROS_DEBUG("Regex didn't match, sentence not valid NMEA? Sentence was: %s", 
                  trimmed.c_str());
        return false;
    }
    
    // Split fields by comma or asterisk
    std::vector<std::string> fields;
    std::stringstream ss(trimmed);
    std::string field;
    
    while (std::getline(ss, field, ','))
    {
        // Remove asterisk and checksum from last field
        size_t asterisk_pos = field.find('*');
        if (asterisk_pos != std::string::npos)
        {
            field = field.substr(0, asterisk_pos);
        }
        fields.push_back(field);
    }
    
    if (fields.empty())
    {
        return false;
    }
    
    // Extract sentence type (ignore $ and talker ID)
    std::string sentence_type = fields[0].substr(3);
    
    // Parse based on sentence type
    std::map<std::string, double> data;
    
    if (sentence_type == "GGA")
    {
        // GGA: $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,aaaa*hh
        if (fields.size() >= 12)
        {
            data["fix_type"] = safe_float(fields[6]);
            data["latitude"] = convert_latitude(fields[2]);
            data["latitude_direction"] = static_cast<double>(fields[3][0]);
            data["longitude"] = convert_longitude(fields[4]);
            data["longitude_direction"] = static_cast<double>(fields[5][0]);
            data["altitude"] = safe_float(fields[9]);
            data["mean_sea_level"] = safe_float(fields[11]);
            data["hdop"] = safe_float(fields[8]);
            data["num_satellites"] = safe_float(fields[7]);
            
            auto time_result = convert_time(fields[1]);
            data["utc_time"] = time_result.first;
            data["utc_time_ns"] = time_result.second;
        }
        else
        {
            return false;
        }
    }
    else if (sentence_type == "RTK")
    { 
        //$WTRTK,-0.700,0.289,-0.032,0.758,-2.32,4.18,-0.11,4,21,5,24,91,-,1,10.7,26.6,311,0,2304.74827627,N,11415.24461437,E,0.00,0.11,52.37*5C 
        // $WTRTK字段对应（索引从0开始）：
        // 0:$WTRTK  1:差分X  2:差分Y  3:差分Z  4:差分R  5:角度X  6:角度Y  7:角度Z
        // 8:定向状态  9:无线连接状态  10:Ntrip连接状态  11:无线信号质量  12:无线通讯数据量
        // 13:GPS航向角  14:安装校准标志  15:电池电压  16:温度  17:基站距离  18:惯导标志
        // 19:惯导纬度(度分)  20:纬度标志  21:惯导经度(度分)  22:经度标志  23:惯导地速
        // 24:惯导航向角  25:惯导高度
        if (fields.size() >= 25)
        {
            // 差分距离字段（米）
            data["diff_x"] = safe_float(fields[1]);                   // 差分 X 距离
            data["diff_y"] = safe_float(fields[2]);                   // 差分 Y 距离
            data["diff_z"] = safe_float(fields[3]);                   // 差分 Z 距离
            data["diff_r"] = safe_float(fields[4]);                   // 差分 R 距离
            
            // 角度字段（±180°）
            data["angle_x"] = safe_float(fields[5]);                   // 角度 X
            data["angle_y"] = safe_float(fields[6]);                   // 角度 Y
            data["angle_z"] = safe_float(fields[7]);                   // 角度 Z
            data["orientation_status"] = safe_float(fields[8]);       // 定向状态
            
            // 连接/信号状态字段
            data["wireless_conn_status"] = safe_float(fields[9]);     // 无线连接状态
            data["ntrip_conn_status"] = safe_float(fields[10]);       // Ntrip 连接状态
            data["wireless_signal_quality"] = safe_float(fields[11]); // 无线信号质量
            data["wireless_data_vol"] = safe_float(fields[12]);       // 无线通讯数据量（byte/秒）
            
            // GPS 航向角（特殊处理：--表示地速<5km/h，设为 NaN）
            if (fields[13] == "--")
            {
                data["gps_heading"] = std::numeric_limits<double>::quiet_NaN();
            }
            else
            {
                data["gps_heading"] = safe_float(fields[13]);
            }
            
            // 设备状态字段
            data["calibration_flag"] = safe_float(fields[14]);        // 安装校准标志（0=未校准，1=已校准）
            data["battery_voltage"] = safe_float(fields[15]);         // 电池电压（V）
            data["temperature"] = safe_float(fields[16]);             // 温度（℃）
            data["base_station_distance"] = safe_float(fields[17]);   // 基站距离（米）
            data["ins_flag"] = safe_float(fields[18]);                // 惯导标志
            
            // 惯导位置字段（度分转度，兼容 GGA 的转换逻辑）
            data["ins_latitude"] = convert_latitude(fields[19]);      // 惯导纬度（度）
            data["ins_latitude_dir"] = static_cast<double>(fields[20][0]); // 纬度标志（N/S）
            data["ins_longitude"] = convert_longitude(fields[21]);    // 惯导经度（度）
            data["ins_longitude_dir"] = static_cast<double>(fields[22][0]); // 经度标志（E/W）
            
            // 惯导运动字段
            data["ins_ground_speed"] = safe_float(fields[23]);        // 惯导地速（km/h）
            data["ins_heading"] = safe_float(fields[24]);             // 惯导航向角（0-360°）
            data["ins_altitude"] = safe_float(fields[25]);            // 惯导高度（米）
        }
        else
        {
            ROS_DEBUG("WTRTK sentence has insufficient fields: expected >=26, got %zu", fields.size());
            return false;
        }
    }
    else if (sentence_type == "RMC")
    {
        // RMC: $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a*hh
        // GNRMC: 组合导航推荐定位信息（NMEA 0183标准，兼容$GPRMC/$GLRMC）
        // 完整字段格式（索引从0开始）：
        // 0: $GNRMC          - 语句头（GN=多系统融合，GP=GPS，GL=GLONASS）
        // 1: hhmmss.ss       - UTC时间（时分秒.毫秒）
        // 2: A/V             - 定位状态（A=有效定位，V=无效/无定位）
        // 3: ddmm.mmmmmmm    - 纬度（度分格式，dd=度，mm.mmmmmmm=分）
        // 4: N/S             - 纬度半球（N=北纬，S=南纬）
        // 5: dddmm.mmmmmmm   - 经度（度分格式，ddd=度，mm.mmmmmmm=分）
        // 6: E/W             - 经度半球（E=东经，W=西经）
        // 7: x.x             - 对地速度（单位：节，1节=1.852km/h）
        // 8: x.x             - 对地航向（真北方向，单位：度，0~359.9°）
        // 9: ddmmyy          - UTC日期（日月年，dd=日，mm=月，yy=年）
        // 10: x.x            - 磁偏角数值（单位：度）
        // 11: E/W            - 磁偏角方向（E=东偏，W=西偏）
        // 12: A/D/C/N/E      - 模式指示（A=自主，D=差分，C=组合，N=无定位，E=估算）
        // 13: *XX            - 校验位（十六进制，可选）
        
        // 基础解析：至少需要10个字段（覆盖核心定位信息），扩展解析支持到13个字段
        if (fields.size() >= 10)
        {
            data["fix_valid"] = convert_status_flag(fields[2]) ? 1.0 : 0.0;
            data["latitude"] = convert_latitude(fields[3]);
            data["latitude_direction"] = static_cast<double>(fields[4][0]);
            data["longitude"] = convert_longitude(fields[5]);
            data["longitude_direction"] = static_cast<double>(fields[6][0]);
            data["speed"] = convert_knots_to_mps(safe_float(fields[7]));
            data["true_course"] = convert_deg_to_rads(safe_float(fields[8]));
            
            auto time_result = convert_time_rmc(fields[9], fields[1]);
            data["utc_time"] = time_result.first;
            data["utc_time_ns"] = time_result.second;

            // 扩展解析：磁偏角（可选字段，fields>=12 时解析）
            if (fields.size() >= 12)
            {
                data["magnetic_variation"] = safe_float(fields[10]);       // 磁偏角数值（度）
                data["magnetic_variation_dir"] = static_cast<double>(fields[11][0]); // 磁偏角方向（E/W 的 ASCII 值）
            }
            
            // 扩展解析：定位模式（可选字段，fields>=13 时解析）
            if (fields.size() >= 13)
            {
                data["mode_indicator"] = static_cast<double>(fields[12][0]); // 模式指示（A/D/C 等的 ASCII 值）
            }
        }
        else
        {
            return false;
        }
    }
    else if (sentence_type == "GST")
    {
        // GST: $--GST,hhmmss.ss,x.x,x.x,x.x,x.x,x.x,x.x,x.x*hh
        if (fields.size() >= 9)
        {
            auto time_result = convert_time(fields[1]);
            data["utc_time"] = time_result.first;
            data["ranges_std_dev"] = safe_float(fields[2]);
            data["semi_major_ellipse_std_dev"] = safe_float(fields[3]);
            data["semi_minor_ellipse_std_dev"] = safe_float(fields[4]);
            data["semi_major_orientation"] = safe_float(fields[5]);
            data["lat_std_dev"] = safe_float(fields[6]);
            data["lon_std_dev"] = safe_float(fields[7]);
            data["alt_std_dev"] = safe_float(fields[8]);
        }
        else
        {
            return false;
        }
    }
    else if (sentence_type == "HDT")
    {
        // HDT: $--HDT,x.x,T*hh
        if (fields.size() >= 2)
        {
            data["heading"] = safe_float(fields[1]);
        }
        else
        {
            return false;
        }
    }
    else if (sentence_type == "VTG")
    {
        // VTG: $--VTG,x.x,T,x.x,M,x.x,N,x.x,K*hh
        if (fields.size() >= 6)
        {
            data["true_course"] = convert_deg_to_rads(safe_float(fields[1]));
            data["speed"] = convert_knots_to_mps(safe_float(fields[5]));
        }
        else
        {
            return false;
        }
    }
    else
    {
        ROS_DEBUG("Sentence type %s not in parse map, ignoring.", sentence_type.c_str());
        return false;
    }
    
    parsed[sentence_type] = data;
    return true;
}

} // namespace libnmea_navsat_driver