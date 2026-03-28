/**
 * @file parser.h
 * @brief NMEA Sentence Parser Library - Utility functions for parsing NMEA GPS data
 * 
 * @description
 * This header file provides utility functions for parsing NMEA 0183 standard GPS
 * sentences. It includes functions for coordinate conversion, time conversion,
 * checksum validation, and sentence parsing. Supports GGA, RMC, GST, HDT, and
 * VTG sentence types.
 * 
 * @author ahangin, AI
 * @email ahangin@todo.todo
 * @version 1.0.0
 * @date 2026-01-15
 * @copyright Copyright (c) 2024 Robot Hardware Team
 * 
 * @par Revision History:
 * - 2026-01-15, ahangin: Initial version
 * - 2026-03-10, ahangin: Added GST and VTG sentence support
 * 
 * @namespace libnmea_navsat_driver
 * @brief Namespace containing all NMEA parsing utilities
 */
#ifndef LIBNMEA_PARSER_H
#define LIBNMEA_PARSER_H

#include <string>
#include <map>
#include <vector>
#include <cmath>
#include <regex>

namespace libnmea_navsat_driver
{

// NMEA parsed data structure
struct NMEAParsedData
{
    std::map<std::string, double> values;
    std::map<std::string, std::string> string_values;
};

// Safe conversion functions
double safe_float(const std::string& field);
int safe_int(const std::string& field);

// Coordinate conversion functions
double convert_latitude(const std::string& field);
double convert_longitude(const std::string& field);

// Time conversion functions
std::pair<double, double> convert_time(const std::string& nmea_utc);
std::pair<double, double> convert_time_rmc(const std::string& date_str, const std::string& time_str);

// Status and unit conversion functions
bool convert_status_flag(const std::string& status_flag);
double convert_knots_to_mps(double knots);
double convert_deg_to_rads(double degs);

// Main parsing function
bool parse_nmea_sentence(const std::string& nmea_sentence, 
                         std::map<std::string, std::map<std::string, double>>& parsed);

// Checksum validation
bool check_nmea_checksum(const std::string& nmea_sentence);

} // namespace libnmea_navsat_driver

#endif // LIBNMEA_PARSER_H