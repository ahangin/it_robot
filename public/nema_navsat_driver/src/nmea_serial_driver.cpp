/**
 * @file nmea_serial_driver.cpp
 * @brief NMEA Serial Driver Node - ROS node for serial GPS data acquisition
 * 
 * @description
 * This is the main entry point for the NMEA GPS driver ROS node. It handles:
 * - Serial port initialization and configuration using Boost.Asio
 * - Continuous reading of NMEA sentences from GPS receiver
 * - Line buffering and sentence validation
 * - Error handling for serial communication failures
 * - Graceful shutdown on ROS interrupt
 * 
 * @par Usage:
 * rosrun nema_navsat_driver nmea_serial_driver _port:=/dev/ttyUSB0 _baud:=460800
 * 
 * @author ahangin, AI
 * @email ahangin@todo.todo
 * @version 1.0.0
 * @date 2026-01-15
 * @copyright Copyright (c) 2024 Robot Hardware Team
 * 
 * @par Revision History:
 * - 2026-01-15, ahangin: Initial version
 * - 2026-03-10, ahangin: Improved error handling and logging
 * - 2026-03-12, ahangin: 优化日志输出，分级显示有效/无效数据；优化串口读取稳定性（超时重试、数据完整性校验）
 * @see driver.h
 * @see parser.h
 * 
 * @param port Serial port device path (default: /dev/ttyUSB0)
 * @param baud Serial port baud rate (default: 4800)
 * @param frame_id TF frame ID for published messages (default: gps)
 */
#include <ros/ros.h>
#include <ros/exception.h>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <sstream>
#include <stdexcept>
#include <cctype>
#include <thread>
#include <chrono>
#include <algorithm>  // 用于字符过滤
#include <errno.h>

#include <ros/console.h>
#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/propertyconfigurator.h>
#include <log4cxx/helpers/exception.h>
#include <ros/package.h>
#include <iomanip>     // 用于格式化打印

#include "nema_navsat_driver/driver.h"
#include "nema_navsat_driver/parser.h"

// 全局常量（可配置）
const size_t BUFFER_SIZE = 512;          // 串口读取缓冲区大小
const size_t MAX_NMEA_LENGTH = 256;      // NMEA语句最大长度（防溢出）
const std::string DEFAULT_FRAME_ID = "gps";
const std::string DEFAULT_SERIAL_PORT = "/dev/ttyUSB0";
const int DEFAULT_BAUD_RATE = 460800;
const int SERIAL_TIMEOUT_MS = 100;       // 串口读取超时时间
const int LOOP_SLEEP_MS = 10;            // 读取循环休眠时间
const int LOOP_RATE_HZ = 20;             // ROS循环频率

// 全局控制变量（线程安全）
std::atomic<bool> stop_threads_(false);  // 原子变量，保证线程安全

/**
 * @brief 校验串口参数有效性
 */
void validateSerialParams(const std::string& port, int baud)
{
    if (port.empty()) {
        throw std::invalid_argument("Serial port path is empty!");
    }
    if (baud <= 0 || baud > 921600) {
        throw std::invalid_argument(
            "Invalid baud rate: " + std::to_string(baud) + 
            " (valid range: 1-921600)"
        );
    }
}

/**
 * @brief RAII 串口资源管理类（自动关闭串口）
 */
class SerialPortGuard
{
public:
    explicit SerialPortGuard(serial::Serial& port, const std::string& port_path)
        : port_(port), port_path_(port_path)
    {
    }
    
    ~SerialPortGuard()
    {
        // 析构时自动关闭串口
        if (port_.isOpen())
        {
            try
            {
                port_.close();
                ROS_INFO("[SerialGuard] Serial port %s closed gracefully", port_path_.c_str());
            }
            catch (const serial::IOException& e)
            {
                ROS_WARN("[SerialGuard] Failed to close serial port %s: %s", port_path_.c_str(), e.what());
            }
        }
    }

    // 禁用拷贝/赋值（避免重复关闭）
    SerialPortGuard(const SerialPortGuard&) = delete;
    SerialPortGuard& operator=(const SerialPortGuard&) = delete;

private:
    serial::Serial& port_;
    std::string port_path_;
};

/**
 * @brief 初始化串口（适配 serial::Serial）
 */
void initSerialPort(serial::Serial& serial_port, const std::string& port, int baud)
{
    try
    {
        // 配置串口参数（8N1 标准格式）
        serial_port.setPort(port);
        serial_port.setBaudrate(baud);
        serial_port.setBytesize(serial::eightbits);    // 8 位数据位
        serial_port.setStopbits(serial::stopbits_one); // 1 位停止位
        serial_port.setParity(serial::parity_none);    // 无校验位
        serial_port.setFlowcontrol(serial::flowcontrol_none); // 无流控
        
        // 设置超时时间（读/写/等待均为 100ms）
        serial::Timeout to = serial::Timeout::simpleTimeout(SERIAL_TIMEOUT_MS);
        serial_port.setTimeout(to);

        // 打开串口（先关闭再打开，避免占用）
        if (serial_port.isOpen())
        {
            serial_port.close();
        }
        serial_port.open();
        
        if (!serial_port.isOpen())
        {
            throw std::runtime_error("Serial port open failed (unknown reason)");
        }

        ROS_INFO("[SerialInit] Serial port '%s' opened successfully (baud rate: %d)", 
                 port.c_str(), baud);
    }
    catch (const serial::IOException& e)
    {
        throw std::runtime_error("[SerialInit] Failed to open/configure serial port: " + std::string(e.what()));
    }
}

/**
 * @brief 校验 NMEA 语句有效性（基础校验）
 * @return true: 有效 NMEA 语句；false: 无效
 */
bool isValidNmeaSentence(const std::string& sentence)
{
    if (sentence.empty() || sentence.length() < 10 || sentence.length() > MAX_NMEA_LENGTH)
    {
        ROS_WARN("[NMEA] Invalid NMEA sentence length: %zu", sentence.length());
        return false;
    }
    if (sentence[0] != '$')
    {
        ROS_WARN("[NMEA] Invalid NMEA sentence format: %s", sentence.c_str());
        return false;
    }
    
    for (char c : sentence)
    {
        if (!std::isprint(static_cast<unsigned char>(c)) && c != ',')
        {
            //ROS_WARN("[NMEA] Invalid NMEA sentence character: %c in %s", c, sentence.c_str());
            return false;
        }
    }
    return true;
}

/**
 * @brief 串口读取循环（独立线程）
 */
void serialReadLoop(serial::Serial& serial_port, RosNMEADriver& driver, const std::string& frame_id)
{
    while (ros::ok() && !stop_threads_.load())
    {
        if (serial_port.isOpen())
        {
            try
            {
                std::string nmea_line = serial_port.readline(BUFFER_SIZE, "\n");
                if (!nmea_line.empty() && nmea_line[0] == '$')
                {
                    if (false == isValidNmeaSentence(nmea_line))
                    {
                        //ROS_INFO( "[NMEA] Received: %s", nmea_line.c_str());
                        
                        try
                        {
                            driver.addSentence(nmea_line, frame_id);
                        }
                        catch (const std::invalid_argument& e)
                        {
                            ROS_WARN("[NMEA] Invalid format: %s | Reason: %s",
                                            nmea_line.c_str(), e.what());
                        }
                        catch (const std::out_of_range& e)
                        {
                            ROS_WARN("[NMEA] Data out of range: %s | Reason: %s",
                                            nmea_line.c_str(), e.what());
                        }
                        catch (const std::exception& e)
                        {
                            ROS_WARN("[NMEA] Parse failed: %s | Reason: %s",
                                            nmea_line.c_str(), e.what());
                        }
                    }
                    else
                    {
                        ROS_WARN("[NMEA] Invalid sentence (filtered): %s", nmea_line.c_str());
                    }
                    
                    // 清空缓冲区，准备下一行
                    nmea_line.clear();
                }
            }
            catch (const serial::IOException& e)
            {
                ROS_ERROR_THROTTLE(1, "[SerialRead] Read error: %s (retrying...)", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 出错后短暂休眠
            }
        }
        else
        {
            ROS_WARN_THROTTLE(1, "[SerialRead] Serial port is closed, retrying connection...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // 尝试重新打开串口
            try
            {
                serial_port.open();
            }
            catch (const serial::IOException& e)
            {
                ROS_ERROR_THROTTLE(1, "[SerialRead] Reopen port failed: %s", e.what());
            }
        }
        // 降低 CPU 占用（10ms 休眠）
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_SLEEP_MS));
    }

    ROS_INFO("[SerialRead] Read loop exited gracefully");
}

int main(int argc, char** argv)
{
    std::string config_path = ros::package::getPath("nema_navsat_driver") + "/config/ros_log_config.ini";
    
    try
    {
        log4cxx::PropertyConfigurator::configure(config_path);
        ROS_INFO("log4cxx configured with file: %s", config_path.c_str());
    }
    catch (const log4cxx::helpers::Exception& e)
    {
        ROS_WARN("log4cxx config file load failed: %s, using basic configurator", e.what());
        log4cxx::BasicConfigurator::configure();
    }

    ros::init(argc, argv, "nmea_serial_driver");
    ros::NodeHandle nh("~");


    std::string serial_port_path;
    int serial_baud_rate;
    std::string frame_id;
    serial::Serial serial_port;

    try
    {
        nh.param<std::string>("port", serial_port_path, DEFAULT_SERIAL_PORT);
        nh.param<int>("baud", serial_baud_rate, DEFAULT_BAUD_RATE);
        nh.param<std::string>("frame_id", frame_id, DEFAULT_FRAME_ID);

        validateSerialParams(serial_port_path, serial_baud_rate);
        ROS_INFO("[Main] Configuring serial port: %s (baud: %d, frame_id: %s)",
                 serial_port_path.c_str(), serial_baud_rate, frame_id.c_str());

        initSerialPort(serial_port, serial_port_path, serial_baud_rate);
        SerialPortGuard serial_guard(serial_port, serial_port_path);

        RosNMEADriver gps_driver;

        std::thread read_thread(serialReadLoop, std::ref(serial_port), 
                                std::ref(gps_driver), frame_id);
        read_thread.detach();  // 分离线程，由 stop_threads_控制退出

        ros::Rate loop_rate(LOOP_RATE_HZ);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }

        stop_threads_.store(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等待线程退出

    }
    catch (const std::invalid_argument& e)
    {
        // 无效参数异常（如波特率非法、串口路径为空）
        ROS_FATAL("[Main] Invalid parameter error: %s", e.what());
        return 1;

    }
    catch (const ros::Exception& e)
    {
        // ROS 通用异常
        ROS_ERROR("[Main] ROS runtime error: %s", e.what());
        return 1;

    }
    catch (const std::runtime_error& e)
    {
        // 串口初始化/读取异常
        std::string error_msg = e.what();
        if (error_msg.find("Permission denied") != std::string::npos)
        {
            ROS_FATAL("[Main] %s | Hint: Run 'sudo chmod 666 %s' or add user to dialout group", 
                      error_msg.c_str(), serial_port_path.c_str());
        }
        else if (error_msg.find("No such file or directory") != std::string::npos)
        {
            ROS_FATAL("[Main] %s | Hint: Check if serial port %s exists (ls /dev/ttyUSB*)", 
                      error_msg.c_str(), serial_port_path.c_str());
        }
        else
        {
            ROS_FATAL("[Main] %s", error_msg.c_str());
        }
        return 1;

    }
    catch (const std::bad_alloc& e)
    {
        // 内存分配失败
        ROS_FATAL("[Main] Memory allocation failed: %s", e.what());
        return 1;

    }
    catch (const std::exception& e)
    {
        // 标准库通用异常
        ROS_ERROR("[Main] Unexpected error: %s", e.what());
        return 1;

    }
    catch (...)
    {
        // 未知异常（兜底）
        ROS_FATAL("[Main] Unknown fatal error occurred!");
        return 1;
    }

    ROS_INFO("[Main] NMEA serial driver node exited normally");
    return 0;
}