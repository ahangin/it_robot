# 和芯星通um482 GNSS网络RTK接收机ROS驱动
The ros driver of UM482 for unicorecomm um482.

## Prerequest and Installation

### 系统要求
- Ubuntu 16.04/18.04/20.04 (兼容ROS Kinetic/Melodic/Noetic)
- ROS Kinetic/Melodic/Noetic 安装完成

### 依赖项安装
sudo apt-get install ros-noetic-nmea-msgs

### protocols listening to 
gpgga/gptra/besyxyz protocol from serial port

### msgs publishing
`sensor_msgs::navSatFix` 
`nmea_msgs::gpgga` 
`nav_msgs::Odometry` 

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

The MIT License is a permissive open source license that allows for reuse within proprietary software provided that all copies of the licensed software include a copy of the MIT License terms and the copyright notice.