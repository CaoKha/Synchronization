#pragma once

#include <libserial/SerialPort.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#define _USE_MATH_DEFINES

/* imu class */
class bno055_arduino_mod
{
    std::string device_;
    LibSerial::SerialPort Serial_port;
    const double g_ = 9.80665;
    

public:
    int rate_ = 100;
    double roll_ = 0, pitch_ = 0, yaw_ = 0;
    ros::Time ros_time;
    uint32_t imu_sequence = 0;

    ros::Publisher imu_publisher;
    ros::Publisher mag_publisher;
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;

    bno055_arduino_mod(ros::NodeHandle &n_imu);
    int openPort();
    void update();
    void publish_data();
    void closePort();
    ~bno055_arduino_mod(){};
};