#pragma once

#include <libserial/SerialPort.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#define _USE_MATH_DEFINES

/* imu class */
class bno055_arduino
{
    std::string device_;
    LibSerial::SerialPort Serial_port;
    int imu_sequence = 0;
    const double g_ = 9.80665;
    

public:
    int rate_ = 100;
    double roll_ = 0, pitch_ = 0, yaw_ = 0;
    ros::Time ros_time;

    ros::Publisher imu_publisher;
    ros::Publisher mag_publisher;
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;

    bno055_arduino(ros::NodeHandle &n_imu);
    int openPort();
    void update();
    void closePort();
    ~bno055_arduino(){};
};