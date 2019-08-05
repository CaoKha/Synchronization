#pragma once 
#include <ros/ros.h>
#include <libserial/SerialPort.h> // serial lib
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#define _USE_MATH_DEFINES

class mpu9255_arduino
{
    std::string device_;
    LibSerial::SerialPort Serial_port;
    int imu_sequence;
    const double g_ = 9.80665;

public:
    int rate_ = 200;
    double roll_ = 0, pitch_ = 0, yaw_ = 0;

    ros::Publisher imu_publisher;
    ros::Publisher mag_publisher;
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;

    mpu9255_arduino(ros::NodeHandle &n_imu);
    int openPort();
    void update();
    void closePort();
    ~mpu9255_arduino(){};
};
