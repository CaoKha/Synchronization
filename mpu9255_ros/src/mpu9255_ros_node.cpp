#include <iostream>   // C++ standard in/out
#include <math.h>  // math lib
#include <cstring> // C string lib
#include <string>  // standard string lib
#include <mpu9255_ros/mpu9255_ros.hpp>

/*--------------------main--------------------*/
int main(int argc, char **argv)
{
    /* Set up imu */
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle imu_node_handler;
    mpu9255_arduino imu(imu_node_handler);
    imu.openPort(); 
    std::cout << "imu is running" << std::endl;

    /* data acquisition */
    ros::Rate loop_rate(imu.rate_);
    while (ros::ok())
    {
        imu.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    /* closing node */
    imu.closePort();
    std::cout << "\nexit success!" << std::endl;
}