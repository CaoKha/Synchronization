#include <bno055_ros/bno055_ros.hpp>
/*--------------------main--------------------*/
int main(int argc, char **argv)
{
    /* set up imu */
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle imu_node_handler;
    bno055_arduino imu(imu_node_handler);
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