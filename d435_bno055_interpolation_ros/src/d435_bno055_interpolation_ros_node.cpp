#include <d435_ros_mod/d435_ros_mod.hpp>
#include <bno055_ros_mod/bno055_ros_mod.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <atomic>
#include <iostream>
std::atomic<bool> token;
int main(int argc, char **argv)
{
    token = true;

    /* imu initiation */
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle imu_node_handler;
    bno055_arduino_mod imu(imu_node_handler);
    imu.openPort();
    std::cout << "imu is running" << std::endl;

    /* camera initiation - this party can be replaced by using the pulses from the camera */
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle image_node_handler;
    realsenseD435_mod camera(image_node_handler);
    rs2::config cfg;
    rs2::pipeline pipe;
    rs2::frame_queue fq(2);
    cfg.enable_stream(RS2_STREAM_COLOR, camera.width_, camera.height_, RS2_FORMAT_RGB8, camera.fps_); //do not change the stream format
    cfg.enable_stream(RS2_STREAM_DEPTH, camera.width_, camera.height_, RS2_FORMAT_Z16, camera.fps_);  //do not change the stream format
    rs2::pipeline_profile profile = pipe.start(cfg);
    camera.depth_scale_meters_ = profile.get_device().first<rs2::depth_sensor>().get_depth_scale(); // Get depth scale
    std::cout << "camera is streaming" << std::endl;

    ros::Rate imu_rate(imu.rate_);
    std::thread imu_thread([&]() {
        while (ros::ok())
        {
            imu.update();
            if (token == true)
            {
                imu.publish_data();
                ros::spinOnce();
            }
            imu_rate.sleep();
        }
    });
    imu_thread.detach();
    std::thread camera_thread([&]() {
        while (ros::ok())
        {
            rs2::frameset frames = fq.wait_for_frame();
            ros::Time ros_timestamp = ros::Time::now();
            camera.publishDepthandColor(frames, ros_timestamp);
            camera.publishAlignedDepth(frames, ros_timestamp);
            ros::spinOnce();
        }
    });
    camera_thread.detach();

    usleep(1000);
    ros::Rate camera_rate(30);
    while (ros::ok())
    {
        fq.enqueue(pipe.wait_for_frames()); //if using pulse as timming indicator: if(gpio_pin == HIGH) { do the interpolation }
        token = false;
        auto current_time = ros::Time::now();
        ros::Duration diff_time = current_time - imu.ros_time;
        double factor = diff_time.nsec / (double)10000000;
        tf2::Quaternion quat_last, quat_next;
        tf2::Vector3 accel_last, accel_next, gyro_last, gyro_next, mag_last, mag_next;
        tf2::convert(imu.imu_msg.orientation, quat_last);
        tf2::convert(imu.imu_msg.linear_acceleration, accel_last);
        tf2::convert(imu.imu_msg.angular_velocity, gyro_last);
        // tf2::convert(imu.mag_msg.magnetic_field, mag_last);
        usleep(10000); 
        tf2::convert(imu.imu_msg.orientation, quat_next);
        tf2::convert(imu.imu_msg.linear_acceleration, accel_next);
        tf2::convert(imu.imu_msg.angular_velocity, gyro_next);
        // tf2::convert(imu.mag_msg.magnetic_field, mag_next);
        tf2::Quaternion quat_inter = quat_last.slerp(quat_next, factor);
        tf2::Vector3 accel_inter = accel_last.lerp(accel_next, factor);
        tf2::Vector3 gyro_inter = gyro_last.lerp(gyro_next, factor);
        // tf2::Vector3 mag_inter = mag_last.lerp(mag_next, factor);
        imu.imu_msg.linear_acceleration.x = (double)accel_inter.getX();
        imu.imu_msg.linear_acceleration.y = (double)accel_inter.getY();
        imu.imu_msg.linear_acceleration.z = (double)accel_inter.getZ();
        imu.imu_msg.angular_velocity.x = (double)gyro_inter.getX();
        imu.imu_msg.angular_velocity.y = (double)gyro_inter.getY();
        imu.imu_msg.angular_velocity.z = (double)gyro_inter.getZ();
        imu.imu_msg.orientation.x = (double)quat_inter.getX();
        imu.imu_msg.orientation.y = (double)quat_inter.getY();
        imu.imu_msg.orientation.z = (double)quat_inter.getZ();
        imu.imu_msg.orientation.w = (double)quat_inter.getW();
        imu.imu_msg.header.stamp = current_time;
        // imu.mag_msg.magnetic_field.x = (double)mag_inter.getX();
        // imu.mag_msg.magnetic_field.y = (double)mag_inter.getY();
        // imu.mag_msg.magnetic_field.z = (double)mag_inter.getZ();
        imu.imu_publisher.publish(imu.imu_msg);
        // imu.imu_sequence++;
        // imu.imu_msg.header.stamp = imu.ros_time;
        // imu.imu_msg.header.seq = imu.imu_sequence;
        // imu.imu_msg.linear_acceleration.x = (double)accel_next.getX();
        // imu.imu_msg.linear_acceleration.y = (double)accel_next.getY();
        // imu.imu_msg.linear_acceleration.z = (double)accel_next.getZ();
        // imu.imu_msg.angular_velocity.x = (double)gyro_next.getX();
        // imu.imu_msg.angular_velocity.y = (double)gyro_next.getY();
        // imu.imu_msg.angular_velocity.z = (double)gyro_next.getZ();
        // imu.imu_msg.orientation.x = (double)quat_next.getX();
        // imu.imu_msg.orientation.y = (double)quat_next.getY();
        // imu.imu_msg.orientation.z = (double)quat_next.getZ();
        // imu.imu_msg.orientation.w = (double)quat_next.getW();
        // imu.mag_msg.header.stamp = imu.imu_msg.header.stamp;
        // imu.mag_msg.header.seq = imu.imu_msg.header.seq;
        // imu.mag_msg.magnetic_field.x = (double)mag_next.getX();
        // imu.mag_msg.magnetic_field.y = (double)mag_next.getY();
        // imu.mag_msg.magnetic_field.z = (double)mag_next.getZ();
        // imu.publish_data();
        ros::spinOnce();
        // std::cout << ros::Time::now() - current_time << std::endl;
        token = true;
        camera_rate.sleep();
    }
    imu.closePort();
    pipe.stop();
    std::cout << "exit.." << std::endl;
}