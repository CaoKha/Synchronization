#include <d435_ros/d435_ros.hpp>

/*--------------------main--------------------*/
int main(int argc, char **argv)
{
    /* set up camera */
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle image_node_handler;
    realsenseD435 camera(image_node_handler);
    rs2::config cfg;
    rs2::pipeline pipe;
    cfg.enable_stream(RS2_STREAM_COLOR, camera.width_, camera.height_, RS2_FORMAT_RGB8, camera.fps_); //do not change the stream format
    cfg.enable_stream(RS2_STREAM_DEPTH, camera.width_, camera.height_, RS2_FORMAT_Z16, camera.fps_);  //do not change the stream format

    /* start streaming the camera */
    rs2::pipeline_profile profile = pipe.start(cfg);
    camera.depth_scale_meters_ = profile.get_device().first<rs2::depth_sensor>().get_depth_scale(); // Get depth scale
    std::cout << "camera is streaming" << std::endl;

    /* data acquisition */
    ros::Rate loop_rate(camera.fps_);
    while (ros::ok())
    {
        rs2::frameset frames = pipe.wait_for_frames();
        ros::Time ros_timestamp = ros::Time::now();
        camera.publishDepthandColor(frames, ros_timestamp);
        camera.publishAlignedDepth(frames, ros_timestamp);
        ros::spinOnce();
        loop_rate.sleep();
    }

    /* closing node */
    pipe.stop();
    std::cout << "\nexit success!" << std::endl;
}