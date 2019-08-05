#pragma once
#include <librealsense2/rs.hpp> // camera lib
#include <librealsense2/rs_advanced_mode.hpp>
#include <ros/ros.h>
#include <map>
#include <assert.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>

/* camera class */
class realsenseD435_mod
{
public:
    int width_;
    int height_;
    int fps_;
    float depth_scale_meters_;
    std::map<rs2_stream, int> image_format_;
    std::map<rs2_stream, std::string> encoding_;
    std::map<rs2_stream, std::string> depth_aligned_encoding_;
    std::map<rs2_stream, std::shared_ptr<rs2::align>> align_;
    std::map<rs2_stream, std::string> optical_frame_id_;
    std::map<rs2_stream, std::string> depth_aligned_frame_id_;
    std::map<rs2_stream, int> seq_;
    std::map<rs2_stream, int> depth_aligned_seq_;
    std::map<rs2_stream, cv::Mat> image_;
    std::map<rs2_stream, cv::Mat> depth_scaled_image_;
    std::map<rs2_stream, cv::Mat> depth_aligned_image_;
    std::map<rs2_stream, image_transport::Publisher> image_publishers_;
    std::map<rs2_stream, image_transport::Publisher> depth_aligned_image_publishers_;
    realsenseD435_mod(ros::NodeHandle &n_image);
    cv::Mat &fix_depth_scale(const cv::Mat &from_image, cv::Mat &to_image);
    void publishFrame(rs2::frame f, const ros::Time &t, const rs2_stream &stream,
                      std::map<rs2_stream, cv::Mat> &images,
                      const std::map<rs2_stream, std::string> &optical_frame_id,
                      const std::map<rs2_stream, std::string> &encoding,
                      std::map<rs2_stream, int> &seq,
                      const std::map<rs2_stream, image_transport::Publisher> &image_publishers,
                      bool copy_data_from_frame = true);
    void publishDepthandColor(rs2::frameset frames, ros::Time &t);
    void publishAlignedDepth(rs2::frameset frames, const ros::Time &t);
    ~realsenseD435_mod();
};