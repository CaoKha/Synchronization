#include "d435_ros_mod/d435_ros_mod.hpp"

/* initiation all the parameters for the camera */
realsenseD435_mod::realsenseD435_mod(ros::NodeHandle &n_image)
{
    width_ = 640;
    height_ = 480;
    fps_ = 30;
    depth_scale_meters_ = 0.001; //defaut
    image_transport::ImageTransport it(n_image);
    image_format_[RS2_STREAM_DEPTH] = CV_16UC1;                             // CVBridge type
    encoding_[RS2_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
    optical_frame_id_[RS2_STREAM_DEPTH] = "camera_depth_frame";
    image_publishers_[RS2_STREAM_DEPTH] = it.advertise("camera/depth/image_raw", 1);
    depth_aligned_encoding_[RS2_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;
    image_[RS2_STREAM_DEPTH] = cv::Mat(height_, width_, image_format_[RS2_STREAM_DEPTH], cv::Scalar(0, 0, 0));
    // Types for color stream
    image_format_[RS2_STREAM_COLOR] = CV_8UC3;                        // CVBridge type
    encoding_[RS2_STREAM_COLOR] = sensor_msgs::image_encodings::RGB8; // ROS message type
    optical_frame_id_[RS2_STREAM_COLOR] = "camera_color_frame";
    image_publishers_[RS2_STREAM_COLOR] = it.advertise("camera/color/image_raw", 1);
    depth_aligned_image_publishers_[RS2_STREAM_COLOR] = it.advertise("camera/aligned_depth_to_color/image_raw", 1);
    depth_aligned_encoding_[RS2_STREAM_COLOR] = sensor_msgs::image_encodings::TYPE_16UC1;
    image_[RS2_STREAM_COLOR] = cv::Mat(height_, width_, image_format_[RS2_STREAM_COLOR], cv::Scalar(0, 0, 0));
    depth_aligned_image_[RS2_STREAM_COLOR] = cv::Mat(height_, width_, image_format_[RS2_STREAM_DEPTH], cv::Scalar(0, 0, 0));
    depth_scaled_image_[RS2_STREAM_COLOR] = cv::Mat(height_, width_, image_format_[RS2_STREAM_DEPTH], cv::Scalar(0, 0, 0));
}

cv::Mat &realsenseD435_mod::fix_depth_scale(const cv::Mat &from_image, cv::Mat &to_image)
{
    static const auto meter_to_mm = 0.001f;
    if (abs(depth_scale_meters_ - meter_to_mm) < 1e-6)
    {
        to_image = from_image;
        return to_image;
    }

    if (to_image.size() != from_image.size())
    {
        to_image.create(from_image.rows, from_image.cols, from_image.type());
    }
    CV_Assert(from_image.depth() == image_format_[RS2_STREAM_DEPTH]);

    int nRows = from_image.rows;
    int nCols = from_image.cols;

    if (from_image.isContinuous())
    {
        nCols *= nRows;
        nRows = 1;
    }

    int i, j;
    const uint16_t *p_from;
    uint16_t *p_to;
    for (i = 0; i < nRows; ++i)
    {
        p_from = from_image.ptr<uint16_t>(i);
        p_to = to_image.ptr<uint16_t>(i);
        for (j = 0; j < nCols; ++j)
        {
            p_to[j] = p_from[j] * depth_scale_meters_ / meter_to_mm;
        }
    }
    return to_image;
}

void realsenseD435_mod::publishFrame(rs2::frame f, const ros::Time &t, const rs2_stream &stream,
                                 std::map<rs2_stream, cv::Mat> &images,
                                 const std::map<rs2_stream, std::string> &optical_frame_id,
                                 const std::map<rs2_stream, std::string> &encoding,
                                 std::map<rs2_stream, int> &seq,
                                 const std::map<rs2_stream, image_transport::Publisher> &image_publishers,
                                 bool copy_data_from_frame)
{
    unsigned int width = 0;
    unsigned int height = 0;
    auto bpp = 1;
    if (f.is<rs2::video_frame>())
    {
        auto image = f.as<rs2::video_frame>();
        width = image.get_width();
        height = image.get_height();
        bpp = image.get_bytes_per_pixel();
    }
    auto &image = images[stream];
    if (copy_data_from_frame)
    {
        if (images[stream].size() != cv::Size(width, height))
        {
            image.create(height, width, image.type());
        }
        image.data = (uint8_t *)f.get_data();
    }
    if (f.is<rs2::depth_frame>())
    {
        image = fix_depth_scale(image, depth_scaled_image_[stream]);
    }
    ++(seq[stream]);
    auto &image_publisher = image_publishers.at(stream);
    sensor_msgs::ImagePtr img;
    img = cv_bridge::CvImage(std_msgs::Header(), encoding.at(stream), image).toImageMsg();
    img->width = width;
    img->height = height;
    img->is_bigendian = false;
    img->step = width * bpp;
    img->header.frame_id = optical_frame_id.at(stream);
    img->header.stamp = t;
    img->header.seq = seq[stream];
    image_publisher.publish(img);
}

void realsenseD435_mod::publishDepthandColor(rs2::frameset frames, ros::Time &t)
{
    for (auto it = frames.begin(); it != frames.end(); ++it)
    {
        auto frame = (*it);
        auto stream_type = frame.get_profile().stream_type();
        publishFrame(frame, t, stream_type,
                     image_,
                     optical_frame_id_,
                     encoding_,
                     seq_,
                     image_publishers_);
    }
}

void realsenseD435_mod::publishAlignedDepth(rs2::frameset frames, const ros::Time &t)
{
    for (auto it = frames.begin(); it != frames.end(); ++it)
    {
        auto frame = (*it);
        auto stream_type = frame.get_profile().stream_type();

        if (RS2_STREAM_DEPTH == stream_type)
            continue;
        auto &image_publisher = depth_aligned_image_publishers_[RS2_STREAM_COLOR];
        std::shared_ptr<rs2::align> align;
        try
        {
            align = align_.at(stream_type);
        }
        catch (const std::out_of_range &e)
        {
            ROS_DEBUG_STREAM("Allocate align filter for:" << rs2_stream_to_string(RS2_STREAM_COLOR));
            align = (align_[stream_type] = std::make_shared<rs2::align>(stream_type));
        }
        rs2::frameset processed = frames.apply_filter(*align);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        publishFrame(aligned_depth_frame, t, RS2_STREAM_COLOR,
                     depth_aligned_image_,
                     optical_frame_id_,
                     depth_aligned_encoding_,
                     depth_aligned_seq_,
                     depth_aligned_image_publishers_);
    }
}

realsenseD435_mod::~realsenseD435_mod() {}
