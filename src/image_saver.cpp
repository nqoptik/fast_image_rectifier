#include "fast_image_rectifier/image_saver.hpp"

ImageSaver::ImageSaver(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
    // Initialise the raw image subscriber
    node_handle_.param<std::string>("raw_image_topic", raw_image_topic_, "/image_raw");
    raw_image_subscriber_ = node_handle_.subscribe(raw_image_topic_, 1, &ImageSaver::image_callback, this);

    // Initialise the rectified image publisher
    image_transport::ImageTransport image_transport(node_handle_);
    rectified_image_publisher_ = image_transport.advertise("/image_rect", 1);
    frame_count_ = 0;
}

ImageSaver::~ImageSaver()
{
    ros::shutdown();
}

void ImageSaver::image_callback(const sensor_msgs::Image::ConstPtr& image_message_ptr)
{
    ++frame_count_;
    cv::Mat image;
    try
    {
        image = cv_bridge::toCvShare(image_message_ptr, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_message_ptr->encoding.c_str());
    }
    std::stringstream ss;
    if (frame_count_ < 10)
    {
        ss << "0000";
    }
    else if (frame_count_ < 100)
    {
        ss << "000";
    }
    else if (frame_count_ < 1000)
    {
        ss << "00";
    }
    else if (frame_count_ < 10000)
    {
        ss << "0";
    }
    ss << frame_count_ << ".jpg";
    std::cout << ss.str() << std::endl;
    cv::imwrite(ss.str(), image);
    // Publish the rectified image
    sensor_msgs::ImagePtr rectified_image_message_ptr;
    rectified_image_message_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    rectified_image_message_ptr->header.frame_id = image_message_ptr->header.frame_id;
    rectified_image_message_ptr->header.stamp = image_message_ptr->header.stamp;
    rectified_image_publisher_.publish(rectified_image_message_ptr);
}
