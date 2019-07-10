/**
 * @file fast_image_rectifier.cpp
 * @author Nguyen Quang <quang@infiniumrobotics.com>
 * @brief The definitions of the FastImageRectifier class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Infinium Robotics, all rights reserved.
 * 
 */

#include "fast_image_rectifier/fast_image_rectifier.hpp"

FastImageRectifier::FastImageRectifier(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
    // Initialise the CameraInfoManager
    camera_info_manager_ptr_ = std::make_shared<camera_info_manager::CameraInfoManager>(node_handle_);

    // Initialise the raw image subscriber
    node_handle_.param<std::string>("raw_image_topic", raw_image_topic_, "image_raw");
    raw_image_subscriber_ = node_handle_.subscribe(raw_image_topic_, 1, &FastImageRectifier::image_callback, this);

    // Initialise the camera_info
    node_handle_.param<std::string>("camera_info_url", camera_info_url_, "");
    if (camera_info_manager_ptr_->loadCameraInfo(camera_info_url_))
    {
        camera_info_message_ = camera_info_manager_ptr_->getCameraInfo();
    }
    else
    {
        ROS_ERROR("Invalid camera_info_url!");
    }

    // Initialise camera matrix and distortion coefficients
    camera_matrix_ = (cv::Mat1d(3, 3) << camera_info_message_.K[0], camera_info_message_.K[1], camera_info_message_.K[2],
                      camera_info_message_.K[3], camera_info_message_.K[4], camera_info_message_.K[5],
                      camera_info_message_.K[6], camera_info_message_.K[7], camera_info_message_.K[8]);
    distortion_coeffcients_ = (cv::Mat1d(1, 5) << camera_info_message_.D[0], camera_info_message_.D[1], camera_info_message_.D[2], camera_info_message_.D[3], camera_info_message_.D[4]);

    // Calculate the undistortion rectify maps
    cv::initUndistortRectifyMap(camera_matrix_, distortion_coeffcients_, cv::Mat(), cv::Mat(), cv::Size(camera_info_message_.width, camera_info_message_.height), CV_16SC2, undistortion_map_1_, undistortion_map_2_);

    // Initialise the rectified image publisher
    image_transport::ImageTransport image_transport(node_handle_);
    rectified_image_publisher_ = image_transport.advertise("image_rect", 1);

    // Initialise the camera info publisher
    camera_info_publisher_ = node_handle_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
}

FastImageRectifier::~FastImageRectifier()
{
    ros::shutdown();
}

void FastImageRectifier::image_callback(const sensor_msgs::Image::ConstPtr& image_message_ptr)
{
    cv::Mat image;
    try
    {
        image = cv_bridge::toCvShare(image_message_ptr, "mono8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", image_message_ptr->encoding.c_str());
    }

    // Rectify the image
    cv::Mat image_rect;
    cv::remap(image, image_rect, undistortion_map_1_, undistortion_map_2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    // Publish the rectified image
    sensor_msgs::ImagePtr rectified_image_message_ptr;
    rectified_image_message_ptr = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_rect).toImageMsg();
    rectified_image_message_ptr->header.frame_id = image_message_ptr->header.frame_id;
    rectified_image_message_ptr->header.stamp = image_message_ptr->header.stamp;
    rectified_image_publisher_.publish(rectified_image_message_ptr);

    // Publish the camera info
    camera_info_message_.header.frame_id = image_message_ptr->header.frame_id;
    camera_info_message_.header.stamp = image_message_ptr->header.stamp;
    camera_info_publisher_.publish(camera_info_message_);
}
