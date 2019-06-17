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

FastImageRectifier::FastImageRectifier(ros::NodeHandle node)
    : node_(node) {
    // Initialise the CameraInfoManager
    camera_info_manager_ptr_ = std::make_shared<camera_info_manager::CameraInfoManager>(node_);

    // Initialise the image raw subscriber
    node_.param<std::string>("image_raw_topic", image_raw_topic_, "image_raw");
    image_raw_sub_ = node_.subscribe(image_raw_topic_, 1, &FastImageRectifier::image_callback, this);

    // Initialise the camera_info
    node_.param<std::string>("camera_info_url", camera_info_url_, "");
    if (camera_info_manager_ptr_->loadCameraInfo(camera_info_url_)) {
        camera_info_ = camera_info_manager_ptr_->getCameraInfo();
    } else {
        ROS_ERROR("Invalid camera_info_url!");
    }

    // Initialise camera matrix and distortion coefficients
    camera_matrix_ = (cv::Mat1d(3, 3) << camera_info_.K[0], camera_info_.K[1], camera_info_.K[2],
                      camera_info_.K[3], camera_info_.K[4], camera_info_.K[5],
                      camera_info_.K[6], camera_info_.K[7], camera_info_.K[8]);
    dis_coef_ = (cv::Mat1d(1, 5) << camera_info_.D[0], camera_info_.D[1], camera_info_.D[2], camera_info_.D[3], camera_info_.D[4]);

    // Calculate the undistortion rectify maps
    cv::initUndistortRectifyMap(camera_matrix_, dis_coef_, cv::Mat(), cv::Mat(), cv::Size(camera_info_.width, camera_info_.height), CV_16SC2, undist_map_1_, undist_map_2_);

    // Initialise the rectified image publisher
    image_transport::ImageTransport it(node_);
    image_rect_pub_ = it.advertise("image_rect", 1);

    // Initialise the camera info publisher
    camera_info_pub_ = node_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
}

FastImageRectifier::~FastImageRectifier() {
    ros::shutdown();
}

void FastImageRectifier::image_callback(const sensor_msgs::ImageConstPtr& img_msg) {
    cv::Mat image;
    try {
        image = cv_bridge::toCvShare(img_msg, "mono8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", img_msg->encoding.c_str());
    }

    // Rectify the image
    cv::Mat image_rect;
    cv::remap(image, image_rect, undist_map_1_, undist_map_2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    // Publish the rectified image
    sensor_msgs::ImagePtr image_rect_msg;
    image_rect_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_rect).toImageMsg();
    image_rect_msg->header.frame_id = img_msg->header.frame_id;
    image_rect_msg->header.stamp = img_msg->header.stamp;
    image_rect_pub_.publish(image_rect_msg);

    // Publish the camera info
    camera_info_.header.frame_id = img_msg->header.frame_id;
    camera_info_.header.stamp = img_msg->header.stamp;
    camera_info_pub_.publish(camera_info_);
}
