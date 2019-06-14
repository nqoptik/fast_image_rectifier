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

FastImageRectifier::FastImageRectifier(ros::NodeHandle node) {
    this->node = node;

    // Initialise the CameraInfoManager
    camera_info_manager_ = new camera_info_manager::CameraInfoManager(node);

    // Initialise the image raw subscriber
    node.param<std::string>("image_raw_topic", image_raw_topic, "image_raw");
    image_raw_sub = node.subscribe(image_raw_topic, 1, &FastImageRectifier::image_callback, this);

    // Initialise the camera_info
    node.param<std::string>("camera_info_url", camera_info_url, "");
    if (camera_info_manager_->loadCameraInfo(camera_info_url)) {
        camera_info = camera_info_manager_->getCameraInfo();
    } else {
        ROS_ERROR("Invalid camera_info_url!");
    }

    // Initialize camera matrix and distortion coefficients
    camera_matrix = (cv::Mat1d(3, 3) << camera_info.K[0], camera_info.K[1], camera_info.K[2],
                     camera_info.K[3], camera_info.K[4], camera_info.K[5],
                     camera_info.K[6], camera_info.K[7], camera_info.K[8]);
    dis_coef = (cv::Mat1d(1, 5) << camera_info.D[0], camera_info.D[1], camera_info.D[2], camera_info.D[3], camera_info.D[4]);

    // Calculate the undistortion rectify maps
    cv::initUndistortRectifyMap(camera_matrix, dis_coef, cv::Mat(), cv::Mat(), cv::Size(camera_info.width, camera_info.height), CV_16SC2, undist_map_1, undist_map_2);

    // Initialize the rectified image publisher
    image_transport::ImageTransport it(node);
    image_rect_pub = it.advertise("image_rect", 1);

    // Initialise the camera info publisher
    camera_info_pub = node.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
}

FastImageRectifier::~FastImageRectifier() {
    delete camera_info_manager_;
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
    cv::remap(image, image_rect, undist_map_1, undist_map_2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    // Publish the rectified image
    sensor_msgs::ImagePtr image_rect_msg;
    image_rect_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_rect).toImageMsg();
    image_rect_msg->header.frame_id = img_msg->header.frame_id;
    image_rect_msg->header.stamp = img_msg->header.stamp;
    image_rect_pub.publish(image_rect_msg);

    // Publish the camera info
    camera_info.header.frame_id = img_msg->header.frame_id;
    camera_info.header.stamp = img_msg->header.stamp;
    camera_info_pub.publish(camera_info);
}
