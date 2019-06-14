/**
 * @file fast_image_rectifier.cpp
 * @author Nguyen Quang <quang@infiniumrobotics.com>
 * @brief The definitions of the FastImageRectifier class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Infinium Robotics, all rights reserved.
 * 
 */

#include "fast_image_rectifier/fast_image_rectifier.h"

FastImageRectifier::FastImageRectifier(ros::NodeHandle node) {
    this->node = node;

    // Initialise the CameraInfoManager
    camera_info_manager_ = new camera_info_manager::CameraInfoManager(node);

    // Initialise the camera_info
    node.param<std::string>("camera_info_url", camera_info_url, "");
    if (camera_info_manager_->loadCameraInfo(camera_info_url)) {
        camera_info = camera_info_manager_->getCameraInfo();
    } else {
        ROS_ERROR("Invalid camera_info_url!");
    }
}

FastImageRectifier::~FastImageRectifier() {
    delete camera_info_manager_;
    ros::shutdown();
}

void FastImageRectifier::image_callback(const sensor_msgs::ImageConstPtr& img_msg) {
}
