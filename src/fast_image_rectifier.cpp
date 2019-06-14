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
}

FastImageRectifier::~FastImageRectifier() {
    ros::shutdown();
}

void FastImageRectifier::image_callback(const sensor_msgs::ImageConstPtr& img_msg) {
}