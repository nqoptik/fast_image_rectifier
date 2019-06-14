/**
 * @file fast_image_rectifier.h
 * @author Nguyen Quang <quang@infiniumrobotics.com>
 * @brief The header file of the FastImageRectifier class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Infinium Robotics, all rights reserved.
 * 
 */

#ifndef _FAST_EXPOSURE_CONTROLLER_H_
#define _FAST_EXPOSURE_CONTROLLER_H_

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class FastImageRectifier {
   private:
    ros::NodeHandle node;  //!< @brief The ros node handle. @since 0.0.1

    camera_info_manager::CameraInfoManager* camera_info_manager_;  //!< @brief The CameraInfoManager object. @since 0.0.1
    std::string camera_info_url;                                   //!< @brief The path to the camera_info file. @since 0.0.1
    sensor_msgs::CameraInfo camera_info;                           //!< @brief The camera infomation. @since 0.0.1

   public:
    /**
     * @brief Construct a new FastImageRectifier object.
     * 
     * @param[in] node The ros node handle.
     * @since 0.0.1
     */
    FastImageRectifier(ros::NodeHandle node);

    /**
     * @brief Destroy the FastImageRectifier object.
     * 
     * @since 0.0.1
     * 
     */
    ~FastImageRectifier();

    /**
     * @brief The image callback function.
     * 
     * @param[in] img_msg The image message.
     * @since 0.0.1
     */
    void image_callback(const sensor_msgs::ImageConstPtr& img_msg);
};

#endif /* _FAST_EXPOSURE_CONTROLLER_H_ */
