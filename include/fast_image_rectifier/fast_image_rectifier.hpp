/**
 * @file fast_image_rectifier.hpp
 * @author Nguyen Quang <quang@infiniumrobotics.com>
 * @brief The header file of the FastImageRectifier class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Infinium Robotics, all rights reserved.
 * 
 */

#ifndef FAST_EXPOSURE_CONTROLLER_HPP
#define FAST_EXPOSURE_CONTROLLER_HPP

#include <iostream>
#include <string>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class FastImageRectifier
{
private:
    ros::NodeHandle node_; //!< @brief The ros node handle. @since 0.0.1

    std::string image_raw_topic_;   //!< @brief The image raw topic. @since 0.0.1
    ros::Subscriber image_raw_sub_; //!< @brief The image raw subscriber. @since 0.0.1

    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_ptr_; //!< @brief The CameraInfoManager object. @since 0.0.1
    std::string camera_info_url_;                                                     //!< @brief The path to the camera_info file. @since 0.0.1
    sensor_msgs::CameraInfo camera_info_;                                             //!< @brief The camera infomation. @since 0.0.1
    cv::Mat camera_matrix_;                                                           //!< @brief The camera matrix. @since 0.0.1
    cv::Mat dis_coef_;                                                                //!< @brief The camera distortion coefficients. @since 0.0.1
    cv::Mat undist_map_1_, undist_map_2_;                                             //!< @brief The undistortion rectify maps. @since 0.0.1

    image_transport::Publisher image_rect_pub_; //!< @brief The rectified image publisher. @since 0.0.1
    ros::Publisher camera_info_pub_;            //!< @brief The camera info publisher

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

#endif // FAST_EXPOSURE_CONTROLLER_HPP
