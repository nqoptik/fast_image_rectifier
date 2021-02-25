#ifndef IMAGE_SAVER_HPP
#define IMAGE_SAVER_HPP

#include <iostream>
#include <string>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ImageSaver
{
private:
    ros::NodeHandle node_handle_; //!< @brief The ros node handle. @since 0.0.1

    std::string raw_image_topic_;          //!< @brief The raw image topic. @since 0.0.1
    ros::Subscriber raw_image_subscriber_; //!< @brief The raw image subscriber. @since 0.0.1

    image_transport::Publisher rectified_image_publisher_; //!< @brief The rectified image publisher. @since 0.0.1

    int frame_count_;

public:
    /**
     * @brief Construct a new ImageSaver object.
     * 
     * @param[in] node_handle The ros node handle.
     * @since 0.0.1
     */
    ImageSaver(const ros::NodeHandle& node_handle);

    /**
     * @brief Destroy the ImageSaver object.
     * 
     * @since 0.0.1
     * 
     */
    ~ImageSaver();

    /**
     * @brief The image callback function.
     * 
     * @param[in] image_message_ptr The image message.
     * @since 0.0.1
     */
    void image_callback(const sensor_msgs::Image::ConstPtr& image_message_ptr);
};

#endif // IMAGE_SAVER_HPP
