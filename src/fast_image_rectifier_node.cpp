/**
 * @file fast_image_rectifier_node.cpp
 * @author Nguyen Quang <quang@infiniumrobotics.com>
 * @brief The fast image rectifier node.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Infinium Robotics, all rights reserved.
 * 
 */

#include "fast_image_rectifier/fast_image_rectifier.hpp"

/**
 * @brief The main function.
 * 
 * @param[in] argc The argument count.
 * @param[in] argv The argument vector.
 * @return The status value.
 * @since 0.0.1
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fast_image_rectifier_node");
    ros::NodeHandle node_handle("~");
    FastImageRectifier fast_image_rectifier(node_handle);
    ros::spin();
    return 0;
}
