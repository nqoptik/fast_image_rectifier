#include "fast_image_rectifier/image_saver.hpp"

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
    ros::init(argc, argv, "image_saver_node");
    ros::NodeHandle node_handle("~");
    ImageSaver image_saver(node_handle);
    ros::spin();
    return 0;
}
