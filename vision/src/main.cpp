#include <rclcpp/rclcpp.hpp>
#include "video_processor.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("video_processor_node");
    auto video_processor = std::make_shared<VideoProcessor>(node);
    video_processor->spin();
    return 0;
}
