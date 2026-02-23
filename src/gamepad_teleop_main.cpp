#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "studica_control/gamepad_component.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<studica_control::GamepadController>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
