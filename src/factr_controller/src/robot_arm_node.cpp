#include "factr_controller/robot_arm.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotArm>();
    node->run();

    rclcpp::shutdown();
    return 0;
}
