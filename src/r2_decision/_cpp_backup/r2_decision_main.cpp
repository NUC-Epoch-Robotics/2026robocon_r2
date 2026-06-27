#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "r2_decision/r2_decision_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<R2DecisionNode>());
    rclcpp::shutdown();
    return 0;
}
