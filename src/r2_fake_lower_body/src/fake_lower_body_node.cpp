#include <rclcpp/rclcpp.hpp>
#include "r2_interfaces/msg/goal_pose.hpp"
#include "r2_interfaces/msg/goal_reached.hpp"

using std::placeholders::_1;

class FakeLowerBodyNode : public rclcpp::Node
{
public:
    FakeLowerBodyNode() : Node("fake_lower_body_node")
    {
        sub_ = create_subscription<r2_interfaces::msg::GoalPose>(
            "/r2/lower_body/goal_pose", 10,
            std::bind(&FakeLowerBodyNode::onGoal, this, _1));

        pub_ = create_publisher<r2_interfaces::msg::GoalReached>(
            "/r2/lower_body/goal_reached", 10);
    }

private:
    void onGoal(const r2_interfaces::msg::GoalPose::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Lower body received goal (%.2f, %.2f)",
                    msg->x, msg->y);

        auto reached = r2_interfaces::msg::GoalReached();
        reached.reached = true;
        pub_->publish(reached);
    }

    rclcpp::Subscription<r2_interfaces::msg::GoalPose>::SharedPtr sub_;
    rclcpp::Publisher<r2_interfaces::msg::GoalReached>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeLowerBodyNode>());
    rclcpp::shutdown();
    return 0;
}
