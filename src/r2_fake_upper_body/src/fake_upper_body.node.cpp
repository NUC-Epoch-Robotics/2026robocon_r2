#include <rclcpp/rclcpp.hpp>
#include "r2_interfaces/msg/arm_command.hpp"
#include "r2_interfaces/msg/arm_done.hpp"

using std::placeholders::_1;

class FakeUpperBodyNode : public rclcpp::Node
{
public:
    FakeUpperBodyNode() : Node("fake_upper_body_node")
    {
        sub_ = create_subscription<r2_interfaces::msg::ArmCommand>(
            "/r2/upper_body/command", 10,
            std::bind(&FakeUpperBodyNode::onCommand, this, _1));

        pub_ = create_publisher<r2_interfaces::msg::ArmDone>(
            "/r2/upper_body/done", 10);
    }

private:
    void onCommand(const r2_interfaces::msg::ArmCommand::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Upper body command received: %d", msg->command);

        auto done = r2_interfaces::msg::ArmDone();
        done.done = true;
        pub_->publish(done);
    }

    rclcpp::Subscription<r2_interfaces::msg::ArmCommand>::SharedPtr sub_;
    rclcpp::Publisher<r2_interfaces::msg::ArmDone>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeUpperBodyNode>());
    rclcpp::shutdown();
    return 0;
}
