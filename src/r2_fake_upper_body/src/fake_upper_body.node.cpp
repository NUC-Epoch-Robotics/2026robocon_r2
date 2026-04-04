#include <rclcpp/rclcpp.hpp>
#include "r2_interfaces/msg/arm_command.hpp"
#include "r2_interfaces/msg/arm_ack.hpp"
#include "r2_interfaces/msg/arm_done.hpp"

using std::placeholders::_1;

class FakeUpperBodyNode : public rclcpp::Node
{
public:
    FakeUpperBodyNode() : Node("juece")
    {
        sub_ = create_subscription<r2_interfaces::msg::ArmCommand>(
            "/r2/upper_body/command",
            10,
            std::bind(&FakeUpperBodyNode::onCommand, this, _1));

        ack_pub_ = create_publisher<r2_interfaces::msg::ArmAck>(
            "/r2/upper_body/ack",
            10);

        pub_ = create_publisher<r2_interfaces::msg::ArmDone>(
            "/r2/upper_body/done",
            10);

        RCLCPP_INFO(this->get_logger(), "Fake upper body node started as juece");
    }

private:
    void publishAck(uint8_t command)
    {
        r2_interfaces::msg::ArmAck ack_msg;
        ack_msg.command = command;
        ack_msg.received = true;
        ack_pub_->publish(ack_msg);
    }

    void finishCommand(uint8_t command)
    {
        r2_interfaces::msg::ArmDone done_msg;
        done_msg.command = command;
        done_msg.done = true;
        done_msg.success = true;

        switch (command)
        {
        case r2_interfaces::msg::ArmCommand::SUCTION_EXTEND:
            RCLCPP_INFO(this->get_logger(), "Fake suction success");
            break;

        case r2_interfaces::msg::ArmCommand::GRIPPER_GRAB:
            RCLCPP_INFO(this->get_logger(), "Fake gripper success");
            break;

        default:
            break;
        }

        pub_->publish(done_msg);
        command_in_progress_ = false;
        active_command_ = r2_interfaces::msg::ArmCommand::IDLE;

        if (finish_timer_)
        {
            finish_timer_->cancel();
            finish_timer_.reset();
        }
    }

    void onCommand(const r2_interfaces::msg::ArmCommand::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received arm command: %d",
            msg->command);

        publishAck(msg->command);

        if (command_in_progress_)
        {
            if (msg->command == active_command_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Duplicate command while running, ACK only: %d",
                    msg->command);
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "New command %d received while command %d still running, ignored",
                    msg->command,
                    active_command_);
            }
            return;
        }

        command_in_progress_ = true;
        active_command_ = msg->command;

        finish_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this, cmd = msg->command]()
            {
                finishCommand(cmd);
            });
    }

    rclcpp::Subscription<r2_interfaces::msg::ArmCommand>::SharedPtr sub_;
    rclcpp::Publisher<r2_interfaces::msg::ArmAck>::SharedPtr ack_pub_;
    rclcpp::Publisher<r2_interfaces::msg::ArmDone>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr finish_timer_;

    bool command_in_progress_{false};
    uint8_t active_command_{r2_interfaces::msg::ArmCommand::IDLE};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeUpperBodyNode>());
    rclcpp::shutdown();
    return 0;
}
