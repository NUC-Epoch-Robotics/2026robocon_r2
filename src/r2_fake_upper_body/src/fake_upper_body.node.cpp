#include <rclcpp/rclcpp.hpp>
#include "robot_serial/msg/juece.hpp"
#include "r2_interfaces/msg/arm_ack.hpp"
#include "r2_interfaces/msg/arm_done.hpp"

using std::placeholders::_1;

class FakeUpperBodyNode : public rclcpp::Node
{
public:
    FakeUpperBodyNode() : Node("juece")
    {
        sub_ = create_subscription<robot_serial::msg::Juece>(
            "/juece",
            10,
            std::bind(&FakeUpperBodyNode::onCommand, this, _1));

        ack_pub_ = create_publisher<r2_interfaces::msg::ArmAck>(
            "/juece_ack",
            10);

        pub_ = create_publisher<r2_interfaces::msg::ArmDone>(
            "/juece_done",
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
        case 1:  // UP_STAIRS
            RCLCPP_INFO(this->get_logger(), "Fake UP_STAIRS");
            break;

        case 2:  // DOWN_STAIRS
            RCLCPP_INFO(this->get_logger(), "Fake DOWN_STAIRS");
            break;

        default:
            break;
        }

        pub_->publish(done_msg);
        command_in_progress_ = false;
        active_command_ = 0;

        if (finish_timer_)
        {
            finish_timer_->cancel();
            finish_timer_.reset();
        }
    }

    void onCommand(const robot_serial::msg::Juece::SharedPtr msg)
    {
        publishAck(msg->status_bit);

        if (command_in_progress_)
        {
            if (msg->status_bit == active_command_)
            {
                // 相同命令: 只计数, 不打日志
                ++dup_count_;
                return;
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "New command %d received while command %d still running, ignored",
                    msg->status_bit,
                    active_command_);
            }
            return;
        }

        if (dup_count_ > 0)
        {
            RCLCPP_INFO(this->get_logger(), "  (duplicate x%d)", dup_count_);
            dup_count_ = 0;
        }

        if (msg->status_bit != last_logged_cmd_)
        {
            RCLCPP_INFO(this->get_logger(), "Received arm command: %d (is_finsh=%d)",
                        msg->status_bit, msg->is_finsh);
            last_logged_cmd_ = msg->status_bit;
        }

        command_in_progress_ = true;
        active_command_ = msg->status_bit;

        finish_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this, cmd = msg->status_bit]()
            {
                finishCommand(cmd);
            });
    }

    rclcpp::Subscription<robot_serial::msg::Juece>::SharedPtr sub_;
    rclcpp::Publisher<r2_interfaces::msg::ArmAck>::SharedPtr ack_pub_;
    rclcpp::Publisher<r2_interfaces::msg::ArmDone>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr finish_timer_;

    bool command_in_progress_{false};
    uint8_t active_command_{0};
    uint8_t last_logged_cmd_{0xFF};  // 上次打印的命令, 0xFF 确保首次必打
    int dup_count_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeUpperBodyNode>());
    rclcpp::shutdown();
    return 0;
}
