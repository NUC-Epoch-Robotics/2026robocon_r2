#include <rclcpp/rclcpp.hpp>
#include "robot_serial/msg/juece.hpp"

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

        ack_pub_ = create_publisher<robot_serial::msg::Juece>(
            "/juece_ack",
            10);

        done_pub_ = create_publisher<robot_serial::msg::Juece>(
            "/juece_done",
            10);

        RCLCPP_INFO(this->get_logger(), "Fake upper body node started as juece");
    }

private:
    void publishAck(uint8_t command)
    {
        robot_serial::msg::Juece ack_msg;
        ack_msg.zhuangtai = 1;   // received
        ack_msg.status_bit = command;
        ack_msg.is_finsh = 0;
        ack_pub_->publish(ack_msg);
    }

    void finishCommand(uint8_t command)
    {
        robot_serial::msg::Juece done_msg;
        done_msg.zhuangtai = 1;    // done
        done_msg.status_bit = command;
        done_msg.is_finsh = 1;     // success

        switch (command)
        {
        case 1:
            // 1 可能是上台阶(来自status_bit)或抓矛头(来自zhuangtai)
            // 两者的 DONE 都回 status_bit=1
            RCLCPP_INFO(this->get_logger(), "Fake DONE: cmd=%d (stairs/spearhead)", command);
            break;

        case 2:  // DOWN_STAIRS
            RCLCPP_INFO(this->get_logger(), "Fake DOWN_STAIRS");
            break;

        default:
            break;
        }

        done_pub_->publish(done_msg);
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
        // 抓矛头指令通过 zhuangtai 字段, 其他指令通过 status_bit
        bool is_spearhead = (msg->zhuangtai != 0);
        uint8_t cmd_val = is_spearhead ? msg->zhuangtai : msg->status_bit;

        publishAck(cmd_val);

        if (command_in_progress_)
        {
            if (cmd_val == active_command_)
            {
                ++dup_count_;
                return;
            }
            if (active_command_ != 0 && cmd_val != 0)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "New command %d received while command %d still running, ignored",
                    cmd_val,
                    active_command_);
                return;
            }
            if (active_command_ != 0)
            {
                RCLCPP_INFO(this->get_logger(),
                    "Preempting cmd=%d -> cmd=%d", active_command_, cmd_val);
            }
            if (finish_timer_)
            {
                finish_timer_->cancel();
                finish_timer_.reset();
            }
            command_in_progress_ = false;
        }

        if (dup_count_ > 0)
        {
            RCLCPP_INFO(this->get_logger(), "  (duplicate x%d)", dup_count_);
            dup_count_ = 0;
        }

        if (cmd_val != last_logged_cmd_)
        {
            if (is_spearhead)
                RCLCPP_INFO(this->get_logger(), "Received spearhead command: zhuangtai=%d", msg->zhuangtai);
            else
                RCLCPP_INFO(this->get_logger(), "Received arm command: %d (is_finsh=%d)",
                            msg->status_bit, msg->is_finsh);
            last_logged_cmd_ = cmd_val;
        }

        command_in_progress_ = true;
        active_command_ = cmd_val;

        finish_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this, cmd = cmd_val]()
            {
                finishCommand(cmd);
            });
    }

    rclcpp::Subscription<robot_serial::msg::Juece>::SharedPtr sub_;
    rclcpp::Publisher<robot_serial::msg::Juece>::SharedPtr ack_pub_;
    rclcpp::Publisher<robot_serial::msg::Juece>::SharedPtr done_pub_;
    rclcpp::TimerBase::SharedPtr finish_timer_;

    bool command_in_progress_{false};
    uint8_t active_command_{0};
    uint8_t last_logged_cmd_{0xFF};
    int dup_count_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeUpperBodyNode>());
    rclcpp::shutdown();
    return 0;
}
