#include <rclcpp/rclcpp.hpp>
#include "r2_interfaces/msg/goal_pose.hpp"
#include "r2_interfaces/msg/goal_reached.hpp"
#include "r2_interfaces/msg/arm_command.hpp"
#include "r2_interfaces/msg/arm_ack.hpp"
#include "r2_interfaces/msg/arm_done.hpp"

using std::placeholders::_1;

// 枚举机器人状态
enum class State
{
    INIT,
    GO_TO_MF_ENTRY,
    OPERATE_MF_ENTRY,
    GO_TO_MF_EXIT,
    DONE
};

// 决策节点类
class R2DecisionNode : public rclcpp::Node
{
public:
    R2DecisionNode() : Node("r2_decision_node")
    {
        // 发布者
        lower_goal_pub_ = create_publisher<r2_interfaces::msg::GoalPose>(
            "/r2/lower_body/goal_pose", 10);

        // 上臂命令发布者
        upper_cmd_pub_ = create_publisher<r2_interfaces::msg::ArmCommand>(
            "/r2/upper_body/command", 10);

        // 上臂命令确认订阅者
        upper_ack_sub_ = create_subscription<r2_interfaces::msg::ArmAck>(
            "/r2/upper_body/ack", 10,
            std::bind(&R2DecisionNode::onUpperAck, this, _1));

        // 订阅者
        lower_reached_sub_ = create_subscription<r2_interfaces::msg::GoalReached>(
            "/r2/lower_body/goal_reached", 10,
            std::bind(&R2DecisionNode::onLowerReached, this, _1));

        // 上臂完成订阅者
        upper_done_sub_ = create_subscription<r2_interfaces::msg::ArmDone>(
            "/r2/upper_body/done", 10,
            std::bind(&R2DecisionNode::onUpperDone, this, _1));

        // 定时器
        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&R2DecisionNode::tick, this));

        RCLCPP_INFO(get_logger(), "R2 Decision Node Started");
    }

    // 私有成员函数和变量
private:
    void tick()
    {
        // 根据当前状态进行决策
        if (state_ == State::INIT)
        {
            transitionTo(State::GO_TO_MF_ENTRY);
        }

        handleUpperCommandReliability();
    }

    void transitionTo(State next)
    {
        // 状态转换逻辑
        state_ = next;
        RCLCPP_INFO(get_logger(), "Transition to state %d", static_cast<int>(state_));

        if (state_ == State::GO_TO_MF_ENTRY)
        {
            publishLowerGoal(1.8f, 0.0f);
        }
        else if (state_ == State::OPERATE_MF_ENTRY)
        {
            startReliableUpperCommand(r2_interfaces::msg::ArmCommand::PICK_KFS);
        }
        else if (state_ == State::GO_TO_MF_EXIT)
        {
            publishLowerGoal(3.2f, 0.0f);
        }
        else if (state_ == State::DONE)
        {
            RCLCPP_INFO(get_logger(), "Mission finished");
        }
    }

    void startReliableUpperCommand(uint8_t cmd)
    {
        pending_upper_cmd_ = cmd;
        waiting_upper_ack_ = true;

        const auto now_time = now();
        upper_cmd_start_time_ = now_time;
        last_upper_send_time_ = now_time;

        publishUpperCommand(cmd);
        RCLCPP_INFO(get_logger(), "Start reliable upper command: %d", cmd);
    }

    void handleUpperCommandReliability()
    {
        const auto now_time = now();

        if (waiting_upper_ack_)
        {
            if ((now_time - last_upper_send_time_).nanoseconds() >= kUpperCommandResendPeriodMs * 1000000)
            {
                publishUpperCommand(pending_upper_cmd_);
                last_upper_send_time_ = now_time;
            }

            if ((now_time - upper_cmd_start_time_).nanoseconds() >= kUpperCommandTimeoutMs * 1000000)
            {
                RCLCPP_WARN(get_logger(),
                            "Upper command %d ACK timeout, keep resending",
                            pending_upper_cmd_);
                upper_cmd_start_time_ = now_time;
            }
            return;
        }

        if ((now_time - last_idle_heartbeat_time_).nanoseconds() >= kIdleHeartbeatPeriodMs * 1000000)
        {
            publishUpperCommand(r2_interfaces::msg::ArmCommand::IDLE);
            last_idle_heartbeat_time_ = now_time;
        }
    }

    void onUpperAck(const r2_interfaces::msg::ArmAck::SharedPtr msg)
    {
        if (!msg->received || !waiting_upper_ack_)
        {
            return;
        }

        if (msg->command != pending_upper_cmd_)
        {
            RCLCPP_WARN(get_logger(),
                        "Ignore ACK for command %d, waiting command is %d",
                        msg->command,
                        pending_upper_cmd_);
            return;
        }

        waiting_upper_ack_ = false;
        last_idle_heartbeat_time_ = now();
        RCLCPP_INFO(get_logger(), "ACK received for upper command: %d", msg->command);
    }

    void publishLowerGoal(float x, float y)
    {
        r2_interfaces::msg::GoalPose msg;
        msg.x = x;
        msg.y = y;
        lower_goal_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Lower body goal: (%.2f, %.2f)", x, y);
    }

    void publishUpperCommand(uint8_t cmd)
    {
        r2_interfaces::msg::ArmCommand msg;
        msg.command = cmd;
        upper_cmd_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Upper body command: %d", cmd);
    }

    void onLowerReached(const r2_interfaces::msg::GoalReached::SharedPtr msg)
    {
        if (!msg->reached)
            return;

        if (state_ == State::GO_TO_MF_ENTRY)
        {
            transitionTo(State::OPERATE_MF_ENTRY);
        }
        else if (state_ == State::GO_TO_MF_EXIT)
        {
            transitionTo(State::DONE);
        }
    }

    void onUpperDone(const r2_interfaces::msg::ArmDone::SharedPtr msg)
    {
        if (!msg->done)
            return;

        if (state_ == State::OPERATE_MF_ENTRY)
        {
            if (msg->command != r2_interfaces::msg::ArmCommand::PICK_KFS)
            {
                return;
            }

            if (!msg->success)
            {
                RCLCPP_WARN(get_logger(), "Upper PICK_KFS failed, retry command");
                startReliableUpperCommand(r2_interfaces::msg::ArmCommand::PICK_KFS);
                return;
            }

            transitionTo(State::GO_TO_MF_EXIT);
        }
    }

    static constexpr int64_t kUpperCommandResendPeriodMs = 100;
    static constexpr int64_t kIdleHeartbeatPeriodMs = 500;
    static constexpr int64_t kUpperCommandTimeoutMs = 1200;

    State state_{State::INIT};

    rclcpp::Publisher<r2_interfaces::msg::GoalPose>::SharedPtr lower_goal_pub_;
    rclcpp::Publisher<r2_interfaces::msg::ArmCommand>::SharedPtr upper_cmd_pub_;

    rclcpp::Subscription<r2_interfaces::msg::GoalReached>::SharedPtr lower_reached_sub_;
    rclcpp::Subscription<r2_interfaces::msg::ArmAck>::SharedPtr upper_ack_sub_;
    rclcpp::Subscription<r2_interfaces::msg::ArmDone>::SharedPtr upper_done_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool waiting_upper_ack_{false};
    uint8_t pending_upper_cmd_{r2_interfaces::msg::ArmCommand::IDLE};
    rclcpp::Time last_upper_send_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time upper_cmd_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_idle_heartbeat_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<R2DecisionNode>());
    rclcpp::shutdown();
    return 0;
}
