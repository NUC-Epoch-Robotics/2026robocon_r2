#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"

#include "r2_interfaces/msg/arm_ack.hpp"
#include "r2_interfaces/msg/arm_command.hpp"
#include "r2_interfaces/msg/arm_done.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using Spin = nav2_msgs::action::Spin;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

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
        // 上臂命令发布者
        upper_cmd_pub_ = create_publisher<r2_interfaces::msg::ArmCommand>(
            "/r2/upper_body/command", 10);

        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose");

        spin_client_ = rclcpp_action::create_client<Spin>(
            this,
            "spin");

        // 上臂命令确认订阅者
        upper_ack_sub_ = create_subscription<r2_interfaces::msg::ArmAck>(
            "/r2/upper_body/ack", 10,
            std::bind(&R2DecisionNode::onUpperAck, this, _1));

        // 上臂完成订阅者
        upper_done_sub_ = create_subscription<r2_interfaces::msg::ArmDone>(
            "/r2/upper_body/done", 10,
            std::bind(&R2DecisionNode::onUpperDone, this, _1));

        // 定时器
        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&R2DecisionNode::tick, this));

        nav_frame_id_ = this->declare_parameter<std::string>("nav_frame_id", "map");
        mf_entry_spin_rad_ = this->declare_parameter<double>("mf_entry_spin_rad", 0.0);
        mf_exit_spin_rad_ = this->declare_parameter<double>("mf_exit_spin_rad", 0.0);

        RCLCPP_INFO(get_logger(), "R2 Decision Node Started");
    }

    // 私有成员函数和变量
private:
    using CompletionCb = std::function<void(bool)>;

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
            sendNavigateAndSpin(
                1.8,
                0.0,
                mf_entry_spin_rad_,
                [this](bool success)
                {
                    if (!success)
                    {
                        RCLCPP_WARN(get_logger(), "GO_TO_MF_ENTRY failed");
                        return;
                    }

                    if (state_ == State::GO_TO_MF_ENTRY)
                    {
                        transitionTo(State::OPERATE_MF_ENTRY);
                    }
                });
        }
        else if (state_ == State::OPERATE_MF_ENTRY)
        {
            startReliableUpperCommand(r2_interfaces::msg::ArmCommand::PICK_KFS);
        }
        else if (state_ == State::GO_TO_MF_EXIT)
        {
            sendNavigateAndSpin(
                3.2,
                0.0,
                mf_exit_spin_rad_,
                [this](bool success)
                {
                    if (!success)
                    {
                        RCLCPP_WARN(get_logger(), "GO_TO_MF_EXIT failed");
                        return;
                    }

                    if (state_ == State::GO_TO_MF_EXIT)
                    {
                        transitionTo(State::DONE);
                    }
                });
        }
        else if (state_ == State::DONE)
        {
            RCLCPP_INFO(get_logger(), "Mission finished");
        }
    }

    template <typename ActionT>
    bool waitActionServer(
        const typename rclcpp_action::Client<ActionT>::SharedPtr &client,
        const std::string &action_name)
    {
        if (client->wait_for_action_server(1s))
        {
            return true;
        }

        RCLCPP_WARN(get_logger(), "Action server not available: %s", action_name.c_str());
        return false;
    }

    void sendNavigateAndSpin(
        double x,
        double y,
        double spin_rad,
        const CompletionCb &on_done)
    {
        sendNavigateGoal(
            x,
            y,
            [this, spin_rad, on_done](bool nav_ok)
            {
                if (!nav_ok)
                {
                    on_done(false);
                    return;
                }

                sendSpinGoal(spin_rad, on_done);
            });
    }

    void sendNavigateGoal(
        double x,
        double y,
        const CompletionCb &on_done)
    {
        if (!waitActionServer<NavigateToPose>(nav_to_pose_client_, "navigate_to_pose"))
        {
            on_done(false);
            return;
        }

        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = nav_frame_id_;
        goal.pose.header.stamp = this->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.position.z = 0.0;
        goal.pose.pose.orientation.x = 0.0;
        goal.pose.pose.orientation.y = 0.0;
        goal.pose.pose.orientation.z = 0.0;
        goal.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(get_logger(), "Send NavigateToPose goal: x=%.2f y=%.2f", x, y);

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.goal_response_callback = [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_WARN(get_logger(), "NavigateToPose goal rejected");
            }
            else
            {
                RCLCPP_INFO(get_logger(), "NavigateToPose goal accepted");
            }
        };

        options.result_callback = [this, on_done](const GoalHandleNavigateToPose::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(get_logger(), "NavigateToPose finished successfully");
                on_done(true);
                return;
            }

            RCLCPP_WARN(get_logger(), "NavigateToPose finished with code: %d", static_cast<int>(result.code));
            on_done(false);
        };

        nav_to_pose_client_->async_send_goal(goal, options);
    }

    void sendSpinGoal(
        double spin_rad,
        const CompletionCb &on_done)
    {
        if (std::fabs(spin_rad) < 1e-4)
        {
            on_done(true);
            return;
        }

        if (!waitActionServer<Spin>(spin_client_, "spin"))
        {
            on_done(false);
            return;
        }

        Spin::Goal goal;
        goal.target_yaw = spin_rad;

        RCLCPP_INFO(get_logger(), "Send Spin goal: yaw=%.3f rad", spin_rad);

        auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
        options.goal_response_callback = [this](std::shared_ptr<GoalHandleSpin> goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_WARN(get_logger(), "Spin goal rejected");
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Spin goal accepted");
            }
        };

        options.result_callback = [this, on_done](const GoalHandleSpin::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(get_logger(), "Spin finished successfully");
                on_done(true);
                return;
            }

            RCLCPP_WARN(get_logger(), "Spin finished with code: %d", static_cast<int>(result.code));
            on_done(false);
        };

        spin_client_->async_send_goal(goal, options);
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

    void publishUpperCommand(uint8_t cmd)
    {
        r2_interfaces::msg::ArmCommand msg;
        msg.command = cmd;
        upper_cmd_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Upper body command: %d", cmd);
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

    rclcpp::Publisher<r2_interfaces::msg::ArmCommand>::SharedPtr upper_cmd_pub_;

    rclcpp::Subscription<r2_interfaces::msg::ArmAck>::SharedPtr upper_ack_sub_;
    rclcpp::Subscription<r2_interfaces::msg::ArmDone>::SharedPtr upper_done_sub_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<Spin>::SharedPtr spin_client_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::string nav_frame_id_{"map"};
    double mf_entry_spin_rad_{0.0};
    double mf_exit_spin_rad_{0.0};

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
