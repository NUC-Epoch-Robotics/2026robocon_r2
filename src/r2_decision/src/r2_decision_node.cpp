#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "r2_interfaces/msg/arm_ack.hpp"
#include "r2_interfaces/msg/arm_command.hpp"
#include "r2_interfaces/msg/arm_done.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using Spin = nav2_msgs::action::Spin;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

enum class State
{
    INIT,
    WAIT_START,
    ZONE1_NAV_POINT,
    ZONE1_OPERATE_POINT,
    ZONE1_FINISH,
    GO_TO_MF_EXIT,
    DONE
};

enum class ButtonState : uint8_t
{
    NONE = 0,
    START = 1,
    ZONE1_RETRY = 2,
    ZONE2_RETRY = 3,
    ZONE3_RETRY = 4,
};

struct WaypointTask
{
    int id;
    double x;
    double y;
    double spin_rad;
};

class R2DecisionNode : public rclcpp::Node
{
public:
    R2DecisionNode() : Node("r2_decision_node")
    {
        upper_cmd_pub_ = create_publisher<r2_interfaces::msg::ArmCommand>(
            "/r2/upper_body/command", 10);

        spear_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
            "spearhead/enable", 10);

        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose");

        spin_client_ = rclcpp_action::create_client<Spin>(
            this,
            "spin");

        upper_ack_sub_ = create_subscription<r2_interfaces::msg::ArmAck>(
            "/r2/upper_body/ack", 10,
            std::bind(&R2DecisionNode::onUpperAck, this, _1));

        upper_done_sub_ = create_subscription<r2_interfaces::msg::ArmDone>(
            "/r2/upper_body/done", 10,
            std::bind(&R2DecisionNode::onUpperDone, this, _1));

        spear_exists_sub_ = create_subscription<std_msgs::msg::Bool>(
            "spearhead/exists", 10,
            std::bind(&R2DecisionNode::onSpearExists, this, _1));

        button_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
            "r2/control/button_state", 10,
            std::bind(&R2DecisionNode::onButtonState, this, _1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&R2DecisionNode::tick, this));

        nav_frame_id_ = this->declare_parameter<std::string>("nav_frame_id", "map");
        zone1_arm_command_ = static_cast<uint8_t>(
            this->declare_parameter<int>("zone1_arm_command", r2_interfaces::msg::ArmCommand::GRIPPER_GRAB));

        // TODO: 按现场标定更新 1~6 号点坐标。
        point_table_[1] = {1,
                           this->declare_parameter<double>("zone1_point_1_x", 0.0),
                           this->declare_parameter<double>("zone1_point_1_y", 0.0),
                           this->declare_parameter<double>("zone1_point_1_spin", 0.0)};
        point_table_[2] = {2,
                           this->declare_parameter<double>("zone1_point_2_x", 0.0),
                           this->declare_parameter<double>("zone1_point_2_y", 0.0),
                           this->declare_parameter<double>("zone1_point_2_spin", 0.0)};
        point_table_[3] = {3,
                           this->declare_parameter<double>("zone1_point_3_x", 0.0),
                           this->declare_parameter<double>("zone1_point_3_y", 0.0),
                           this->declare_parameter<double>("zone1_point_3_spin", 0.0)};
        point_table_[4] = {4,
                           this->declare_parameter<double>("zone1_point_4_x", 0.0),
                           this->declare_parameter<double>("zone1_point_4_y", 0.0),
                           this->declare_parameter<double>("zone1_point_4_spin", 0.0)};
        point_table_[5] = {5,
                           this->declare_parameter<double>("zone1_point_5_x", 0.0),
                           this->declare_parameter<double>("zone1_point_5_y", 0.0),
                           this->declare_parameter<double>("zone1_point_5_spin", 0.0)};
        point_table_[6] = {6,
                           this->declare_parameter<double>("zone1_point_6_x", 0.0),
                           this->declare_parameter<double>("zone1_point_6_y", 0.0),
                           this->declare_parameter<double>("zone1_point_6_spin", 0.0)};

        zone1_route_ids_ = {2, 1, 5, 4, 6, 3};

        mf_exit_x_ = this->declare_parameter<double>("mf_exit_x", 3.2);
        mf_exit_y_ = this->declare_parameter<double>("mf_exit_y", 0.0);
        mf_exit_spin_rad_ = this->declare_parameter<double>("mf_exit_spin_rad", 0.0);

        RCLCPP_INFO(get_logger(), "R2 Decision Node Started");
    }

private:
    using CompletionCb = std::function<void(bool)>;

    void tick()
    {
        if (state_ == State::INIT)
        {
            transitionTo(State::WAIT_START);
        }

        handleButtonTransitions();
        handleUpperCommandReliability();
    }

    void handleButtonTransitions()
    {
        if (state_ == State::DONE)
        {
            return;
        }

        if (pending_start_ && state_ == State::WAIT_START)
        {
            pending_start_ = false;
            current_zone1_index_ = 0;
            zone1_arm_retry_count_ = 0;
            publishSpearEnable(true);
            transitionTo(State::ZONE1_NAV_POINT);
            return;
        }

        if (pending_zone1_retry_)
        {
            if (nav_chain_in_progress_)
            {
                return;
            }

            pending_zone1_retry_ = false;
            current_zone1_index_ = 0;
            zone1_arm_retry_count_ = 0;
            publishSpearEnable(true);
            transitionTo(State::ZONE1_NAV_POINT);
            return;
        }

        if (pending_zone2_retry_)
        {
            pending_zone2_retry_ = false;
            RCLCPP_INFO(get_logger(), "ZONE2 retry latched (reserved)");
        }

        if (pending_zone3_retry_)
        {
            pending_zone3_retry_ = false;
            RCLCPP_INFO(get_logger(), "ZONE3 retry latched (reserved)");
        }
    }

    void transitionTo(State next)
    {
        state_ = next;
        RCLCPP_INFO(get_logger(), "Transition to state %d", static_cast<int>(state_));

        if (state_ == State::WAIT_START)
        {
            publishSpearEnable(false);
            RCLCPP_INFO(get_logger(), "Waiting START button");
            return;
        }

        if (state_ == State::ZONE1_NAV_POINT)
        {
            if (current_zone1_index_ >= zone1_route_ids_.size())
            {
                transitionTo(State::ZONE1_FINISH);
                return;
            }

            const int point_id = zone1_route_ids_[current_zone1_index_];
            const auto it = point_table_.find(point_id);
            if (it == point_table_.end())
            {
                RCLCPP_WARN(get_logger(), "Missing point id=%d, skip", point_id);
                ++current_zone1_index_;
                transitionTo(State::ZONE1_NAV_POINT);
                return;
            }

            const auto task = it->second;
            RCLCPP_INFO(get_logger(), "Zone1 nav -> point %d", task.id);
            sendNavigateAndSpin(
                task.x,
                task.y,
                task.spin_rad,
                [this](bool success)
                {
                    if (!success)
                    {
                        RCLCPP_WARN(get_logger(), "Zone1 point nav failed, skip to next point");
                        ++current_zone1_index_;
                        transitionTo(State::ZONE1_NAV_POINT);
                        return;
                    }

                    if (state_ == State::ZONE1_NAV_POINT)
                    {
                        transitionTo(State::ZONE1_OPERATE_POINT);
                    }
                });
            return;
        }

        if (state_ == State::ZONE1_OPERATE_POINT)
        {
            const int point_id = zone1_route_ids_[current_zone1_index_];
            if (!spearhead_exists_)
            {
                RCLCPP_INFO(get_logger(), "Point %d: spearhead not found, go next", point_id);
                ++current_zone1_index_;
                transitionTo(State::ZONE1_NAV_POINT);
                return;
            }

            zone1_arm_retry_count_ = 0;
            RCLCPP_INFO(get_logger(), "Point %d: spearhead found, execute arm cmd=%d", point_id, zone1_arm_command_);
            startReliableUpperCommand(zone1_arm_command_);
            return;
        }

        if (state_ == State::ZONE1_FINISH)
        {
            publishSpearEnable(false);
            transitionTo(State::GO_TO_MF_EXIT);
            return;
        }

        if (state_ == State::GO_TO_MF_EXIT)
        {
            sendNavigateAndSpin(
                mf_exit_x_,
                mf_exit_y_,
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
                        publishSpearEnable(false);
                        transitionTo(State::DONE);
                    }
                });
            return;
        }

        if (state_ == State::DONE)
        {
            publishSpearEnable(false);
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
        nav_chain_in_progress_ = true;
        sendNavigateGoal(
            x,
            y,
            [this, spin_rad, on_done](bool nav_ok)
            {
                if (!nav_ok)
                {
                    nav_chain_in_progress_ = false;
                    on_done(false);
                    return;
                }

                sendSpinGoal(
                    spin_rad,
                    [this, on_done](bool spin_ok)
                    {
                        nav_chain_in_progress_ = false;
                        on_done(spin_ok);
                    });
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

    void publishSpearEnable(bool enable)
    {
        if (spear_camera_enabled_ == enable)
        {
            return;
        }

        std_msgs::msg::Bool msg;
        msg.data = enable;
        spear_enable_pub_->publish(msg);

        spear_camera_enabled_ = enable;
        RCLCPP_INFO(get_logger(), "spearhead camera %s", enable ? "enabled" : "disabled");
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

    void onSpearExists(const std_msgs::msg::Bool::SharedPtr msg)
    {
        spearhead_exists_ = msg->data;
    }

    void onButtonState(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        const auto now_time = now();
        if (msg->data == last_button_state_ &&
            (now_time - last_button_event_time_).nanoseconds() < kButtonDebounceMs * 1000000)
        {
            return;
        }

        last_button_state_ = msg->data;
        last_button_event_time_ = now_time;
        const auto button = static_cast<ButtonState>(msg->data);

        if (button == ButtonState::START)
        {
            pending_start_ = true;
            return;
        }

        if (button == ButtonState::ZONE1_RETRY)
        {
            pending_zone1_retry_ = true;
            return;
        }

        if (button == ButtonState::ZONE2_RETRY)
        {
            pending_zone2_retry_ = true;
            return;
        }

        if (button == ButtonState::ZONE3_RETRY)
        {
            pending_zone3_retry_ = true;
        }
    }

    void onUpperDone(const r2_interfaces::msg::ArmDone::SharedPtr msg)
    {
        if (!msg->done)
        {
            return;
        }

        if (state_ != State::ZONE1_OPERATE_POINT)
        {
            return;
        }

        if (msg->command != zone1_arm_command_)
        {
            return;
        }

        waiting_upper_ack_ = false;

        if (!msg->success)
        {
            if (zone1_arm_retry_count_ < kZone1ArmMaxRetry)
            {
                ++zone1_arm_retry_count_;
                RCLCPP_WARN(get_logger(), "Zone1 arm failed, retry %d/%d", zone1_arm_retry_count_, kZone1ArmMaxRetry);
                startReliableUpperCommand(zone1_arm_command_);
                return;
            }

            RCLCPP_WARN(get_logger(), "Zone1 arm failed after retry, skip point");
        }

        zone1_arm_retry_count_ = 0;
        ++current_zone1_index_;
        transitionTo(State::ZONE1_NAV_POINT);
    }

    static constexpr int64_t kUpperCommandResendPeriodMs = 100;
    static constexpr int64_t kIdleHeartbeatPeriodMs = 500;
    static constexpr int64_t kUpperCommandTimeoutMs = 1200;
    static constexpr int64_t kButtonDebounceMs = 120;
    static constexpr int kZone1ArmMaxRetry = 1;

    State state_{State::INIT};

    rclcpp::Publisher<r2_interfaces::msg::ArmCommand>::SharedPtr upper_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spear_enable_pub_;

    rclcpp::Subscription<r2_interfaces::msg::ArmAck>::SharedPtr upper_ack_sub_;
    rclcpp::Subscription<r2_interfaces::msg::ArmDone>::SharedPtr upper_done_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spear_exists_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_state_sub_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<Spin>::SharedPtr spin_client_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::string nav_frame_id_{"map"};
    uint8_t zone1_arm_command_{r2_interfaces::msg::ArmCommand::GRIPPER_GRAB};

    std::unordered_map<int, WaypointTask> point_table_;
    std::vector<int> zone1_route_ids_;
    std::size_t current_zone1_index_{0};

    double mf_exit_x_{3.2};
    double mf_exit_y_{0.0};
    double mf_exit_spin_rad_{0.0};

    bool spearhead_exists_{false};
    bool spear_camera_enabled_{false};
    bool nav_chain_in_progress_{false};

    bool pending_start_{false};
    bool pending_zone1_retry_{false};
    bool pending_zone2_retry_{false};
    bool pending_zone3_retry_{false};
    uint8_t last_button_state_{static_cast<uint8_t>(ButtonState::NONE)};
    rclcpp::Time last_button_event_time_{0, 0, RCL_ROS_TIME};

    int zone1_arm_retry_count_{0};

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
