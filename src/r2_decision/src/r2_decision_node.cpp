#include <rclcpp/rclcpp.hpp>
#include "r2_interfaces/msg/goal_pose.hpp"
#include "r2_interfaces/msg/goal_reached.hpp"
#include "r2_interfaces/msg/arm_command.hpp"
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
            std::chrono::milliseconds(100),
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
            publishUpperCommand(r2_interfaces::msg::ArmCommand::PICK_KFS);
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
            transitionTo(State::GO_TO_MF_EXIT);
        }
    }

    State state_{State::INIT};

    rclcpp::Publisher<r2_interfaces::msg::GoalPose>::SharedPtr lower_goal_pub_;
    rclcpp::Publisher<r2_interfaces::msg::ArmCommand>::SharedPtr upper_cmd_pub_;

    rclcpp::Subscription<r2_interfaces::msg::GoalReached>::SharedPtr lower_reached_sub_;
    rclcpp::Subscription<r2_interfaces::msg::ArmDone>::SharedPtr upper_done_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<R2DecisionNode>());
    rclcpp::shutdown();
    return 0;
}
