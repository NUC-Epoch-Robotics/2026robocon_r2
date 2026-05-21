#include <algorithm>
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

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "robot_serial/msg/juece.hpp"

#include "r2_interfaces/msg/arm_ack.hpp"
#include "r2_interfaces/msg/arm_done.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

enum class State
{
    INIT,
    WAIT_START,
    ZONE1_NAV_POINT,
    ZONE1_OPERATE_POINT,
    ZONE1_DOCK_R1,
    ZONE1_UP_STAIRS,
    ZONE1_DOWN_STAIRS,
    ZONE1_FINISH,
    ZONE2_ENTRY_GRAB,
    ZONE2_NAV_POINT,
    ZONE2_ROTATE,
    ZONE2_WAIT_SCENE_CONFIRM,
    ZONE2_GRAB,
    ZONE2_UP_STAIRS,
    ZONE2_DOWN_STAIRS,
    ZONE2_FINISH,
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

enum class GrabContext : uint8_t
{
    NONE = 0,
    ENTRY = 1,
    ZONE2_FIXED = 2,
};

enum class StairContext : uint8_t
{
    NORMAL = 0,
    POINT0 = 1,
};

struct WaypointTask
{
    int id;
    double x;
    double y;
    double z;
    uint8_t arm_command{0}; // 该点要发的手臂指令, 0=IDLE
    bool skip_dock{false};  // 非矛头点跳过对接
};

struct Zone2Task
{
    int id;
    double x;
    double y;
    double z;
    double qx{0.0};
    double qy{0.0};
    double qz{0.0};
    double qw{1.0};
    bool use_rotate{false};
    double rqx{0.0};
    double rqy{0.0};
    double rqz{0.0};
    double rqw{1.0};
    double approach_x{0.0};
    double approach_y{0.0};
    uint8_t block_height{0};
    uint8_t stand_height{0};
    uint8_t grab_scene; // 1, 2, or 3
    uint8_t arm_command;
    int8_t stair_cmd{0};      // 1=上, 2=下, 0=无
    uint8_t grab_is_finsh{0}; // is_finsh for this grab
    double rotate_x{0.0};
    double rotate_y{0.0};
};

struct Zone2BlockInfo
{
    double x;
    double y;
    double z;
    uint8_t grab_scene; // 1/2/3, 硬编码
};

struct Zone2FixedPoint
{
    double x;
    double y;
    double z;
    double qx{0.0};
    double qy{0.0};
    double qz{0.0};
    double qw{1.0};
    bool use_rotate{false};
    double rqx{0.0};
    double rqy{0.0};
    double rqz{0.0};
    double rqw{1.0};
    // grab on forest
    double approach_x{0.0};
    double approach_y{0.0};
    uint8_t block_height{0}; // target block height (1/2/3)
    uint8_t stand_height{0}; // stand block height (1/2/3)
    int8_t stair_cmd{0};     // 1=up, 2=down, 0=none
    double rotate_x{0.0};
    double rotate_y{0.0};
};

constexpr int kMaxZone2FixedPoints = 8;

class R2DecisionNode : public rclcpp::Node
{
public:
    R2DecisionNode() : Node("r2_decision_node")
    {
        // ── publishers ──────────────────────────────────────────
        upper_cmd_pub_ = create_publisher<robot_serial::msg::Juece>(
            "/juece", 10);

        spear_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
            "spearhead/enable", 10);

        lightboard_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
            "lightboard/enable", 10);

        grab_scene_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
            "grab_scene/enable", 10);
        grab_scene_expected_pub_ = create_publisher<std_msgs::msg::UInt8>(
            "grab_scene/expected_scene", 10);

        // ── action clients ──────────────────────────────────────
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // ── subscriptions ───────────────────────────────────────
        upper_ack_sub_ = create_subscription<r2_interfaces::msg::ArmAck>(
            "/juece_ack", 10,
            std::bind(&R2DecisionNode::onUpperAck, this, _1));

        upper_done_sub_ = create_subscription<r2_interfaces::msg::ArmDone>(
            "/juece_done", 10,
            std::bind(&R2DecisionNode::onUpperDone, this, _1));

        up_juece_sub_ = create_subscription<robot_serial::msg::Juece>(
            "up_juece", rclcpp::SensorDataQoS(),
            std::bind(&R2DecisionNode::onUpJuece, this, _1));

        down_juece_sub_ = create_subscription<robot_serial::msg::Juece>(
            "down_juece", rclcpp::SensorDataQoS(),
            std::bind(&R2DecisionNode::onDownJuece, this, _1));

        spear_exists_sub_ = create_subscription<std_msgs::msg::Bool>(
            "spearhead/exists", 10,
            std::bind(&R2DecisionNode::onSpearExists, this, _1));

        lightboard_map_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
            "lightboard/map", 10,
            std::bind(&R2DecisionNode::onLightboardMap, this, _1));

        grab_scene_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
            "grab_scene/ready", 10,
            std::bind(&R2DecisionNode::onGrabSceneReady, this, _1));

        button_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
            "r2/control/button_state", 10,
            std::bind(&R2DecisionNode::onButtonState, this, _1));

        // ── main loop ───────────────────────────────────────────
        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&R2DecisionNode::tick, this));

        // ── params ──────────────────────────────────────────────
        nav_frame_id_ = this->declare_parameter<std::string>("nav_frame_id", "odom");

        // Zone1 arm command
        zone1_arm_command_ = static_cast<uint8_t>(
            this->declare_parameter<int>("zone1_arm_command", 0));

        // Zone1 矛头位置: 基准点 + (n-1)*间距 (6个矛头等距排列, 200mm)
        double spearhead_base_x = this->declare_parameter<double>("spearhead_base_x", 0.0);
        double spearhead_base_y = this->declare_parameter<double>("spearhead_base_y", 0.0);
        double spearhead_base_z = this->declare_parameter<double>("spearhead_base_z", 0.0);
        double spearhead_spacing = this->declare_parameter<double>("spearhead_spacing", 0.2);
        for (int n = 1; n <= 6; ++n)
        {
            point_table_[n] = {n,
                               spearhead_base_x + (n - 1) * spearhead_spacing,
                               spearhead_base_y,
                               spearhead_base_z,
                               zone1_arm_command_,
                               false}; // 矛头点: 走抓取→对接流程
        }

        zone1_route_ids_ = this->declare_parameter<std::vector<int64_t>>(
            "zone1_route", std::vector<int64_t>{2, 1, 4, 5, 3, 6});

        // Zone1 额外导航点 (非矛头, ID 7=上台阶, ID 8=下台阶)
        point_table_[7] = {7,
                           this->declare_parameter<double>("zone1_point_2_x", 0.0),
                           this->declare_parameter<double>("zone1_point_2_y", 0.0),
                           this->declare_parameter<double>("zone1_point_2_z", 0.0),
                           0,
                           true}; // 上台阶前导航点, 不发手臂指令
        point_table_[8] = {8,
                           this->declare_parameter<double>("zone1_point_3_x", 0.0),
                           this->declare_parameter<double>("zone1_point_3_y", 0.0),
                           this->declare_parameter<double>("zone1_point_3_z", 0.0),
                           0,
                           true}; // 下台阶前导航点, 不发手臂指令

        // R1 对接位置
        dock_r1_x_ = this->declare_parameter<double>("dock_r1_x", 0.0);
        dock_r1_y_ = this->declare_parameter<double>("dock_r1_y", 0.0);
        dock_r1_z_ = this->declare_parameter<double>("dock_r1_z", 0.0);

        // Zone2 12个方块的物理坐标 和 grab_scene (硬编码场景类型，坐标标定后填入)
        // 地形 (上=入口, 下=出口):
        //   col0 col1 col2
        //    2    1    2    row0 (入口)
        //    3    2    1    row1
        //    2    3    2    row2
        //    1    2    1    row3 (出口)
        for (int idx = 0; idx < 12; ++idx)
        {
            char px[32], py[32], ps[32];
            snprintf(px, sizeof(px), "zone2_block_%d_x", idx);
            snprintf(py, sizeof(py), "zone2_block_%d_y", idx);
            snprintf(ps, sizeof(ps), "zone2_block_%d_z", idx);
            zone2_blocks_[idx].x = this->declare_parameter<double>(px, 0.0);
            zone2_blocks_[idx].y = this->declare_parameter<double>(py, 0.0);
            zone2_blocks_[idx].z = this->declare_parameter<double>(ps, 0.0);
        }
        // grab_scene 硬编码 (入口→出口, 从左到右)
        zone2_blocks_[0].grab_scene = 2;
        zone2_blocks_[1].grab_scene = 1;
        zone2_blocks_[2].grab_scene = 2;
        zone2_blocks_[3].grab_scene = 1;
        zone2_blocks_[4].grab_scene = 2;
        zone2_blocks_[5].grab_scene = 3;
        zone2_blocks_[6].grab_scene = 2;
        zone2_blocks_[7].grab_scene = 3;
        zone2_blocks_[8].grab_scene = 2;
        zone2_blocks_[9].grab_scene = 1;
        zone2_blocks_[10].grab_scene = 2;
        zone2_blocks_[11].grab_scene = 1;

        // 超时参数
        zone1_max_time_s_ = this->declare_parameter<double>("zone1_max_time_s", 120.0);
        scene_confirm_timeout_s_ = this->declare_parameter<double>("scene_confirm_timeout_s", 5.0);
        dock_timeout_s_ = this->declare_parameter<double>("dock_timeout_s", 15.0);

        // MF出口
        mf_exit_x_ = this->declare_parameter<double>("mf_exit_x", 3.2);
        mf_exit_y_ = this->declare_parameter<double>("mf_exit_y", 0.0);
        mf_exit_z_ = this->declare_parameter<double>("mf_exit_z", 0.0);

        // Zone2 固定路线 (6点, 硬编码台阶)
        use_fixed_zone2_route_ = this->declare_parameter<bool>("use_fixed_zone2_route", true);
        zone2_fixed_backoff_ = this->declare_parameter<double>("zone2_fixed_backoff", 0.1);
        zone2_fixed_count_ = this->declare_parameter<int>("zone2_fixed_count", 6);
        if (zone2_fixed_count_ < 0)
            zone2_fixed_count_ = 0;
        if (zone2_fixed_count_ > kMaxZone2FixedPoints)
            zone2_fixed_count_ = kMaxZone2FixedPoints;
        for (int i = 0; i < kMaxZone2FixedPoints; ++i)
        {
            char px[32], py[32], pyaw[32], pqx[32], pqy[32], pqz[32], pqw[32];
            char rqx[32], rqy[32], rqz[32], rqw[32], ruse[32];
            snprintf(px, sizeof(px), "zone2_fixed_%d_x", i);
            snprintf(py, sizeof(py), "zone2_fixed_%d_y", i);
            snprintf(pyaw, sizeof(pyaw), "zone2_fixed_%d_z", i);
            snprintf(pqx, sizeof(pqx), "zone2_fixed_%d_qx", i);
            snprintf(pqy, sizeof(pqy), "zone2_fixed_%d_qy", i);
            snprintf(pqz, sizeof(pqz), "zone2_fixed_%d_qz", i);
            snprintf(pqw, sizeof(pqw), "zone2_fixed_%d_qw", i);
            snprintf(ruse, sizeof(ruse), "zone2_fixed_%d_use_rotate", i);
            snprintf(rqx, sizeof(rqx), "zone2_fixed_%d_rqx", i);
            snprintf(rqy, sizeof(rqy), "zone2_fixed_%d_rqy", i);
            snprintf(rqz, sizeof(rqz), "zone2_fixed_%d_rqz", i);
            snprintf(rqw, sizeof(rqw), "zone2_fixed_%d_rqw", i);
            zone2_fixed_[i].x = this->declare_parameter<double>(px, 0.0);
            zone2_fixed_[i].y = this->declare_parameter<double>(py, 0.0);
            zone2_fixed_[i].z = this->declare_parameter<double>(pyaw, 0.0);
            zone2_fixed_[i].qx = this->declare_parameter<double>(pqx, 0.0);
            zone2_fixed_[i].qy = this->declare_parameter<double>(pqy, 0.0);
            zone2_fixed_[i].qz = this->declare_parameter<double>(pqz, 0.0);
            zone2_fixed_[i].qw = this->declare_parameter<double>(pqw, 1.0);
            zone2_fixed_[i].use_rotate = this->declare_parameter<bool>(ruse, false);
            zone2_fixed_[i].rqx = this->declare_parameter<double>(rqx, 0.0);
            zone2_fixed_[i].rqy = this->declare_parameter<double>(rqy, 0.0);
            zone2_fixed_[i].rqz = this->declare_parameter<double>(rqz, 0.0);
            zone2_fixed_[i].rqw = this->declare_parameter<double>(rqw, 1.0);
            char appx[32], appy[32], bth[32], sth[32], stair[32];
            snprintf(appx, sizeof(appx), "zone2_fixed_%d_approach_x", i);
            snprintf(appy, sizeof(appy), "zone2_fixed_%d_approach_y", i);
            snprintf(bth, sizeof(bth), "zone2_fixed_%d_block_height", i);
            snprintf(sth, sizeof(sth), "zone2_fixed_%d_stand_height", i);
            snprintf(stair, sizeof(stair), "zone2_fixed_%d_stair_cmd", i);
            zone2_fixed_[i].approach_x = this->declare_parameter<double>(appx, 0.0);
            zone2_fixed_[i].approach_y = this->declare_parameter<double>(appy, 0.0);
            zone2_fixed_[i].block_height = static_cast<uint8_t>(this->declare_parameter<int>(bth, 0));
            zone2_fixed_[i].stand_height = static_cast<uint8_t>(this->declare_parameter<int>(sth, 0));
            zone2_fixed_[i].stair_cmd = static_cast<int8_t>(
                this->declare_parameter<int>(stair, 0));
            char rotx[32], roty[32];
            snprintf(rotx, sizeof(rotx), "zone2_fixed_%d_rotate_x", i);
            snprintf(roty, sizeof(roty), "zone2_fixed_%d_rotate_y", i);
            zone2_fixed_[i].rotate_x = this->declare_parameter<double>(rotx, 0.0);
            zone2_fixed_[i].rotate_y = this->declare_parameter<double>(roty, 0.0);
        }

        // Zone2 入口抓取参数 (x=1.6 ↔ x=2.0, 入口区地面→梅花林row0方块)
        entry_approach_x_ = this->declare_parameter<double>("entry_approach_x", 1.6);
        entry_block0_x_ = this->declare_parameter<double>("entry_block0_x", 2.0);
        entry_block0_y_ = this->declare_parameter<double>("entry_block0_y", 0.289);
        entry_block0_is_finsh_ = static_cast<uint8_t>(this->declare_parameter<int>("entry_block0_is_finsh", 2));
        entry_block2_x_ = this->declare_parameter<double>("entry_block2_x", 3.0);
        entry_block2_y_ = this->declare_parameter<double>("entry_block2_y", 1.41);
        entry_block2_is_finsh_ = static_cast<uint8_t>(this->declare_parameter<int>("entry_block2_is_finsh", 1));

        // ── 模拟模式: 无真实硬件时打印决策输出 ──
        sim_mode_ = this->declare_parameter<bool>("sim_mode", false);

        RCLCPP_INFO(get_logger(), "R2 Decision Node Started (sim_mode=%d)", sim_mode_);
    }

private:
    using CompletionCb = std::function<void(bool)>;

    // ═════════════════════════════════════════════════════════════
    // 主循环
    // ═════════════════════════════════════════════════════════════

    void tick()
    {
        if (state_ == State::INIT)
        {
            transitionTo(State::WAIT_START);
        }

        handleButtonTransitions();
        handleUpperCommandReliability();
        handleDockSpearheadTransition();
        handleSceneConfirmTimeout();
        handleZone1TimeLimit();
    }

    // ═════════════════════════════════════════════════════════════
    // 按钮
    // ═════════════════════════════════════════════════════════════

    void handleButtonTransitions()
    {
        if (state_ == State::DONE)
            return;

        if (pending_start_ && state_ == State::WAIT_START)
        {
            pending_start_ = false;
            current_zone1_index_ = 0;
            dock_success_count_ = 0;
            zone1_arm_retry_count_ = 0;
            zone1_start_time_ = now();
            publishSpearEnable(true);
            transitionTo(State::ZONE1_NAV_POINT);
            return;
        }

        if (pending_zone1_retry_)
        {
            if (nav_chain_in_progress_)
                return;
            pending_zone1_retry_ = false;
            current_zone1_index_ = 0;
            dock_success_count_ = 0;
            zone1_arm_retry_count_ = 0;
            zone1_start_time_ = now();
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

    // ═════════════════════════════════════════════════════════════
    // 对接: 矛头消失 = 对接成功
    // ═════════════════════════════════════════════════════════════

    void handleDockSpearheadTransition()
    {
        if (state_ != State::ZONE1_DOCK_R1)
            return;
        if (nav_chain_in_progress_)
            return;

        // 尚未到达对接位时，记录矛头初始状态
        if (!dock_arrived_)
        {
            dock_arrived_ = true;
            spearhead_was_present_ = spearhead_exists_;
            dock_start_time_ = now();
            RCLCPP_INFO(get_logger(),
                        "Arrived at R1 dock. spearhead present=%d, waiting for R1 to take it...",
                        spearhead_was_present_);
            return;
        }

        // 矛头从有→无 = R1取走, 对接成功
        if (spearhead_was_present_ && !spearhead_exists_)
        {
            dock_success_count_++;
            RCLCPP_INFO(get_logger(),
                        ">>> DOCK SUCCESS #%d: spearhead taken by R1 <<<",
                        dock_success_count_);

            // 记录光板数据
            if (lightboard_map_received_)
            {
                latest_lightboard_map_ = lightboard_map_;
                RCLCPP_INFO(get_logger(),
                            "Lightboard map captured: [%d %d %d %d %d %d %d %d %d %d %d %d]",
                            latest_lightboard_map_[0], latest_lightboard_map_[1],
                            latest_lightboard_map_[2], latest_lightboard_map_[3],
                            latest_lightboard_map_[4], latest_lightboard_map_[5],
                            latest_lightboard_map_[6], latest_lightboard_map_[7],
                            latest_lightboard_map_[8], latest_lightboard_map_[9],
                            latest_lightboard_map_[10], latest_lightboard_map_[11]);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "No lightboard map received during dock");
            }

            // 判断是否继续一区循环
            bool keep_going = (dock_success_count_ < kMaxDocks) &&
                              (current_zone1_index_ + 1 < zone1_route_ids_.size());

            if (keep_going)
            {
                ++current_zone1_index_;
                transitionTo(State::ZONE1_NAV_POINT);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Zone1 docks done: %d, going up stairs",
                            dock_success_count_);
                transitionTo(State::ZONE1_UP_STAIRS);
            }
            return;
        }

        // 对接超时
        auto elapsed = (now() - dock_start_time_).seconds();
        if (elapsed > dock_timeout_s_)
        {
            RCLCPP_WARN(get_logger(), "Dock timeout (%.1fs), skip this dock", elapsed);
            ++current_zone1_index_;
            transitionTo(State::ZONE1_NAV_POINT);
        }
    }

    // ═════════════════════════════════════════════════════════════
    // 场景确认超时
    // ═════════════════════════════════════════════════════════════

    void handleSceneConfirmTimeout()
    {
        if (state_ != State::ZONE2_WAIT_SCENE_CONFIRM)
            return;

        auto elapsed = (now() - scene_confirm_start_time_).seconds();
        if (elapsed > scene_confirm_timeout_s_)
        {
            RCLCPP_WARN(get_logger(), "Scene confirm timeout (%.1fs), skip block", elapsed);
            publishGrabSceneEnable(false);
            ++current_zone2_index_;
            transitionTo(State::ZONE2_NAV_POINT);
        }
    }

    // ═════════════════════════════════════════════════════════════
    // 一区总时间限制
    // ═════════════════════════════════════════════════════════════

    void handleZone1TimeLimit()
    {
        if (state_ < State::ZONE1_NAV_POINT || state_ > State::ZONE1_DOCK_R1)
            return;

        auto elapsed = (now() - zone1_start_time_).seconds();
        if (elapsed > zone1_max_time_s_)
        {
            RCLCPP_WARN(get_logger(), "Zone1 time limit reached (%.1fs), going up stairs", elapsed);
            publishSpearEnable(false);
            publishLightboardEnable(false);
            transitionTo(State::ZONE1_UP_STAIRS);
        }
    }

    // ═════════════════════════════════════════════════════════════
    // 状态机
    // ═════════════════════════════════════════════════════════════

    void transitionTo(State next)
    {
        if (sim_mode_)
        {
            RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            RCLCPP_INFO(get_logger(), "  [SIM] state: %d → %d", static_cast<int>(state_),
                        static_cast<int>(next));
            RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        }

        state_ = next;

        if (state_ == State::WAIT_START)
        {
            publishSpearEnable(false);
            publishLightboardEnable(false);
            RCLCPP_INFO(get_logger(), "Waiting START button...");
            return;
        }

        // ── Zone1 导航到点 ──────────────────────────────────────
        if (state_ == State::ZONE1_NAV_POINT)
        {
            if (current_zone1_index_ >= zone1_route_ids_.size())
            {
                // 固定路线: 先入口抓块再上台阶(台阶在 Zone2 固定路线点0)
                // 非固定路线: 保持原有上下台阶流程
                if (use_fixed_zone2_route_)
                    transitionTo(State::ZONE1_FINISH);
                else
                    transitionTo(State::ZONE1_UP_STAIRS);
                return;
            }

            const int point_id = zone1_route_ids_[current_zone1_index_];
            const auto it = point_table_.find(point_id);
            if (it == point_table_.end())
            {
                RCLCPP_WARN(get_logger(), "Zone1: missing point id=%d, skip", point_id);
                ++current_zone1_index_;
                transitionTo(State::ZONE1_NAV_POINT);
                return;
            }

            const auto task = it->second;
            RCLCPP_INFO(get_logger(), "Zone1: nav → point %d (%.2f, %.2f)",
                        task.id, task.x, task.y);
            sendNavigateWithQuat(
                task.x, task.y, task.z, 0.0, 0.0, 0.0, 1.0,
                [this](bool success)
                {
                    if (!success)
                    {
                        RCLCPP_WARN(get_logger(), "Zone1: nav to point failed, skip");
                        ++current_zone1_index_;
                        transitionTo(State::ZONE1_NAV_POINT);
                        return;
                    }
                    if (state_ == State::ZONE1_NAV_POINT)
                        transitionTo(State::ZONE1_OPERATE_POINT);
                });
            return;
        }

        // ── Zone1 操作点 ──────────────────────────────────────
        if (state_ == State::ZONE1_OPERATE_POINT)
        {
            const int point_id = zone1_route_ids_[current_zone1_index_];
            const auto it = point_table_.find(point_id);
            const auto task = it->second;

            if (task.arm_command == 0)
            {
                RCLCPP_INFO(get_logger(), "Zone1 point %d: arm cmd=0, skip", point_id);
                ++current_zone1_index_;
                transitionTo(State::ZONE1_NAV_POINT);
                return;
            }

            if (!task.skip_dock && !spearhead_exists_)
            {
                RCLCPP_INFO(get_logger(), "Zone1 point %d: no spearhead, skip", point_id);
                ++current_zone1_index_;
                transitionTo(State::ZONE1_NAV_POINT);
                return;
            }

            zone1_arm_retry_count_ = 0;
            RCLCPP_INFO(get_logger(), "Zone1 point %d: arm cmd=%d", point_id, task.arm_command);
            startReliableUpperCommand(task.arm_command);
            return;
        }

        // ── Zone1 对接 R1 ───────────────────────────────────────
        if (state_ == State::ZONE1_DOCK_R1)
        {
            dock_arrived_ = false;
            publishLightboardEnable(true);

            RCLCPP_INFO(get_logger(), "Zone1: nav → R1 dock (%.2f, %.2f)",
                        dock_r1_x_, dock_r1_y_);
            sendNavigateWithQuat(
                dock_r1_x_, dock_r1_y_, dock_r1_z_, 0.0, 0.0, 0.0, 1.0,
                [this](bool success)
                {
                    if (!success)
                    {
                        RCLCPP_WARN(get_logger(), "Zone1: nav to R1 dock failed!");
                        return;
                    }
                    // 导航到达, tick() 中的 handleDockSpearheadTransition 接管等待
                });
            return;
        }

        // ── 上台阶 ──────────────────────────────────────────────
        if (state_ == State::ZONE1_UP_STAIRS)
        {
            publishSpearEnable(false);
            publishLightboardEnable(false);
            RCLCPP_INFO(get_logger(), "Zone1: UP_STAIRS — 10Hz publish status_bit 0→1");
            startStairPublishing(1);
            return;
        }

        // ── 下台阶 ──────────────────────────────────────────────
        if (state_ == State::ZONE1_DOWN_STAIRS)
        {
            RCLCPP_INFO(get_logger(), "Zone1: DOWN_STAIRS — 10Hz publish status_bit 0→2");
            startStairPublishing(2);
            return;
        }

        // ── Zone1 结束 → 规划 Zone2 ─────────────────────────────
        if (state_ == State::ZONE1_FINISH)
        {
            publishSpearEnable(false);
            publishLightboardEnable(false);
            if (use_fixed_zone2_route_)
                buildZone2FixedRoute();
            else
                buildZone2Route(latest_lightboard_map_);
            current_zone2_index_ = 0;

            if (zone2_tasks_.empty())
            {
                RCLCPP_WARN(get_logger(), "Zone2: no tasks, mission done");
                transitionTo(State::DONE);
            }
            else if (use_fixed_zone2_route_)
            {
                RCLCPP_INFO(get_logger(), "Zone2: %zu tasks planned, start entry grab", zone2_tasks_.size());
                entry_grab_step_ = 0;
                transitionTo(State::ZONE2_ENTRY_GRAB);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Zone2: %zu tasks planned", zone2_tasks_.size());
                transitionTo(State::ZONE2_NAV_POINT);
            }
            return;
        }

        // ── Zone2 入口抓取 (仅 block0 col0 y=0.289; block2 移至 zone2_fixed_0) ─
        if (state_ == State::ZONE2_ENTRY_GRAB)
        {
            const double block_x = entry_block0_x_;
            const double block_y = entry_block0_y_;
            const uint8_t is_finsh = entry_block0_is_finsh_;

            // step 0: nav to approach
            if (entry_grab_step_ == 0)
            {
                RCLCPP_INFO(get_logger(), "EntryGrab step0: nav approach (%.2f,%.2f)", entry_approach_x_, block_y);
                entry_grab_step_ = 1;
                sendNavigateWithQuat(
                    entry_approach_x_, block_y, 0.0, 0, 0, 0, 1,
                    [this](bool)
                    { if (state_ == State::ZONE2_ENTRY_GRAB) transitionTo(State::ZONE2_ENTRY_GRAB); });
                return;
            }

            // step 1: open arm, move to block
            if (entry_grab_step_ == 1)
            {
                if (grab_context_ != GrabContext::ENTRY)
                {
                    RCLCPP_INFO(get_logger(), "EntryGrab step1: START is_finsh=%d, forward to block", is_finsh);
                    startEntryGrabPublish(is_finsh);
                    entry_grab_step_ = 2;
                    sendNavigateWithQuat(
                        block_x, block_y, 0.0, 0, 0, 0, 1,
                        [this](bool)
                        { if (state_ == State::ZONE2_ENTRY_GRAB) transitionTo(State::ZONE2_ENTRY_GRAB); });
                }
                return;
            }

            // step 2: wait for up_juece done
            if (entry_grab_step_ == 2)
                return;

            // step 3: retreat to approach, then finish
            if (entry_grab_step_ == 3)
            {
                RCLCPP_INFO(get_logger(), "EntryGrab step3: retreat done, retract and continue");
                stopEntryGrabPublish();
                entry_grab_step_ = 0;
                transitionTo(State::ZONE2_NAV_POINT);
                return;
            }
            return;
        }

        // ── Zone2 导航到方块 ────────────────────────────────────
        if (state_ == State::ZONE2_NAV_POINT)
        {
            if (current_zone2_index_ >= zone2_tasks_.size())
            {
                transitionTo(State::ZONE2_FINISH);
                return;
            }

            const auto &task = zone2_tasks_[current_zone2_index_];
            const bool has_grab = (task.approach_x != 0.0 || task.approach_y != 0.0);
            const double nav_x = has_grab ? task.approach_x : task.x;
            const double nav_y = has_grab ? task.approach_y : task.y;

            RCLCPP_INFO(get_logger(), "Zone2: nav → point %d (%.2f,%.2f) z=%.3f stair=%d grab=%d",
                        task.id, nav_x, nav_y, task.z, task.stair_cmd, has_grab);

            sendNavigateWithQuat(
                nav_x, nav_y, task.z, task.qx, task.qy, task.qz, task.qw,
                [this, task, has_grab](bool success)
                {
                    if (!success)
                    {
                        RCLCPP_WARN(get_logger(), "Zone2: nav failed, skip");
                        ++current_zone2_index_;
                        transitionTo(State::ZONE2_NAV_POINT);
                        return;
                    }
                    if (state_ != State::ZONE2_NAV_POINT)
                        return;
                    if (use_fixed_zone2_route_)
                    {
                        if (has_grab)
                        {
                            zone2_grab_step_ = 0;
                            zone2_point0_substep_ = 0;
                            transitionTo(State::ZONE2_GRAB);
                        }
                        else if (task.use_rotate)
                            transitionTo(State::ZONE2_ROTATE);
                        else if (task.stair_cmd == 1)
                            transitionTo(State::ZONE2_UP_STAIRS);
                        else if (task.stair_cmd == 2)
                            transitionTo(State::ZONE2_DOWN_STAIRS);
                        else
                            transitionTo(State::ZONE2_FINISH);
                    }
                    else
                    {
                        transitionTo(State::ZONE2_WAIT_SCENE_CONFIRM);
                    }
                });
            return;
        }

        // ── Zone2 原地转向 ──────────────────────────────────────────
        if (state_ == State::ZONE2_ROTATE)
        {
            const auto &task = zone2_tasks_[current_zone2_index_];
            RCLCPP_INFO(get_logger(), "Zone2: ROTATE @ (%.2f,%.2f) q=(%.3f,%.3f,%.3f,%.3f)",
                        task.x, task.y, task.rqx, task.rqy, task.rqz, task.rqw);

            sendNavigateWithQuat(
                task.x, task.y, task.z, task.rqx, task.rqy, task.rqz, task.rqw,
                [this, task](bool success)
                {
                    if (!success)
                        RCLCPP_WARN(get_logger(), "Zone2: rotate nav failed");
                    if (state_ != State::ZONE2_ROTATE)
                        return;
                    if (task.stair_cmd == 1 || task.stair_cmd == 2)
                    {
                        zone2_post_rotate_stairs_done_ = true;
                        if (task.stair_cmd == 1)
                            transitionTo(State::ZONE2_UP_STAIRS);
                        else
                            transitionTo(State::ZONE2_DOWN_STAIRS);
                    }
                    else
                    {
                        ++current_zone2_index_;
                        transitionTo(State::ZONE2_NAV_POINT);
                    }
                });
            return;
        }

        // ── Zone2 上台阶 ──────────────────────────────────────────
        if (state_ == State::ZONE2_UP_STAIRS)
        {
            RCLCPP_INFO(get_logger(), "Zone2: UP_STAIRS — 10Hz cmd=1");
            startStairPublishing(1);
            return;
        }

        // ── Zone2 下台阶 ──────────────────────────────────────────
        if (state_ == State::ZONE2_DOWN_STAIRS)
        {
            RCLCPP_INFO(get_logger(), "Zone2: DOWN_STAIRS — 10Hz cmd=2");
            startStairPublishing(2);
            return;
        }

        // ── Zone2 等待场景确认 ──────────────────────────────────
        if (state_ == State::ZONE2_WAIT_SCENE_CONFIRM)
        {
            const auto &task = zone2_tasks_[current_zone2_index_];
            publishGrabSceneExpected(task.grab_scene);
            publishGrabSceneEnable(true);
            scene_confirm_start_time_ = now();
            RCLCPP_INFO(get_logger(), "Zone2 block %d: waiting scene %d confirmation...",
                        task.id, task.grab_scene);
            // onGrabSceneReady 驱动跳转
            return;
        }

        // ── Zone2 抓取 ──────────────────────────────────────────
        if (state_ == State::ZONE2_GRAB)
        {
            const auto &task = zone2_tasks_[current_zone2_index_];

            if (use_fixed_zone2_route_)
            {
                if (task.id == 0 && zone2_point0_sequence_active_)
                {
                    handlePoint0Substep(task);
                    return;
                }
                if (task.id == 0)
                {
                    if (zone2_grab_step_ == 0)
                    {
                        if (grab_context_ != GrabContext::ZONE2_FIXED)
                        {
                            RCLCPP_INFO(get_logger(), "Point0 grab: START is_finsh=%d", task.grab_is_finsh);
                            startZone2GrabPublish(task.grab_is_finsh);
                        }
                        RCLCPP_INFO(get_logger(), "Point0 grab: forward to block (%.2f,%.2f)", task.x, task.y);
                        zone2_grab_step_ = 1;
                        sendNavigateWithQuat(
                            task.x, task.y, 0.0, task.qx, task.qy, task.qz, task.qw,
                            [this](bool)
                            { if (state_ == State::ZONE2_GRAB) transitionTo(State::ZONE2_GRAB); });
                        return;
                    }
                    if (zone2_grab_step_ == 1)
                    {
                        return;
                    }
                    if (zone2_grab_step_ == 3)
                    {
                        zone2_point0_sequence_active_ = true;
                        zone2_grab_step_ = 0;
                        handlePoint0Substep(task);
                        return;
                    }
                    return;
                }
                if (grab_context_ != GrabContext::ZONE2_FIXED)
                {
                    if (zone2_grab_step_ == 0)
                    {
                        const double yaw = yawFromQuat(task.qx, task.qy, task.qz, task.qw);
                        const double backoff_x = task.approach_x - std::cos(yaw) * zone2_fixed_backoff_;
                        const double backoff_y = task.approach_y - std::sin(yaw) * zone2_fixed_backoff_;
                        RCLCPP_INFO(get_logger(), "Zone2Grab point %d: backoff to (%.2f,%.2f)",
                                    task.id, backoff_x, backoff_y);
                        zone2_grab_step_ = 1;
                        sendNavigateWithQuat(
                            backoff_x, backoff_y, 0.0, task.qx, task.qy, task.qz, task.qw,
                            [this](bool)
                            { if (state_ == State::ZONE2_GRAB) transitionTo(State::ZONE2_GRAB); });
                        return;
                    }
                    if (zone2_grab_step_ == 1)
                    {
                        RCLCPP_INFO(get_logger(), "Zone2Grab point %d: START is_finsh=%d", task.id, task.grab_is_finsh);
                        startZone2GrabPublish(task.grab_is_finsh);
                        zone2_grab_step_ = 2;
                    }
                }
                return;
            }

            // 光板路线: 原有场景确认+手臂指令
            publishGrabSceneEnable(false);
            RCLCPP_INFO(get_logger(), "Zone2 block %d: scene confirmed, arm cmd=%d",
                        task.id, task.arm_command);
            zone2_arm_retry_count_ = 0;
            startReliableUpperCommand(task.arm_command);
            return;
        }

        // ── Zone2 结束 ──────────────────────────────────────────
        if (state_ == State::ZONE2_FINISH)
        {
            publishGrabSceneEnable(false);
            if (use_fixed_zone2_route_)
            {
                RCLCPP_INFO(get_logger(), "Zone2 finished, mission complete");
                transitionTo(State::DONE);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Zone2 finished, go to MF exit");
                transitionTo(State::GO_TO_MF_EXIT);
            }
            return;
        }

        // ── 去MF出口 ────────────────────────────────────────────
        if (state_ == State::GO_TO_MF_EXIT)
        {
            sendNavigateWithQuat(
                mf_exit_x_, mf_exit_y_, mf_exit_z_, 0.0, 0.0, 0.0, 1.0,
                [this](bool success)
                {
                    if (!success)
                        RCLCPP_WARN(get_logger(), "GO_TO_MF_EXIT nav failed");
                    if (state_ == State::GO_TO_MF_EXIT)
                    {
                        publishSpearEnable(false);
                        transitionTo(State::DONE);
                    }
                });
            return;
        }

        // ── 完赛 ────────────────────────────────────────────────
        if (state_ == State::DONE)
        {
            publishSpearEnable(false);
            publishLightboardEnable(false);
            RCLCPP_INFO(get_logger(), "=== MISSION COMPLETE ===");
        }
    }

    // ═════════════════════════════════════════════════════════════
    // Zone2 路径规划 — 遵循规则 4.4.13~4.4.19
    // ═════════════════════════════════════════════════════════════
    //
    // 树林 3×4 布局 (入口在上, 出口在下):
    //   col:  0   1   2
    //   row0:  0   1   2   ← 入口方块 1/2/3 (4.4.13)
    //   row1:  3   4   5
    //   row2:  6   7   8
    //   row3:  9  10  11   ← 出口方块 10/11/12 (4.4.19)
    //
    // 约束:
    //   4.4.14 — 只能拿取相邻方块(四连通)上的 KFS
    //   4.4.13 — 必须从入口方块(0/1/2)进入树林
    //   4.4.19 — 必须从出口方块(9/10/11)离开树林
    //   4.4.15 — 入口方块有 R2 KFS 时，第一个必须从入口区收集
    //   4.4.16 — 离开树林前必须携带至少 1 个 R2 KFS
    //   8.8    — 不得接触假 KFS (FAKE=3)
    //   8.7    — 不得接触 R1 KFS (R1=1)
    //   8.9    — 不得完全进入有 KFS 的方块 (R2 从相邻方块部分进入)

    // 四连通邻接表
    static constexpr int ADJ[12][4] = {
        {1, 3, -1, -1},  // 0
        {0, 2, 4, -1},   // 1
        {1, 5, -1, -1},  // 2
        {0, 4, 6, -1},   // 3
        {1, 3, 5, 7},    // 4
        {2, 4, 8, -1},   // 5
        {3, 7, 9, -1},   // 6
        {4, 6, 8, 10},   // 7
        {5, 7, 11, -1},  // 8
        {6, 10, -1, -1}, // 9
        {7, 9, 11, -1},  // 10
        {8, 10, -1, -1}, // 11
    };

    static bool is_entry_block(int idx) { return idx == 0 || idx == 1 || idx == 2; }
    static bool is_exit_block(int idx) { return idx == 9 || idx == 10 || idx == 11; }

    void buildZone2FixedRoute()
    {
        zone2_tasks_.clear();

        for (int i = 0; i < zone2_fixed_count_; ++i)
        {
            Zone2Task t;
            t.id = i;
            t.x = zone2_fixed_[i].x;
            t.y = zone2_fixed_[i].y;
            t.z = zone2_fixed_[i].z;
            t.qx = zone2_fixed_[i].qx;
            t.qy = zone2_fixed_[i].qy;
            t.qz = zone2_fixed_[i].qz;
            t.qw = zone2_fixed_[i].qw;
            t.use_rotate = zone2_fixed_[i].use_rotate;
            t.rqx = zone2_fixed_[i].rqx;
            t.rqy = zone2_fixed_[i].rqy;
            t.rqz = zone2_fixed_[i].rqz;
            t.rqw = zone2_fixed_[i].rqw;
            t.approach_x = zone2_fixed_[i].approach_x;
            t.approach_y = zone2_fixed_[i].approach_y;
            t.block_height = zone2_fixed_[i].block_height;
            t.stand_height = zone2_fixed_[i].stand_height;
            t.rotate_x = zone2_fixed_[i].rotate_x;
            t.rotate_y = zone2_fixed_[i].rotate_y;
            t.grab_scene = 0;
            t.arm_command = 0;
            // 高度差 → is_finsh: Δh=1→1, Δh=2→2, 负→3
            if (t.approach_x != 0.0 || t.approach_y != 0.0)
            {
                int dh = static_cast<int>(t.block_height) - static_cast<int>(t.stand_height);
                if (dh == 1)
                    t.grab_is_finsh = 1;
                else if (dh == 2)
                    t.grab_is_finsh = 2;
                else if (dh < 0)
                    t.grab_is_finsh = 3;
                else
                    t.grab_is_finsh = 0;
            }

            // 计算与下个点的高度差决定台阶方向
            if (zone2_fixed_[i].stair_cmd != 0)
            {
                t.stair_cmd = zone2_fixed_[i].stair_cmd;
            }
            else if (i + 1 < zone2_fixed_count_)
            {
                double dz = zone2_fixed_[i + 1].z - zone2_fixed_[i].z;
                t.stair_cmd = (dz > 0) ? 1 : 2; // 正=上, 负=下
            }
            else
            {
                t.stair_cmd = 0; // 最后一个点, 无台阶
            }

            zone2_tasks_.push_back(t);
        }

        RCLCPP_INFO(get_logger(), "buildZone2FixedRoute: %zu tasks:", zone2_tasks_.size());
        for (size_t i = 0; i < zone2_tasks_.size(); ++i)
        {
            const auto &t = zone2_tasks_[i];
            RCLCPP_INFO(get_logger(),
                        "  #%zu (%.2f,%.2f) z=%.3f stair=%d app=(%.2f,%.2f) "
                        "h_stand=%d h_block=%d is_finsh=%d rot=%d",
                        i, t.x, t.y, t.z, t.stair_cmd, t.approach_x, t.approach_y,
                        t.stand_height, t.block_height, t.grab_is_finsh, t.use_rotate);
        }
    }

    void buildZone2Route(const std::vector<uint8_t> &lightboard_map)
    {
        zone2_tasks_.clear();

        // ── 无光板数据: 跳过 Zone2, 直通 DONE ──────────────────
        if (lightboard_map.size() != 12)
        {
            RCLCPP_WARN(get_logger(),
                        "buildZone2Route: no lightboard data (size=%zu), Zone2 skipped",
                        lightboard_map.size());
            return; // zone2_tasks_ 留空, ZONE1_FINISH → DONE
        }

        // ── 1. 收集 R2 目标方块 & 构建可通过性 ──────────────
        // 可通过: EMPTY(0) 或 R2(2); 不可通过: R1(1) 或 FAKE(3)
        // 注意: R2 方块允许"部分进入"(FAQ), 故视为可通过
        std::vector<int> r2_targets;
        bool passable[12] = {false};

        for (int i = 0; i < 12; ++i)
        {
            if (lightboard_map[i] == 2)
                r2_targets.push_back(i);
            passable[i] = (lightboard_map[i] == 0 || lightboard_map[i] == 2);
        }

        // ── 2. 无 R2 KFS 可收集 ─────────────────────────────
        if (r2_targets.empty())
        {
            RCLCPP_WARN(get_logger(),
                        "buildZone2Route: no R2 blocks on lightboard "
                        "(4.4.16 violated: cannot leave forest without KFS!)");
            return;
        }

        // ── 3. BFS 计算入口到各方块的最短距离 ────────────────
        // 入口区与 {0,1,2} 相邻
        int dist[12];
        std::fill_n(dist, 12, -1);
        {
            // 手动 BFS (无 queue 依赖, 网格很小)
            int queue[12], head = 0, tail = 0;
            for (int e : {0, 1, 2})
            {
                if (passable[e])
                {
                    dist[e] = 1;
                    queue[tail++] = e;
                }
            }
            while (head < tail)
            {
                int cur = queue[head++];
                for (int ni = 0; ni < 4; ++ni)
                {
                    int nb = ADJ[cur][ni];
                    if (nb < 0)
                        continue;
                    if (dist[nb] < 0 && passable[nb])
                    {
                        dist[nb] = dist[cur] + 1;
                        queue[tail++] = nb;
                    }
                }
            }
        }

        // ── 4. 规则 4.4.15: 入口方块有 R2 KFS → 第一个从入口区取 ──
        // 排序: 入口 R2 排最前, 其余按 BFS 距离升序
        std::sort(r2_targets.begin(), r2_targets.end(),
                  [&](int a, int b)
                  {
                      bool ae = is_entry_block(a), be = is_entry_block(b);
                      if (ae != be)
                          return ae > be; // 入口方块优先
                      if (dist[a] != dist[b])
                          return dist[a] < dist[b]; // 距离近的优先
                      return a < b;                 // 同距离按索引
                  });

        // ── 5. 检查不可达方块 ────────────────────────────────
        for (int t : r2_targets)
        {
            if (dist[t] < 0)
                RCLCPP_WARN(get_logger(),
                            "buildZone2Route: R2 block %d is UNREACHABLE (no passable path from entry)!",
                            t);
        }

        // ── 6. 贪心生成访问顺序 (保证相邻性) ─────────────────
        // current_pos = -1 表示入口区 (与 {0,1,2} 相邻)
        int current_pos = -1;
        std::vector<int> plan;

        for (int target : r2_targets)
        {
            if (dist[target] < 0)
                continue; // 不可达, 跳过

            // 找到从 current_pos 到 target 的路径上的第一个中间方块
            // (BFS 路径反推, 简单场景下直接取 target 的 BFS 前驱)
            int step = findFirstStep(current_pos, target, passable);
            if (step >= 0 && step != target && !isAlreadyPlanned(plan, step))
            {
                // 此中间方块是 R2 站立取 KFS 的位置
                plan.push_back(step);
            }
            plan.push_back(target);
            passable[target] = true; // KFS 取走后该方块变为完全可通过
            current_pos = target;
        }

        // ── 7. 检查出口可达性 (4.4.19) ───────────────────────
        {
            bool exit_reachable = false;
            if (current_pos < 0)
            {
                // 还在入口区 — 只有入口方块有 KFS 且已取完的情况
                exit_reachable = true; // 从入口区可直接到出口方块
            }
            else
            {
                for (int ex : {9, 10, 11})
                    if (passable[ex] && isAdjacent(current_pos, ex))
                        exit_reachable = true;
                // 放宽: 只要出口方块可通过即可 (相邻约束由最后一次移动满足)
                for (int ex : {9, 10, 11})
                    if (passable[ex])
                        exit_reachable = true;
            }
            if (!exit_reachable)
                RCLCPP_WARN(get_logger(),
                            "buildZone2Route: NO passable exit! (4.4.19 may be violated)");
        }

        // ── 8. 生成 Zone2Task 列表 ────────────────────────────
        for (int idx : plan)
        {
            Zone2Task t;
            t.id = idx;
            t.x = zone2_blocks_[idx].x;
            t.y = zone2_blocks_[idx].y;
            t.z = zone2_blocks_[idx].z;
            t.grab_scene = zone2_blocks_[idx].grab_scene;
            t.arm_command = sceneToArmCmd(t.grab_scene);
            zone2_tasks_.push_back(t);
        }

        // ── 9. 规则 4.4.16 检查 ───────────────────────────────
        if (zone2_tasks_.empty())
        {
            RCLCPP_WARN(get_logger(),
                        "buildZone2Route: 0 tasks planned! R2 has no KFS to carry out of forest. "
                        "(4.4.16: must carry >=1 R2 KFS when leaving)");
        }

        // ── dump ──────────────────────────────────────────────
        RCLCPP_INFO(get_logger(),
                    "buildZone2Route: %zu tasks (entry->exit, adjacency-respecting):",
                    zone2_tasks_.size());
        int task_idx = 0;
        for (const auto &t : zone2_tasks_)
        {
            const char *pos = is_entry_block(t.id)  ? "[ENTRY]"
                              : is_exit_block(t.id) ? "[EXIT]"
                                                    : "";
            RCLCPP_INFO(get_logger(), "  #%d block %d %s (%.2f,%.2f) scene=%d cmd=%d",
                        task_idx++, t.id, pos, t.x, t.y, t.grab_scene, t.arm_command);
        }
    }

    // ── 规划辅助函数 ────────────────────────────────────────────

    // 从入口区(-1)或方块 pos 出发, 找到去 target 路径上的第一个非 target 方块
    // 用于确定 R2 站立位置 (从相邻方块伸手拿 KFS, 符合 8.9)
    static int findFirstStep(int pos, int target, const bool passable[12])
    {
        // 入口区: target 是入口方块时, step = -1 (从入口区直接拿)
        if (pos < 0 && is_entry_block(target))
            return -1; // 从入口区直接伸手

        // BFS 反推: 从 target 往回找, 直到碰到 pos
        int prev[12];
        std::fill_n(prev, 12, -2);
        int queue[12], head = 0, tail = 0;

        if (pos >= 0)
        {
            prev[pos] = -1;
            queue[tail++] = pos;
        }
        else
        {
            for (int e : {0, 1, 2})
            {
                if (passable[e])
                {
                    prev[e] = -1;
                    queue[tail++] = e;
                }
            }
        }

        bool found = false;
        while (head < tail)
        {
            int cur = queue[head++];
            if (cur == target)
            {
                found = true;
                break;
            }
            for (int ni = 0; ni < 4; ++ni)
            {
                int nb = ADJ[cur][ni];
                if (nb < 0 || prev[nb] >= -1 || !passable[nb])
                    continue;
                prev[nb] = cur;
                queue[tail++] = nb;
            }
        }

        if (!found)
            return target; // 直接给 target

        // 沿 prev 链回退, 找到 target 的前驱
        int step = target;
        while (prev[step] >= 0)
            step = prev[step];

        return step; // 从 pos 出发的第一步
    }

    static bool isAdjacent(int a, int b)
    {
        if (b < 0)
            return is_entry_block(a);
        for (int ni = 0; ni < 4; ++ni)
            if (ADJ[a][ni] == b)
                return true;
        return false;
    }

    static bool isAlreadyPlanned(const std::vector<int> &plan, int idx)
    {
        for (int p : plan)
            if (p == idx)
                return true;
        return false;
    }

    static uint8_t sceneToArmCmd(uint8_t scene)
    {
        // TODO: 抓取指令待硬件确定后映射
        (void)scene;
        return 0;
    }

    static double yawFromQuat(double qx, double qy, double qz, double qw)
    {
        const double siny_cosp = 2.0 * (qw * qz + qx * qy);
        const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    static uint8_t orientationFromQuat(double qx, double qy, double qz, double qw)
    {
        const double yaw = yawFromQuat(qx, qy, qz, qw);
        const double kPi = 3.14159265358979323846;
        const double kHalfPi = kPi * 0.5;
        const auto angleDiff = [](double a, double b)
        {
            return std::atan2(std::sin(a - b), std::cos(a - b));
        };

        const double d0 = std::fabs(angleDiff(yaw, 0.0));
        const double dPosY = std::fabs(angleDiff(yaw, kHalfPi));
        const double dNegY = std::fabs(angleDiff(yaw, -kHalfPi));
        const double dNegX = std::fabs(angleDiff(yaw, kPi));

        uint8_t state = 0; // +X
        double best = d0;
        if (dPosY < best)
        {
            best = dPosY;
            state = 1; // +Y
        }
        if (dNegY < best)
        {
            best = dNegY;
            state = 2; // -Y
        }
        if (dNegX < best)
        {
            state = 3; // -X
        }
        return state;
    }

    // ═════════════════════════════════════════════════════════════
    // 导航 action 封装
    // ═════════════════════════════════════════════════════════════

    template <typename ActionT>
    bool waitActionServer(
        const typename rclcpp_action::Client<ActionT>::SharedPtr &client,
        const std::string &action_name)
    {
        if (client->wait_for_action_server(1s))
            return true;
        RCLCPP_WARN(get_logger(), "Action server not available: %s", action_name.c_str());
        return false;
    }

    void sendNavigateWithQuat(double x, double y, double z,
                              double qx, double qy, double qz, double qw,
                              const CompletionCb &on_done)
    {
        nav_chain_in_progress_ = true;

        if (!waitActionServer<NavigateToPose>(nav_to_pose_client_, "navigate_to_pose"))
        {
            nav_chain_in_progress_ = false;
            on_done(false);
            return;
        }

        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = nav_frame_id_;
        goal.pose.header.stamp = this->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.position.z = z;
        goal.pose.pose.orientation.x = qx;
        goal.pose.pose.orientation.y = qy;
        goal.pose.pose.orientation.z = qz;
        goal.pose.pose.orientation.w = qw;

        RCLCPP_INFO(get_logger(), "NAV→ (%.2f,%.2f,%.2f) q=(%.3f,%.3f,%.3f,%.3f)", x, y, z, qx, qy, qz, qw);

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.goal_response_callback = [this](std::shared_ptr<GoalHandleNavigateToPose> gh)
        {
            RCLCPP_INFO(get_logger(), "NavigateToPose %s", gh ? "ACCEPTED" : "REJECTED");
        };
        options.result_callback = [this, on_done, qx, qy, qz, qw](const GoalHandleNavigateToPose::WrappedResult &r)
        {
            bool ok = (r.code == rclcpp_action::ResultCode::SUCCEEDED);
            RCLCPP_INFO(get_logger(), "NAV done: %s", ok ? "OK" : "FAIL");
            nav_chain_in_progress_ = false;
            if (ok)
                orientation_state_ = orientationFromQuat(qx, qy, qz, qw);
            on_done(ok);
        };
        nav_to_pose_client_->async_send_goal(goal, options);
    }

    // ═════════════════════════════════════════════════════════════
    // 上位机↔下位机 可靠指令
    // ═════════════════════════════════════════════════════════════

    void startReliableUpperCommand(uint8_t cmd)
    {
        // 硬件要求: 先发 IDLE(0) 再发目标指令
        if (cmd != 0)
        {
            publishCmd(0);
        }

        pending_upper_cmd_ = cmd;
        waiting_upper_ack_ = true;

        const auto now_time = now();
        upper_cmd_start_time_ = now_time;
        last_upper_send_time_ = now_time;

        publishCmd(cmd);
        RCLCPP_INFO(get_logger(), "ARM→ cmd=%d (等待ACK...)", cmd);
    }

    void handleUpperCommandReliability()
    {
        const auto now_time = now();

        if (waiting_upper_ack_)
        {
            if ((now_time - last_upper_send_time_).nanoseconds() >= kUpperCommandResendPeriodMs * 1000000)
            {
                publishCmd(pending_upper_cmd_);
                last_upper_send_time_ = now_time;
            }
            if ((now_time - upper_cmd_start_time_).nanoseconds() >= kUpperCommandTimeoutMs * 1000000)
            {
                RCLCPP_WARN(get_logger(), "ARM cmd %d ACK timeout, resending...", pending_upper_cmd_);
                upper_cmd_start_time_ = now_time;
            }
            return;
        }

        if (!stair_timer_ && !grab_timer_ && !entry_grab_timer_ && !zone2_grab_timer_ &&
            (now_time - last_idle_heartbeat_time_).nanoseconds() >= kIdleHeartbeatPeriodMs * 1000000)
        {
            publishCmd(0);
            last_idle_heartbeat_time_ = now_time;
        }
    }

    // ═════════════════════════════════════════════════════════════
    // 上/下台阶: 10Hz 持续发布, 等待 down_juece 完成
    // ═════════════════════════════════════════════════════════════

    void startStairPublishing(uint8_t target_cmd, StairContext context = StairContext::NORMAL)
    {
        stair_target_cmd_ = target_cmd;
        stair_context_ = context;
        stair_phase_ = 0;
        stair_waiting_done_ = true;
        stair_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&R2DecisionNode::tickStair, this));
    }

    void tickStair()
    {
        if (stair_phase_ == 0)
        {
            publishCmd(0);
            stair_phase_ = 1;
        }
        else
        {
            publishCmd(stair_target_cmd_);
        }
    }

    void stopStairPublishing()
    {
        if (stair_timer_)
        {
            stair_timer_->cancel();
            stair_timer_.reset();
        }
        stair_waiting_done_ = false;
        publishCmd(0);
    }

    void startEntryGrabPublish(uint8_t is_finsh)
    {
        stopEntryGrabPublish();
        entry_grab_is_finsh_ = is_finsh;
        grab_context_ = GrabContext::ENTRY;
        entry_grab_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&R2DecisionNode::tickEntryGrab, this));
    }

    void tickEntryGrab()
    {
        publishCmd(0, entry_grab_is_finsh_);
    }

    void stopEntryGrabPublish()
    {
        if (entry_grab_timer_)
        {
            entry_grab_timer_->cancel();
            entry_grab_timer_.reset();
        }
        publishCmd(0, 0);
    }

    void startZone2GrabPublish(uint8_t is_finsh)
    {
        stopZone2GrabPublish();
        zone2_grab_is_finsh_ = is_finsh;
        grab_context_ = GrabContext::ZONE2_FIXED;
        zone2_grab_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&R2DecisionNode::tickZone2Grab, this));
    }

    void tickZone2Grab()
    {
        publishCmd(0, zone2_grab_is_finsh_);
    }

    void stopZone2GrabPublish()
    {
        if (zone2_grab_timer_)
        {
            zone2_grab_timer_->cancel();
            zone2_grab_timer_.reset();
        }
        publishCmd(0, 0);
    }

    // Point 0 custom: NAV to (3.0,1.41) 与上台阶并发 → 右转 → 上台阶 → 转回 → 上台阶
    void handlePoint0Substep(const Zone2Task &task)
    {
        const double rx = task.rotate_x != 0.0 ? task.rotate_x : task.approach_x;
        const double ry = task.rotate_y != 0.0 ? task.rotate_y : task.approach_y;

        switch (zone2_point0_substep_)
        {
        case 0: // NAV to (3.0,1.41) + UP_STAIRS #1 并发
            RCLCPP_INFO(get_logger(), "Point0 substep 0: NAV→ (%.2f,%.2f) + UP_STAIRS #1", rx, ry);
            sendNavigateWithQuat(rx, ry, 0, 0, 0, 0, 1,
                                 [this](bool success)
                                 {
                                     if (!success)
                                         RCLCPP_WARN(get_logger(), "Point0: NAV to rotate point failed");
                                 });
            startStairPublishing(1, StairContext::POINT0);
            break;
        case 1: // rotate right (-Y), rqz=-0.707
            RCLCPP_INFO(get_logger(), "Point0 substep 1: ROTATE right (%.3f,%.3f,%.3f,%.3f)",
                        task.rqx, task.rqy, task.rqz, task.rqw);
            sendNavigateWithQuat(
                rx, ry, 0, task.rqx, task.rqy, task.rqz, task.rqw,
                [this](bool success)
                {
                    if (!success)
                        RCLCPP_WARN(get_logger(), "Point0: rotate right NAV failed");
                    if (state_ == State::ZONE2_GRAB)
                    {
                        zone2_point0_substep_ = 2;
                        transitionTo(State::ZONE2_GRAB);
                    }
                });
            break;
        case 2: // up_stairs #2
            RCLCPP_INFO(get_logger(), "Point0 substep 2: UP_STAIRS #2");
            startStairPublishing(1, StairContext::POINT0);
            break;
        case 3: // rotate back (+X), qz=0
            RCLCPP_INFO(get_logger(), "Point0 substep 3: ROTATE back (%.3f,%.3f,%.3f,%.3f)",
                        task.qx, task.qy, task.qz, task.qw);
            sendNavigateWithQuat(
                rx, ry, 0, task.qx, task.qy, task.qz, task.qw,
                [this](bool success)
                {
                    if (!success)
                        RCLCPP_WARN(get_logger(), "Point0: rotate back NAV failed");
                    if (state_ == State::ZONE2_GRAB)
                    {
                        zone2_point0_substep_ = 4;
                        transitionTo(State::ZONE2_GRAB);
                    }
                });
            break;
        case 4: // up_stairs #3
            RCLCPP_INFO(get_logger(), "Point0 substep 4: UP_STAIRS #3");
            startStairPublishing(1, StairContext::POINT0);
            break;
        case 5: // done
            RCLCPP_INFO(get_logger(), "Point0 all substeps done, advance");
            zone2_point0_sequence_active_ = false;
            zone2_point0_substep_ = 0;
            ++current_zone2_index_;
            transitionTo(State::ZONE2_NAV_POINT);
            break;
        }
    }

    void startGrabSequence(uint8_t is_finsh)
    {
        grab_is_finsh_ = is_finsh;
        grab_phase_ = 0;
        grab_phase_start_ = now();
        grab_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&R2DecisionNode::tickGrab, this));
        RCLCPP_INFO(get_logger(), "GRAB: start sequence is_finsh=%d", is_finsh);
    }

    void tickGrab()
    {
        auto elapsed = (now() - grab_phase_start_).seconds();

        if (grab_phase_ == 0)
        {
            publishCmd(0, 0);
            if (elapsed >= 0.5)
            {
                grab_phase_ = 1;
                grab_phase_start_ = now();
            }
        }
        else if (grab_phase_ == 1)
        {
            publishCmd(0, grab_is_finsh_);
            if (elapsed >= 15.0)
            {
                grab_phase_ = 2;
                grab_phase_start_ = now();
            }
        }
        else
        {
            publishCmd(0, 0);
            if (elapsed >= 0.5)
            {
                grab_timer_->cancel();
                grab_timer_.reset();
                RCLCPP_INFO(get_logger(), "GRAB: sequence done is_finsh=%d", grab_is_finsh_);
                onGrabDone();
            }
        }
    }

    void onGrabDone()
    {
        // 林内抓取完成: 导航到目标块中心
        if (state_ == State::ZONE2_GRAB)
        {
            const auto &task = zone2_tasks_[current_zone2_index_];
            RCLCPP_INFO(get_logger(), "Zone2: grab done → nav to block %d (%.2f,%.2f)",
                        task.id, task.x, task.y);
            sendNavigateWithQuat(
                task.x, task.y, task.z, 0, 0, 0, 1,
                [this](bool success)
                {
                    if (!success)
                        RCLCPP_WARN(get_logger(), "Zone2: nav to block after grab failed");
                    if (state_ != State::ZONE2_GRAB)
                        return;
                    // 到达目标块, 判断转向→台阶→下一个点
                    const auto &t = zone2_tasks_[current_zone2_index_];
                    if (t.use_rotate)
                        transitionTo(State::ZONE2_ROTATE);
                    else if (t.stair_cmd == 1)
                        transitionTo(State::ZONE2_UP_STAIRS);
                    else if (t.stair_cmd == 2)
                        transitionTo(State::ZONE2_DOWN_STAIRS);
                    else
                        transitionTo(State::ZONE2_FINISH);
                });
        }
    }

    void publishCmd(uint8_t status_bit, uint8_t is_finsh = 0)
    {
        robot_serial::msg::Juece msg;
        msg.zhuangtai = 0;
        msg.is_finsh = is_finsh;
        msg.status_bit = status_bit;
        upper_cmd_pub_->publish(msg);
    }

    // ═════════════════════════════════════════════════════════════
    // 各类 topic 回调
    // ═════════════════════════════════════════════════════════════

    void onUpperAck(const r2_interfaces::msg::ArmAck::SharedPtr msg)
    {
        if (!msg->received || !waiting_upper_ack_)
            return;
        if (msg->command != pending_upper_cmd_)
        {
            RCLCPP_WARN(get_logger(), "Ignore ACK for cmd %d, waiting %d",
                        msg->command, pending_upper_cmd_);
            return;
        }
        waiting_upper_ack_ = false;
        last_idle_heartbeat_time_ = now();
        RCLCPP_INFO(get_logger(), "ARM ACK received: cmd=%d", msg->command);
    }

    void onUpperDone(const r2_interfaces::msg::ArmDone::SharedPtr msg)
    {
        if (!msg->done)
            return;

        waiting_upper_ack_ = false;
        if (msg->command != last_arm_done_cmd_ || msg->success != last_arm_done_success_)
        {
            RCLCPP_INFO(get_logger(), "ARM DONE: cmd=%d success=%d", msg->command, msg->success);
            last_arm_done_cmd_ = msg->command;
            last_arm_done_success_ = msg->success;
        }

        // Zone1 操作完成
        if (state_ == State::ZONE1_OPERATE_POINT)
        {
            const auto it = point_table_.find(zone1_route_ids_[current_zone1_index_]);
            const bool skip_dock = (it != point_table_.end()) && it->second.skip_dock;

            if (!msg->success && zone1_arm_retry_count_ < kZone1ArmMaxRetry)
            {
                ++zone1_arm_retry_count_;
                RCLCPP_WARN(get_logger(), "Zone1 arm retry %d/%d",
                            zone1_arm_retry_count_, kZone1ArmMaxRetry);
                startReliableUpperCommand(msg->command);
                return;
            }
            if (!msg->success)
                RCLCPP_WARN(get_logger(), "Zone1 arm failed after retry, skip");

            zone1_arm_retry_count_ = 0;

            if (skip_dock)
            {
                // 非矛头点: 直接走下个点
                ++current_zone1_index_;
                transitionTo(State::ZONE1_NAV_POINT);
            }
            else
            {
                transitionTo(State::ZONE1_DOCK_R1);
            }
            return;
        }

        // Zone2 抓取完成 (仅光板路线; 固定路线由 zone2_grab_timer_ 自行管理)
        if (state_ == State::ZONE2_GRAB && !use_fixed_zone2_route_)
        {
            if (!msg->success && zone2_arm_retry_count_ < kZone2ArmMaxRetry)
            {
                ++zone2_arm_retry_count_;
                const auto &task = zone2_tasks_[current_zone2_index_];
                RCLCPP_WARN(get_logger(), "Zone2 arm retry %d/%d",
                            zone2_arm_retry_count_, kZone2ArmMaxRetry);
                startReliableUpperCommand(task.arm_command);
                return;
            }
            if (!msg->success)
                RCLCPP_WARN(get_logger(), "Zone2 arm failed after retry, skip");

            zone2_arm_retry_count_ = 0;
            ++current_zone2_index_;
            transitionTo(State::ZONE2_NAV_POINT);
            return;
        }
    }

    void onUpJuece(const robot_serial::msg::Juece::SharedPtr msg)
    {
        if (msg->zhuangtai != 1 || grab_context_ == GrabContext::NONE)
            return;

        if (grab_context_ == GrabContext::ENTRY)
        {
            grab_context_ = GrabContext::NONE;
            entry_grab_step_ = 3;
            RCLCPP_INFO(get_logger(), "EntryGrab: done (up_juece), retreat to approach");
            sendNavigateWithQuat(
                entry_approach_x_, entry_block0_y_, 0.0, 0, 0, 0, 1,
                [this](bool)
                { if (state_ == State::ZONE2_ENTRY_GRAB) transitionTo(State::ZONE2_ENTRY_GRAB); });
            return;
        }

        if (grab_context_ == GrabContext::ZONE2_FIXED)
        {
            stopZone2GrabPublish();
            grab_context_ = GrabContext::NONE;
            zone2_grab_step_ = 0;
            const auto &task = zone2_tasks_[current_zone2_index_];
            if (task.id == 0)
            {
                zone2_grab_step_ = 3;
                RCLCPP_INFO(get_logger(), "Point0 grab done, retreat to approach (%.2f,%.2f)",
                            task.approach_x, task.approach_y);
                sendNavigateWithQuat(
                    task.approach_x, task.approach_y, 0.0, task.qx, task.qy, task.qz, task.qw,
                    [this](bool)
                    { if (state_ == State::ZONE2_GRAB) transitionTo(State::ZONE2_GRAB); });
                return;
            }

            if (task.stair_cmd == 1)
                transitionTo(State::ZONE2_UP_STAIRS);
            else if (task.stair_cmd == 2)
                transitionTo(State::ZONE2_DOWN_STAIRS);
            else if (task.use_rotate)
                transitionTo(State::ZONE2_ROTATE);
            else
                transitionTo(State::ZONE2_FINISH);
        }
    }

    void onDownJuece(const robot_serial::msg::Juece::SharedPtr msg)
    {
        if (msg->zhuangtai != 1 || !stair_waiting_done_)
            return;

        stopStairPublishing();
        RCLCPP_INFO(get_logger(), "Stair done (down_juece), cmd=%d", stair_target_cmd_);

        if (stair_context_ == StairContext::POINT0)
        {
            ++zone2_point0_substep_;
            if (state_ == State::ZONE2_GRAB)
                transitionTo(State::ZONE2_GRAB);
            return;
        }

        if (state_ == State::ZONE1_UP_STAIRS)
            transitionTo(State::ZONE1_DOWN_STAIRS);
        else if (state_ == State::ZONE1_DOWN_STAIRS)
            transitionTo(State::ZONE1_FINISH);
        else if (state_ == State::ZONE2_UP_STAIRS || state_ == State::ZONE2_DOWN_STAIRS)
        {
            if (zone2_post_rotate_stairs_done_)
            {
                zone2_post_rotate_stairs_done_ = false;
                ++current_zone2_index_;
                transitionTo(State::ZONE2_NAV_POINT);
            }
            else
            {
                const auto &t = zone2_tasks_[current_zone2_index_];
                if (t.use_rotate)
                    transitionTo(State::ZONE2_ROTATE);
                else
                {
                    ++current_zone2_index_;
                    transitionTo(State::ZONE2_NAV_POINT);
                }
            }
        }
    }

    void onSpearExists(const std_msgs::msg::Bool::SharedPtr msg)
    {
        spearhead_exists_ = msg->data;
    }

    void onLightboardMap(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        lightboard_map_.assign(msg->data.begin(), msg->data.end());
        lightboard_map_received_ = true;
    }

    void onGrabSceneReady(const std_msgs::msg::Bool::SharedPtr msg)
    {
        grab_scene_ready_ = msg->data;
        if (msg->data && state_ == State::ZONE2_WAIT_SCENE_CONFIRM)
        {
            RCLCPP_INFO(get_logger(), "Scene CONFIRMED!");
            transitionTo(State::ZONE2_GRAB);
        }
    }

    void onButtonState(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        const auto now_time = now();
        if (msg->data == last_button_state_ &&
            (now_time - last_button_event_time_).nanoseconds() < kButtonDebounceMs * 1000000)
            return;

        last_button_state_ = msg->data;
        last_button_event_time_ = now_time;
        const auto button = static_cast<ButtonState>(msg->data);

        if (button == ButtonState::START)
        {
            pending_start_ = true;
        }
        if (button == ButtonState::ZONE1_RETRY)
        {
            pending_zone1_retry_ = true;
        }
        if (button == ButtonState::ZONE2_RETRY)
        {
            pending_zone2_retry_ = true;
        }
        if (button == ButtonState::ZONE3_RETRY)
        {
            pending_zone3_retry_ = true;
        }
    }

    // ═════════════════════════════════════════════════════════════
    // sensor 开关
    // ═════════════════════════════════════════════════════════════

    void publishSpearEnable(bool enable)
    {
        if (spear_camera_enabled_ == enable)
            return;
        std_msgs::msg::Bool msg;
        msg.data = enable;
        spear_enable_pub_->publish(msg);
        spear_camera_enabled_ = enable;
        RCLCPP_INFO(get_logger(), "spearhead camera %s", enable ? "ON" : "OFF");
    }

    void publishLightboardEnable(bool enable)
    {
        if (lightboard_enabled_ == enable)
            return;
        std_msgs::msg::Bool msg;
        msg.data = enable;
        lightboard_enable_pub_->publish(msg);
        lightboard_enabled_ = enable;
        RCLCPP_INFO(get_logger(), "lightboard camera %s", enable ? "ON" : "OFF");
    }

    void publishGrabSceneEnable(bool enable)
    {
        if (grab_scene_enabled_ == enable)
            return;
        std_msgs::msg::Bool msg;
        msg.data = enable;
        grab_scene_enable_pub_->publish(msg);
        grab_scene_enabled_ = enable;
        RCLCPP_INFO(get_logger(), "grab_scene %s", enable ? "ON" : "OFF");
    }

    void publishGrabSceneExpected(uint8_t scene)
    {
        std_msgs::msg::UInt8 msg;
        msg.data = scene;
        grab_scene_expected_pub_->publish(msg);
    }

    // ═════════════════════════════════════════════════════════════
    // 常量 & 参数
    // ═════════════════════════════════════════════════════════════

    static constexpr int64_t kUpperCommandResendPeriodMs = 100;
    static constexpr int64_t kIdleHeartbeatPeriodMs = 500;
    static constexpr int64_t kUpperCommandTimeoutMs = 1200;
    static constexpr int64_t kButtonDebounceMs = 120;
    static constexpr int kZone1ArmMaxRetry = 1;
    static constexpr int kZone2ArmMaxRetry = 1;
    static constexpr int kMaxDocks = 3;

    // ── 状态 ────────────────────────────────────────────────────
    State state_{State::INIT};

    // ── publishers ──────────────────────────────────────────────
    rclcpp::Publisher<robot_serial::msg::Juece>::SharedPtr upper_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spear_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lightboard_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grab_scene_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr grab_scene_expected_pub_;

    // ── subscriptions ───────────────────────────────────────────
    rclcpp::Subscription<r2_interfaces::msg::ArmAck>::SharedPtr upper_ack_sub_;
    rclcpp::Subscription<r2_interfaces::msg::ArmDone>::SharedPtr upper_done_sub_;
    rclcpp::Subscription<robot_serial::msg::Juece>::SharedPtr up_juece_sub_;
    rclcpp::Subscription<robot_serial::msg::Juece>::SharedPtr down_juece_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spear_exists_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr lightboard_map_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grab_scene_ready_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_state_sub_;

    // ── action clients ──────────────────────────────────────────
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

    // ── timer ───────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr timer_;

    // ── 参数值 ──────────────────────────────────────────────────
    std::string nav_frame_id_{"map"};
    uint8_t zone1_arm_command_{0};

    std::unordered_map<int, WaypointTask> point_table_;
    std::vector<int64_t> zone1_route_ids_;
    std::size_t current_zone1_index_{0};

    double dock_r1_x_{0.0}, dock_r1_y_{0.0}, dock_r1_z_{0.0};
    double dock_timeout_s_{15.0};
    double zone1_max_time_s_{120.0};
    double scene_confirm_timeout_s_{5.0};

    Zone2BlockInfo zone2_blocks_[12];
    bool use_fixed_zone2_route_{true};
    int zone2_fixed_count_{6};
    Zone2FixedPoint zone2_fixed_[kMaxZone2FixedPoints];
    std::vector<Zone2Task> zone2_tasks_;
    std::size_t current_zone2_index_{0};
    int zone2_arm_retry_count_{0};

    double mf_exit_x_{3.2}, mf_exit_y_{0.0}, mf_exit_z_{0.0};

    // ── 状态标志 ────────────────────────────────────────────────
    bool sim_mode_{false};

    bool spearhead_exists_{false};
    bool spear_camera_enabled_{false};

    bool lightboard_enabled_{false};
    std::vector<uint8_t> lightboard_map_;
    std::vector<uint8_t> latest_lightboard_map_;
    bool lightboard_map_received_{false};

    bool grab_scene_enabled_{false};
    bool grab_scene_ready_{false};

    bool nav_chain_in_progress_{false};
    uint8_t orientation_state_{0}; // 0:+X, 1:+Y, 2:-Y, 3:-X

    // Zone1 dock
    bool dock_arrived_{false};
    bool spearhead_was_present_{false};
    int dock_success_count_{0};
    rclcpp::Time dock_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time zone1_start_time_{0, 0, RCL_ROS_TIME};

    // Zone2 scene
    rclcpp::Time scene_confirm_start_time_{0, 0, RCL_ROS_TIME};

    // buttons
    bool pending_start_{false};
    bool pending_zone1_retry_{false};
    bool pending_zone2_retry_{false};
    bool pending_zone3_retry_{false};
    uint8_t last_button_state_{static_cast<uint8_t>(ButtonState::NONE)};
    rclcpp::Time last_button_event_time_{0, 0, RCL_ROS_TIME};

    // upper command reliability
    int zone1_arm_retry_count_{0};
    bool waiting_upper_ack_{false};
    uint8_t pending_upper_cmd_{0};
    uint8_t last_arm_done_cmd_{0xFF};
    bool last_arm_done_success_{true};
    rclcpp::Time last_upper_send_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time upper_cmd_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_idle_heartbeat_time_{0, 0, RCL_ROS_TIME};

    // stair sequence (10Hz publish, wait down_juece)
    rclcpp::TimerBase::SharedPtr stair_timer_;
    int stair_phase_{0};
    uint8_t stair_target_cmd_{0};
    bool stair_waiting_done_{false};
    StairContext stair_context_{StairContext::NORMAL};

    // grab sequence (legacy timed flow; currently unused)
    rclcpp::TimerBase::SharedPtr grab_timer_;
    int grab_phase_{0};
    rclcpp::Time grab_phase_start_;
    uint8_t grab_is_finsh_{0};

    // entry grab (block0 at col0 only; block2 moved to zone2_fixed_0)
    int entry_grab_step_{0}; // 0=nav approach, 1=forward+grab, 2=wait done, 3=retreat
    rclcpp::TimerBase::SharedPtr entry_grab_timer_;
    uint8_t entry_grab_is_finsh_{0};
    uint8_t entry_block0_is_finsh_{2};
    double entry_approach_x_{1.6};
    double entry_block0_x_{2.0}, entry_block0_y_{0.289};
    double entry_block2_x_{3.0}, entry_block2_y_{1.41};
    int zone2_grab_step_{0}; // 0=backoff, 1=start grab, 2=wait done
    rclcpp::TimerBase::SharedPtr zone2_grab_timer_;
    uint8_t zone2_grab_is_finsh_{0};
    double zone2_fixed_backoff_{0.1};
    bool zone2_post_rotate_stairs_done_{false}; // 旋转后再上台阶标记, 防止死循环
    int zone2_point0_substep_{0};               // 点0自定义: 0=上台阶1, 1=右转, 2=上台阶2, 3=左转, 4=上台阶3, 5=完成
    bool zone2_point0_sequence_active_{false};
    GrabContext grab_context_{GrabContext::NONE};
    uint8_t entry_block2_is_finsh_{1};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<R2DecisionNode>());
    rclcpp::shutdown();
    return 0;
}
