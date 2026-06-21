#include "r2_decision/r2_decision_node.hpp"

#include <cmath>

// ==========================================================================
// ActionDispatcher constructor
// ==========================================================================

ActionDispatcher::ActionDispatcher(rclcpp::Node &node) : node_(node)
{
    upper_cmd_pub_ = node_.create_publisher<robot_serial::msg::Juece>("/juece", rclcpp::SensorDataQoS());
    spear_enable_pub_ = node_.create_publisher<std_msgs::msg::Bool>("spearhead/enable", 10);
    lightboard_enable_pub_ = node_.create_publisher<std_msgs::msg::Bool>("lightboard/enable", 10);
    grab_scene_enable_pub_ = node_.create_publisher<std_msgs::msg::Bool>("grab_scene/enable", 10);
    grab_scene_expected_pub_ = node_.create_publisher<std_msgs::msg::UInt8>("grab_scene/expected_scene", 10);
    cmd_vel_pub_ = node_.create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(&node_, "navigate_to_pose");
}

// ==========================================================================
// Navigation
// ==========================================================================

template <typename ActionT>
bool ActionDispatcher::waitActionServer(
    const typename rclcpp_action::Client<ActionT>::SharedPtr &client,
    const std::string &action_name)
{
    if (client->wait_for_action_server(std::chrono::seconds(1)))
        return true;
    RCLCPP_WARN(rclcpp::get_logger("actions"), "Action server not available: %s", action_name.c_str());
    return false;
}

void ActionDispatcher::sendNavigateWithQuat(double x, double y, double z,
                                            double qx, double qy, double qz, double qw,
                                            Context &ctx)
{
    ctx.nav_chain_in_progress = true;
    ctx.current_x = x;
    ctx.current_y = y;

    if (!waitActionServer<NavigateToPose>(nav_to_pose_client_, "navigate_to_pose"))
    {
        ctx.nav_chain_in_progress = false;
        postEvent({EventType::NAV_DONE, false});
        return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = ctx.nav_frame_id;
    goal.pose.header.stamp = node_.now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = z;
    goal.pose.pose.orientation.x = qx;
    goal.pose.pose.orientation.y = qy;
    goal.pose.pose.orientation.z = qz;
    goal.pose.pose.orientation.w = qw;

    RCLCPP_INFO(rclcpp::get_logger("actions"), "NAV to (%.2f,%.2f,%.2f) q=(%.3f,%.3f,%.3f,%.3f)",
                x, y, z, qx, qy, qz, qw);

    auto *nav_flag = &ctx.nav_chain_in_progress;
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback = [](std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> gh)
    {
        RCLCPP_INFO(rclcpp::get_logger("actions"), "NavigateToPose %s", gh ? "ACCEPTED" : "REJECTED");
    };
    options.result_callback = [this, nav_flag](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &r)
    {
        bool ok = (r.code == rclcpp_action::ResultCode::SUCCEEDED);
        *nav_flag = false;
        RCLCPP_INFO(rclcpp::get_logger("actions"), "NAV done: %s", ok ? "OK" : "FAIL");
        postEvent({EventType::NAV_DONE, ok});
    };
    nav_to_pose_client_->async_send_goal(goal, options);
}

// ==========================================================================
// Arm command
// ==========================================================================

void ActionDispatcher::sendArmCommand(uint8_t cmd)
{
    if (cmd != 0)
        publishCmdWithArea(0); // brief idle before command

    pending_upper_cmd_ = cmd;
    waiting_upper_ack_ = true;
    hold_cmd_ = cmd;  // ACK→DONE 期间心跳维持此指令, 防止回退到 zhuangtai=0

    auto now_time = node_.now();
    upper_cmd_start_time_ = now_time;
    last_upper_send_time_ = now_time;

    publishCmdWithArea(cmd);
    RCLCPP_INFO(rclcpp::get_logger("actions"), "ARM cmd=%d (waiting ACK...)", cmd);
}

void ActionDispatcher::handleAck(uint8_t command)
{
    // 未在等待 ACK 时到达的 ACK, 通常是 hold 心跳重发带回的回声, 静默丢弃不刷屏.
    if (!waiting_upper_ack_)
    {
        if (command != pending_upper_cmd_)
            RCLCPP_DEBUG(rclcpp::get_logger("actions"),
                         "Ignore ACK for cmd %d, not waiting", command);
        return;
    }
    if (command != pending_upper_cmd_)
    {
        RCLCPP_WARN(rclcpp::get_logger("actions"), "Ignore ACK for cmd %d, waiting %d",
                    command, pending_upper_cmd_);
        return;
    }
    waiting_upper_ack_ = false;
    last_idle_heartbeat_time_ = node_.now();
    RCLCPP_INFO(rclcpp::get_logger("actions"), "ARM ACK received: cmd=%d", command);
}

void ActionDispatcher::handleArmDone(uint8_t command, bool success)
{
    waiting_upper_ack_ = false;
    hold_cmd_ = 0;  // 清除 sendArmCommand 设的 hold, 允许心跳回 idle
    if (command != last_arm_done_cmd_ || success != last_arm_done_success_)
    {
        RCLCPP_INFO(rclcpp::get_logger("actions"), "ARM DONE: cmd=%d success=%d", command, success);
        last_arm_done_cmd_ = command;
        last_arm_done_success_ = success;
    }
}

// ==========================================================================
// Spearhead command (zhuangtai field)
// ==========================================================================

void ActionDispatcher::sendSpearheadCommand(uint8_t cmd)
{
    // 清理旧状态, 防止心跳在 DONE 后发 zhuangtai=0
    spearhead_active_ = false;
    waiting_spearhead_ack_ = false;

    pending_spearhead_cmd_ = cmd;
    waiting_spearhead_ack_ = true;
    spearhead_active_ = true;  // 直到 DONE 前一直有效, 用于回调路由
    spearhead_acked_ = false;        // 还没收到对应 ACK (下位机送达应答)
    spearhead_retry_count_ = 1;      // 首次发送也算一次

    auto now_time = node_.now();
    spearhead_cmd_start_time_ = now_time;
    last_spearhead_send_time_ = now_time;

    publishCmdWithArea(0, 0, cmd);  // status_bit=0, is_finsh=0, zhuangtai=cmd
    RCLCPP_INFO(rclcpp::get_logger("actions"), "SPEARHEAD cmd zhuangtai=%d (waiting ACK...)", cmd);
}

void ActionDispatcher::handleSpearheadAck(uint8_t command)
{
    // ACK 只表示"下位机收到了这条指令". 注意: 下位机忙时(上条还在跑)收到新指令,
    // 虽然会"忽略"但仍会回 ACK, 所以 ACK 不能证明它真的开始执行.
    // 这里 ACK 只用于停止 ACK-级快速重发; 真正完成要等 DONE.
    if (!spearhead_active_ || command != pending_spearhead_cmd_)
    {
        // hold 期间心跳重发同一指令会不断带回 ACK, 这是正常的, 不刷屏
        return;
    }
    if (waiting_spearhead_ack_)
    {
        waiting_spearhead_ack_ = false;
        last_idle_heartbeat_time_ = node_.now();
        RCLCPP_INFO(rclcpp::get_logger("actions"), "SPEARHEAD ACK received: cmd=%d", command);
    }
    spearhead_acked_ = true;
    // 收到 ACK 后重置"等待 DONE"计时: 下位机已开始执行这条指令
    spearhead_cmd_start_time_ = node_.now();
}

bool ActionDispatcher::handleSpearheadDone(uint8_t command, bool success)
{
    // 严格校验: 只有对本条 pending 指令的 DONE 才算完成.
    // 迟到的上一条 DONE (command != pending_spearhead_cmd_) 丢弃, 避免误触发 ARM_DONE.
    if (!spearhead_active_ || command != pending_spearhead_cmd_)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("actions"),
                     "Drop stale SPEARHEAD DONE: cmd=%d (pending=%d)", command, pending_spearhead_cmd_);
        return false;
    }

    waiting_spearhead_ack_ = false;
    spearhead_active_ = false;
    pending_spearhead_cmd_ = 0;
    spearhead_acked_ = false;
    spearhead_retry_count_ = 0;
    spearhead_done_pending_ = true;  // 标记: 跳过下一次心跳, 等 FSM 处理完事件

    RCLCPP_INFO(rclcpp::get_logger("actions"), "SPEARHEAD DONE: cmd=%d success=%d", command, success);
    last_spearhead_done_cmd_ = command;
    last_spearhead_done_success_ = success;
    return true;  // 上层 (onUpperDone) 据此 postEvent(ARM_DONE)
}

// ==========================================================================
// Tick (called from node at 20ms)
// ==========================================================================

void ActionDispatcher::tick(Context &ctx)
{
    area_ = ctx.area;
    tickReliability();
}

void ActionDispatcher::tickReliability()
{
    auto now_time = node_.now();

    // spearhead 指令可靠性: ACK 超时快速重发 + DONE 超时重发有限次再跳
    if (spearhead_active_ && pending_spearhead_cmd_ != 0)
    {
        // 阶段1: 还没收到 ACK → 每 kUpperCommandResendPeriodMs 重发, 有限次后放弃 (防下位机无响应死等)
        if (waiting_spearhead_ack_)
        {
            if ((now_time - last_spearhead_send_time_).nanoseconds() >= kUpperCommandResendPeriodMs * 1'000'000)
            {
                if (spearhead_retry_count_ >= kSpearheadMaxRetry)
                {
                    RCLCPP_WARN(rclcpp::get_logger("actions"),
                                "SPEARHEAD cmd %d no ACK after %d attempts, give up",
                                pending_spearhead_cmd_, kSpearheadMaxRetry);
                    uint8_t giveup_cmd = pending_spearhead_cmd_;
                    spearhead_active_ = false;
                    waiting_spearhead_ack_ = false;
                    pending_spearhead_cmd_ = 0;
                    spearhead_acked_ = false;
                    spearhead_retry_count_ = 0;
                    spearhead_done_pending_ = true;
                    last_spearhead_done_cmd_ = giveup_cmd;
                    last_spearhead_done_success_ = false;
                    postEvent({EventType::ARM_DONE, false});
                    return;
                }
                ++spearhead_retry_count_;
                publishCmdWithArea(0, 0, pending_spearhead_cmd_);
                last_spearhead_send_time_ = now_time;
                spearhead_cmd_start_time_ = now_time;
            }
        }
        // 阶段2: 收到 ACK 但等不到 DONE → 超过 kSpearheadDoneTimeoutMs 重发, 有限次后再跳过
        else if (spearhead_acked_ &&
                 (now_time - spearhead_cmd_start_time_).nanoseconds() >= kSpearheadDoneTimeoutMs * 1'000'000)
        {
            if (spearhead_retry_count_ < kSpearheadMaxRetry)
            {
                ++spearhead_retry_count_;
                RCLCPP_WARN(rclcpp::get_logger("actions"),
                            "SPEARHEAD cmd %d DONE timeout, resend (%d/%d)",
                            pending_spearhead_cmd_, spearhead_retry_count_, kSpearheadMaxRetry);
                publishCmdWithArea(0, 0, pending_spearhead_cmd_);
                waiting_spearhead_ack_ = true;
                spearhead_acked_ = false;
                last_spearhead_send_time_ = now_time;
                spearhead_cmd_start_time_ = now_time;
            }
            else
            {
                // 重试耗尽: 当作失败交给 FSM 跳过
                RCLCPP_WARN(rclcpp::get_logger("actions"),
                            "SPEARHEAD cmd %d DONE timeout after %d retries, give up",
                            pending_spearhead_cmd_, kSpearheadMaxRetry);
                uint8_t giveup_cmd = pending_spearhead_cmd_;
                spearhead_active_ = false;
                waiting_spearhead_ack_ = false;
                pending_spearhead_cmd_ = 0;
                spearhead_acked_ = false;
                spearhead_retry_count_ = 0;
                spearhead_done_pending_ = true;
                last_spearhead_done_cmd_ = giveup_cmd;
                last_spearhead_done_success_ = false;
                postEvent({EventType::ARM_DONE, false});
            }
        }
        return;
    }

    if (waiting_upper_ack_)
    {
        if ((now_time - last_upper_send_time_).nanoseconds() >= kUpperCommandResendPeriodMs * 1'000'000)
        {
            publishCmdWithArea(pending_upper_cmd_);
            last_upper_send_time_ = now_time;
        }
        if ((now_time - upper_cmd_start_time_).nanoseconds() >= kUpperCommandTimeoutMs * 1'000'000)
        {
            RCLCPP_WARN(rclcpp::get_logger("actions"), "ARM cmd %d ACK timeout, resending...", pending_upper_cmd_);
            upper_cmd_start_time_ = now_time;
        }
        return;
    }

    // idle heartbeat — removed
}

// ==========================================================================
// Stair — 发1后500ms发0
// ==========================================================================

void ActionDispatcher::startStair(uint8_t target_cmd, Context &ctx, StairContext sc)
{
    stopStair();
    stair_target_cmd_ = target_cmd;
    stair_context_ = sc;
    stair_active_ = true;

    publishCmdWithArea(stair_target_cmd_);

    stair_timer_ = node_.create_wall_timer(
        std::chrono::milliseconds(500),
        [this] {
            if (stair_active_)
            {
                publishCmdWithArea(0);
                stair_active_ = false;
            }
            if (stair_timer_)
            {
                stair_timer_->cancel();
                stair_timer_.reset();
            }
        });

    (void)ctx;
}

void ActionDispatcher::stopStair()
{
    if (stair_timer_)
    {
        stair_timer_->cancel();
        stair_timer_.reset();
    }
    if (stair_active_)
    {
        publishCmdWithArea(0);
    }
    stair_active_ = false;
}

// ==========================================================================
// Entry grab — 单次发送
// ==========================================================================

void ActionDispatcher::startEntryGrab(uint8_t is_finsh, Context &ctx)
{
    stopEntryGrab();
    ctx.grab_context = GrabContext::ENTRY;

    // 单次发送 is_finsh
    publishCmdWithArea(0, is_finsh);
}

void ActionDispatcher::stopEntryGrab()
{
    publishCmdWithArea(0, 0);
}

// ==========================================================================
// Zone2 grab — 单次发送
// ==========================================================================

void ActionDispatcher::startZone2Grab(uint8_t is_finsh, Context &ctx)
{
    stopZone2Grab();
    ctx.grab_context = GrabContext::ZONE2_FIXED;

    // 单次发送 is_finsh
    publishCmdWithArea(0, is_finsh);
}

void ActionDispatcher::stopZone2Grab()
{
    publishCmdWithArea(0, 0);
}

// ==========================================================================
// Sensor enable
// ==========================================================================

void ActionDispatcher::enableSpear(bool enable)
{
    if (spear_camera_enabled_ == enable) return;
    std_msgs::msg::Bool msg;
    msg.data = enable;
    spear_enable_pub_->publish(msg);
    spear_camera_enabled_ = enable;
    RCLCPP_INFO(rclcpp::get_logger("actions"), "spearhead camera %s", enable ? "ON" : "OFF");
}

void ActionDispatcher::enableLightboard(bool enable)
{
    if (lightboard_enabled_ == enable) return;
    std_msgs::msg::Bool msg;
    msg.data = enable;
    lightboard_enable_pub_->publish(msg);
    lightboard_enabled_ = enable;
    RCLCPP_INFO(rclcpp::get_logger("actions"), "lightboard camera %s", enable ? "ON" : "OFF");
}

void ActionDispatcher::enableGrabScene(bool enable, uint8_t expected_scene)
{
    if (grab_scene_enabled_ == enable) return;
    std_msgs::msg::Bool msg;
    msg.data = enable;
    grab_scene_enable_pub_->publish(msg);
    grab_scene_enabled_ = enable;

    if (enable)
    {
        std_msgs::msg::UInt8 smsg;
        smsg.data = expected_scene;
        grab_scene_expected_pub_->publish(smsg);
    }
    RCLCPP_INFO(rclcpp::get_logger("actions"), "grab_scene %s (scene=%d)", enable ? "ON" : "OFF", expected_scene);
}

// ==========================================================================
// Low-level publish
// ==========================================================================

void ActionDispatcher::publishCmd(uint8_t status_bit, uint8_t is_finsh, uint8_t zhuangtai, uint8_t area)
{
    robot_serial::msg::Juece msg;
    msg.zhuangtai = zhuangtai;
    msg.is_finsh = is_finsh;
    msg.status_bit = status_bit;
    msg.area = area;
    upper_cmd_pub_->publish(msg);
}

// 自动带 area_ 的版本
void ActionDispatcher::publishCmdWithArea(uint8_t status_bit, uint8_t is_finsh, uint8_t zhuangtai)
{
    publishCmd(status_bit, is_finsh, zhuangtai, area_);
}

// ==========================================================================
// cmd_vel (visual align)
// ==========================================================================

void ActionDispatcher::publishCmdVel(double linear_x, double linear_y, double angular_z)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.linear.y = linear_y;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = angular_z;
    cmd_vel_pub_->publish(msg);
}

void ActionDispatcher::stopCmdVel()
{
    geometry_msgs::msg::Twist msg;  // all zeros
    cmd_vel_pub_->publish(msg);
}

// ==========================================================================
// Utilities
// ==========================================================================

double ActionDispatcher::yawFromQuat(double qx, double qy, double qz, double qw)
{
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}
