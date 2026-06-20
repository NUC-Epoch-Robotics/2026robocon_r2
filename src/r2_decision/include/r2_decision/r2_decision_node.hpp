#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "robot_serial/msg/juece.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

// ==========================================================================
// Enums
// ==========================================================================

enum class EventType : uint8_t
{
    START_PRESSED,
    ZONE1_RETRY,
    ZONE2_RETRY,
    ZONE3_RETRY,
    NAV_DONE,
    ARM_DONE,
    UP_JUECE_DONE,
    DOWN_JUECE_DONE,
    SCENE_READY,
    DOCK_SUCCESS,
    DOCK_TIMEOUT,
    ZONE1_TIMEOUT,
    SCENE_CONFIRM_TIMEOUT,
    VISION_ALIGN_DONE,
};

struct Event
{
    EventType type;
    bool success{false};
    uint8_t command{0};
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

// ==========================================================================
// Task / config data structures
// ==========================================================================

struct WaypointTask
{
    int id;
    double x, y, z;
    uint8_t arm_command{0};
    bool skip_dock{false};
    bool use_spearhead{false};  // true → sendSpearheadCommand, false → sendArmCommand
    uint8_t docking_cmd{0};    // 矛头对接指令: 2=五号矛头, 3=四号矛头, 0=无
};

struct Zone2Task
{
    int id;
    double x, y, z;
    double qx{0}, qy{0}, qz{0}, qw{1};
    bool use_rotate{false};
    double rqx{0}, rqy{0}, rqz{0}, rqw{1};
    double approach_x{0}, approach_y{0};
    uint8_t block_height{0}, stand_height{0};
    uint8_t grab_scene{0}, arm_command{0};
    int8_t stair_cmd{0};
    uint8_t grab_is_finsh{0};
    double rotate_x{0}, rotate_y{0};
    int grab_adjacent_block{-1};  // >=0: 在当前格转向拿相邻格的KFS，不移动
};

struct Zone2BlockInfo
{
    double x, y, z;
    uint8_t grab_scene;
};

struct Zone2FixedPoint
{
    double x, y, z;
    double qx{0}, qy{0}, qz{0}, qw{1};
    bool use_rotate{false};
    double rqx{0}, rqy{0}, rqz{0}, rqw{1};
    double approach_x{0}, approach_y{0};
    uint8_t block_height{0}, stand_height{0};
    int8_t stair_cmd{0};
    double rotate_x{0}, rotate_y{0};
};

constexpr int kMaxZone2FixedPoints = 8;
constexpr int kMaxDocks = 3;
constexpr int kZone1ArmMaxRetry = 1;
constexpr int kZone2ArmMaxRetry = 1;
constexpr int64_t kUpperCommandResendPeriodMs = 100;
constexpr int64_t kIdleHeartbeatPeriodMs = 500;
constexpr int64_t kUpperCommandTimeoutMs = 1200;
constexpr int64_t kButtonDebounceMs = 120;

// ==========================================================================
// Context — all mutable state and read-only config in one place
// ==========================================================================

struct Context
{
    // ---- config (read-only after init) ----
    std::string nav_frame_id{"odom"};
    uint8_t zone1_arm_command{0};
    uint8_t spearhead_extend_cmd{2};   // 伸吸盘指令 (zhuangtai 字段)
    std::unordered_map<int, WaypointTask> point_table;
    std::vector<int64_t> zone1_route_ids;
    double dock_r1_x{0}, dock_r1_y{0}, dock_r1_z{0};
    double dock_timeout_s{15};
    double zone1_max_time_s{120};
    double scene_confirm_timeout_s{5};
    Zone2BlockInfo zone2_blocks[12]{};
    bool use_fixed_zone2_route{true};
    int zone2_fixed_count{6};
    Zone2FixedPoint zone2_fixed[kMaxZone2FixedPoints]{};
    double zone2_fixed_backoff{0.1};
    double mf_exit_x{3.2}, mf_exit_y{0}, mf_exit_z{0};
    double entry_approach_x{1.6};
    double entry_block0_x{2.0}, entry_block0_y{0.289};
    uint8_t entry_block0_is_finsh{2};
    double entry_block2_x{3.0}, entry_block2_y{1.41};
    uint8_t entry_block2_is_finsh{1};
    bool sim_mode{false};

    // ---- sensor mirror ----
    bool spearhead_exists{false};
    bool lightboard_map_received{false};
    uint8_t area{0};  // 串口发的区号: 0=无状态, 1=一区, 2=二区, 3=三区

    // ---- odometry mirror (from /odin1/odometry) ----
    double odom_x{0.0};
    double odom_y{0.0};
    double odom_yaw{0.0};
    bool odom_received{false};

    // ---- odometry fine-tune target point (configurable via ROS params) ----
    double fine_tune_target_x{0.0};
    double fine_tune_target_y{0.0};
    double fine_tune_target_yaw{0.0};

    // ---- odometry fine-tune parameters (bang-bang) ----
    double fine_tune_xy_threshold{0.01};     // position tolerance (m)
    double fine_tune_yaw_threshold{0.05};    // yaw tolerance (rad)
    int fine_tune_stable_required{5};        // consecutive aligned frames
    double fine_tune_timeout_s{15.0};        // timeout (s)
    double fine_tune_speed_x{0.05};          // x correction speed (m/s)
    double fine_tune_speed_y{0.05};          // y correction speed (m/s)
    double fine_tune_speed_yaw{0.2};         // yaw correction speed (rad/s)
    std::vector<uint8_t> lightboard_map;
    std::vector<uint8_t> latest_lightboard_map;
    bool grab_scene_ready{false};

    // ---- button ----
    uint8_t last_button_state{0};
    rclcpp::Time last_button_event_time{0, 0, RCL_ROS_TIME};

    // ---- navigation ----
    bool nav_chain_in_progress{false};
    double current_x{0.0};  // 上次导航目标x
    double current_y{0.0};  // 上次导航目标y

    // ---- zone1 progress ----
    size_t zone1_index{0};
    int zone1_arm_retry{0};
    int dock_success_count{0};
    bool dock_arrived{false};
    int dock_step{0};  // 0=move, 1=rotate, 2=wait
    bool spearhead_was_present{false};
    rclcpp::Time dock_start_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time zone1_start_time{0, 0, RCL_ROS_TIME};
    int spearhead_post_dock_step{0};
    rclcpp::Time spearhead_post_dock_start{0, 0, RCL_ROS_TIME};

    // ---- rotation state (ROTATE_90_CW / ROTATE_180 via cmd_vel + odom) ----
    double rotation_target_yaw{0.0};
    int rotation_stable_count{0};
    rclcpp::Time rotation_start_time{0, 0, RCL_ROS_TIME};

    // ---- odometry fine-tune state (deprecated, kept for compatibility) ----
    int fine_tune_stable_count{0};
    rclcpp::Time fine_tune_start_time{0, 0, RCL_ROS_TIME};

    // ---- wait_5s state ----
    rclcpp::Time wait_5s_start_time{0, 0, RCL_ROS_TIME};

    // ---- zone2 progress ----
    size_t zone2_index{0};
    int zone2_arm_retry{0};
    bool zone2_stair_pending{false};  // true = 等台阶完成后再旋转
    int entry_grab_step{0};
    int zone2_grab_step{0};
    int zone2_point0_substep{0};
    bool zone2_point0_sequence_active{false};
    bool point0_nav_sent{false};
    GrabContext grab_context{GrabContext::NONE};
    bool zone2_post_rotate_stairs_done{false};
    bool zone2_post_grab_nav_pending{false};
    rclcpp::Time scene_confirm_start_time{0, 0, RCL_ROS_TIME};

    // ---- task data (computed by decision tree) ----
    std::vector<Zone2Task> zone2_tasks;
};

// ==========================================================================
// Forward declarations
// ==========================================================================

class ActionDispatcher;
class TopState;

// ==========================================================================
// ActionDispatcher — all ROS I/O, timers, and reliability logic
// ==========================================================================

class ActionDispatcher
{
public:
    using EventSink = std::function<void(Event)>;

    explicit ActionDispatcher(rclcpp::Node &node);

    void setEventSink(EventSink sink) { post_event_ = std::move(sink); }
    void postEvent(const Event &e) { if (post_event_) post_event_(e); }

    // --- navigation ---
    void sendNavigateWithQuat(double x, double y, double z,
                              double qx, double qy, double qz, double qw,
                              Context &ctx);

    // --- arm command ---
    void sendArmCommand(uint8_t cmd);
    void handleAck(uint8_t command);
    void handleArmDone(uint8_t command, bool success);
    bool isWaitingAck() const { return waiting_upper_ack_; }

    // --- spearhead command (uses zhuangtai field) ---
    void sendSpearheadCommand(uint8_t cmd);
    void handleSpearheadAck(uint8_t command);
    void handleSpearheadDone(uint8_t command, bool success);
    bool isWaitingSpearheadAck() const { return waiting_spearhead_ack_; }
    bool hasPendingSpearhead() const { return spearhead_active_; }

    // --- stair ---
    void startStair(uint8_t target_cmd, Context &ctx, StairContext sc = StairContext::NORMAL);
    void stopStair();
    bool isStairActive() const { return stair_timer_ != nullptr; }

    // --- grab ---
    void startEntryGrab(uint8_t is_finsh, Context &ctx);
    void stopEntryGrab();
    void startZone2Grab(uint8_t is_finsh, Context &ctx);
    void stopZone2Grab();

    // --- sensor enable ---
    void enableSpear(bool enable);
    void enableLightboard(bool enable);
    void enableGrabScene(bool enable, uint8_t expected_scene = 0);

    // --- low-level publish ---
    void publishCmd(uint8_t status_bit, uint8_t is_finsh = 0, uint8_t zhuangtai = 0, uint8_t area = 0);

    // --- cmd_vel (odometry fine-tune) ---
    void publishCmdVel(double linear_x, double linear_y, double angular_z = 0.0);
    void stopCmdVel();

    // --- called each tick ---
    void tick(Context &ctx);

    // --- utils ---
    static double yawFromQuat(double qx, double qy, double qz, double qw);

private:
    template <typename ActionT>
    bool waitActionServer(const typename rclcpp_action::Client<ActionT>::SharedPtr &client,
                          const std::string &action_name);

    void tickReliability();
    void tickStair();
    void tickEntryGrab();
    void tickZone2Grab();

    rclcpp::Node &node_;
    EventSink post_event_;

    // publishers
    rclcpp::Publisher<robot_serial::msg::Juece>::SharedPtr upper_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spear_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lightboard_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grab_scene_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr grab_scene_expected_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

    // timers
    rclcpp::TimerBase::SharedPtr stair_timer_;
    rclcpp::TimerBase::SharedPtr entry_grab_timer_;
    rclcpp::TimerBase::SharedPtr zone2_grab_timer_;

    // arm reliability
    bool waiting_upper_ack_{false};
    uint8_t pending_upper_cmd_{0};
    uint8_t last_arm_done_cmd_{0xFF};
    bool last_arm_done_success_{true};
    rclcpp::Time last_upper_send_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time upper_cmd_start_time_{0, 0, RCL_ROS_TIME};

    // spearhead reliability
    bool waiting_spearhead_ack_{false};
    bool spearhead_active_{false};  // true from sendSpearheadCommand until DONE
    uint8_t pending_spearhead_cmd_{0};
    uint8_t last_spearhead_done_cmd_{0xFF};
    bool last_spearhead_done_success_{true};
    rclcpp::Time last_spearhead_send_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time spearhead_cmd_start_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Time last_idle_heartbeat_time_{0, 0, RCL_ROS_TIME};

    // camera state
    bool spear_camera_enabled_{false};
    bool lightboard_enabled_{false};
    bool grab_scene_enabled_{false};

    // stair state
    int stair_phase_{0};
    uint8_t stair_target_cmd_{0};
    StairContext stair_context_{StairContext::NORMAL};

    // entry grab state
    uint8_t entry_grab_is_finsh_{0};

    // zone2 grab state
    uint8_t zone2_grab_is_finsh_{0};
};

// ==========================================================================
// TopState — base class for all top-level states
// ==========================================================================

class TopState
{
public:
    virtual ~TopState() = default;
    virtual std::unique_ptr<TopState> onEnter(Context &ctx, ActionDispatcher &act) { return nullptr; }
    virtual std::unique_ptr<TopState> onTick(Context &ctx, ActionDispatcher &act) { return nullptr; }
    virtual void onExit(Context &ctx, ActionDispatcher &act) {}
    virtual std::unique_ptr<TopState> handleEvent(Context &ctx, ActionDispatcher &act, const Event &e) { return nullptr; }
    virtual const char *name() const = 0;
};

// ==========================================================================
// StateMachine — owns current state, delegates tick/event
// ==========================================================================

class StateMachine
{
public:
    void tick(Context &ctx, ActionDispatcher &act);
    void handleEvent(Context &ctx, ActionDispatcher &act, const Event &e);
    void start(std::unique_ptr<TopState> initial, Context &ctx, ActionDispatcher &act);
    const char *currentName() const;

private:
    void transitionTo(std::unique_ptr<TopState> next, Context &ctx, ActionDispatcher &act);
    std::unique_ptr<TopState> current_;
};

// ==========================================================================
// Concrete top-level states
// ==========================================================================

class BootState : public TopState
{
public:
    std::unique_ptr<TopState> onEnter(Context &ctx, ActionDispatcher &act) override;
    const char *name() const override { return "Boot"; }
};

class WaitStartState : public TopState
{
public:
    std::unique_ptr<TopState> onEnter(Context &ctx, ActionDispatcher &act) override;
    std::unique_ptr<TopState> handleEvent(Context &ctx, ActionDispatcher &act, const Event &e) override;
    const char *name() const override { return "WaitStart"; }
};

class Zone1State : public TopState
{
public:
    std::unique_ptr<TopState> onEnter(Context &ctx, ActionDispatcher &act) override;
    std::unique_ptr<TopState> onTick(Context &ctx, ActionDispatcher &act) override;
    std::unique_ptr<TopState> handleEvent(Context &ctx, ActionDispatcher &act, const Event &e) override;
    void onExit(Context &ctx, ActionDispatcher &act) override;
    const char *name() const override;

private:
    enum class Sub : uint8_t
    {
        EXTEND_SUCTION,     // 开局伸吸盘
        NAV_POINT,          // 第一段: 变x (全局坐标系)
        ROTATE_90_CW,       // 顺时针转90度 (Nav2 navigate_to_pose)
        NAV_POINT_Y,        // 第二段: 变y (全局坐标系)
        OPERATE,            // 抓矛头 (is_finsh=1)
        ROTATE_180,         // 转180度 (Nav2 navigate_to_pose)
        DOCKING,            // 矛头对接 (is_finsh=2/3)
        WAIT_5S,            // 等5秒, 然后发 is_finsh=0 复位
        FINISH,
    };
    Sub sub_{Sub::EXTEND_SUCTION};

    void enterSub(Context &ctx, ActionDispatcher &act);
    std::unique_ptr<TopState> handleSubEvent(Context &ctx, ActionDispatcher &act, const Event &e);
    void checkTimeLimit(Context &ctx, ActionDispatcher &act);
};

class Zone2State : public TopState
{
public:
    std::unique_ptr<TopState> onEnter(Context &ctx, ActionDispatcher &act) override;
    std::unique_ptr<TopState> onTick(Context &ctx, ActionDispatcher &act) override;
    std::unique_ptr<TopState> handleEvent(Context &ctx, ActionDispatcher &act, const Event &e) override;
    void onExit(Context &ctx, ActionDispatcher &act) override;
    const char *name() const override;

private:
    enum class Sub : uint8_t
    {
        ENTRY_GRAB,
        NAV_POINT,
        ROTATE,
        ROTATE_GRAB,    // 动态路线: 转向拿相邻格KFS
        WAIT_SCENE,
        GRAB,
        UP_STAIRS,
        DOWN_STAIRS,
        FINISH,
    };
    Sub sub_{Sub::ENTRY_GRAB};

    void enterSub(Context &ctx, ActionDispatcher &act);
    std::unique_ptr<TopState> handleSubEvent(Context &ctx, ActionDispatcher &act, const Event &e);
    void tickEntryGrab(Context &ctx, ActionDispatcher &act);
    void tickGrab(Context &ctx, ActionDispatcher &act);
    void handlePoint0Substep(Context &ctx, ActionDispatcher &act);
    void checkSceneTimeout(Context &ctx, ActionDispatcher &act);
};

class ExitState : public TopState
{
public:
    std::unique_ptr<TopState> onEnter(Context &ctx, ActionDispatcher &act) override;
    std::unique_ptr<TopState> handleEvent(Context &ctx, ActionDispatcher &act, const Event &e) override;
    const char *name() const override { return "Exit"; }
};

class DoneState : public TopState
{
public:
    std::unique_ptr<TopState> onEnter(Context &ctx, ActionDispatcher &act) override;
    std::unique_ptr<TopState> handleEvent(Context &ctx, ActionDispatcher &act, const Event &e) override;
    const char *name() const override { return "Done"; }
};

// ==========================================================================
// R2DecisionNode — thin ROS2 shell
// ==========================================================================

class R2DecisionNode : public rclcpp::Node
{
public:
    R2DecisionNode();

    // 决策树 (public, fsm.cpp 和 zone2_planner.cpp 都会调用)
    static void buildZone2FixedRoute(Context &ctx);
    static void buildZone2Route(Context &ctx, const std::vector<uint8_t> &lightboard_map);

private:
    void tick();
    void postEvent(EventType type, bool success = false, uint8_t command = 0);
    void processEvents();

    // callbacks (only update Context + post Event)
    void onUpperAck(const robot_serial::msg::Juece::SharedPtr msg);
    void onUpperDone(const robot_serial::msg::Juece::SharedPtr msg);
    void onUpJuece(const robot_serial::msg::Juece::SharedPtr msg);
    void onDownJuece(const robot_serial::msg::Juece::SharedPtr msg);
    void onSpearExists(const std_msgs::msg::Bool::SharedPtr msg);
    void onLightboardMap(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void onGrabSceneReady(const std_msgs::msg::Bool::SharedPtr msg);
    void onButtonState(const std_msgs::msg::UInt8::SharedPtr msg);
    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onArea(const robot_serial::msg::Juece::SharedPtr msg);

    // ROS2
    rclcpp::Subscription<robot_serial::msg::Juece>::SharedPtr upper_ack_sub_;
    rclcpp::Subscription<robot_serial::msg::Juece>::SharedPtr upper_done_sub_;
    rclcpp::Subscription<robot_serial::msg::Juece>::SharedPtr up_juece_sub_;
    rclcpp::Subscription<robot_serial::msg::Juece>::SharedPtr down_juece_sub_;
    rclcpp::Subscription<robot_serial::msg::Juece>::SharedPtr area_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spear_exists_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr lightboard_map_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grab_scene_ready_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // core
    Context ctx_;
    ActionDispatcher actions_;
    StateMachine fsm_;
    std::deque<Event> event_queue_;
};
