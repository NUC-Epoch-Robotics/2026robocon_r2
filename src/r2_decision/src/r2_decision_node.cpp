#include <cstdio>

#include "r2_decision/r2_decision_node.hpp"

using std::placeholders::_1;

R2DecisionNode::R2DecisionNode() : Node("r2_decision_node"), actions_(*this)
{
    // ── wire ActionDispatcher event sink ──────────────────────
    actions_.setEventSink([this](const Event &e) { event_queue_.push_back(e); });

    // ── subscriptions ─────────────────────────────────────────
    upper_ack_sub_ = create_subscription<robot_serial::msg::Juece>(
        "/juece_ack", 10,
        std::bind(&R2DecisionNode::onUpperAck, this, _1));

    upper_done_sub_ = create_subscription<robot_serial::msg::Juece>(
        "/juece_done", 10,
        std::bind(&R2DecisionNode::onUpperDone, this, _1));

    up_juece_sub_ = create_subscription<robot_serial::msg::Juece>(
        "up_juece", rclcpp::SensorDataQoS(),
        std::bind(&R2DecisionNode::onUpJuece, this, _1));

    down_juece_sub_ = create_subscription<robot_serial::msg::Juece>(
        "down_juece", rclcpp::SensorDataQoS(),
        std::bind(&R2DecisionNode::onDownJuece, this, _1));

    area_sub_ = create_subscription<robot_serial::msg::Juece>(
        "juece/area", 10,
        std::bind(&R2DecisionNode::onArea, this, _1));

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

    // ── odometry subscription (for fine-tune) ────────────────
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odin1/odometry", 10,
        std::bind(&R2DecisionNode::onOdometry, this, _1));

    // ── main loop ─────────────────────────────────────────────
    timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&R2DecisionNode::tick, this));

    // ── params → Context ──────────────────────────────────────
    ctx_.nav_frame_id = declare_parameter<std::string>("nav_frame_id", "odom");

    // 起点位置 (开局坐标)
    ctx_.current_x = declare_parameter<double>("start_x", -0.352);
    ctx_.current_y = declare_parameter<double>("start_y", -0.259);

    ctx_.zone1_arm_command = static_cast<uint8_t>(declare_parameter<int>("zone1_arm_command", 0));
    ctx_.spearhead_extend_cmd = static_cast<uint8_t>(declare_parameter<int>("spearhead_extend_cmd", 2));

    // 唯一要抓的矛头点 (id=5): 坐标直接来自 launch 参数, 其余矛头不抓
    double spearhead_base_x = declare_parameter<double>("spearhead_base_x", 0.0);
    double spearhead_base_y = declare_parameter<double>("spearhead_base_y", 0.0);
    double spearhead_base_z = declare_parameter<double>("spearhead_base_z", 0.0);
    ctx_.point_table[5] = {5,
                           spearhead_base_x,
                           spearhead_base_y,
                           spearhead_base_z,
                           ctx_.zone1_arm_command,
                           true};  // use_spearhead=true
    // 五号矛头对接 (is_finsh=2)
    ctx_.point_table[5].docking_cmd = 2;

    ctx_.zone1_route_ids = declare_parameter<std::vector<int64_t>>(
        "zone1_route", std::vector<int64_t>{4, 5});

    ctx_.point_table[7] = {7,
                           declare_parameter<double>("zone1_point_2_x", 0.0),
                           declare_parameter<double>("zone1_point_2_y", 0.0),
                           declare_parameter<double>("zone1_point_2_z", 0.0),
                           0, true};
    ctx_.point_table[8] = {8,
                           declare_parameter<double>("zone1_point_3_x", 0.0),
                           declare_parameter<double>("zone1_point_3_y", 0.0),
                           declare_parameter<double>("zone1_point_3_z", 0.0),
                           0, true};

    ctx_.dock_r1_x = declare_parameter<double>("dock_r1_x", 0.0);
    ctx_.dock_r1_y = declare_parameter<double>("dock_r1_y", 0.0);
    ctx_.dock_r1_z = declare_parameter<double>("dock_r1_z", 0.0);

    // Zone2 block coordinates
    for (int idx = 0; idx < 12; ++idx)
    {
        char px[32], py[32], ps[32];
        std::snprintf(px, sizeof(px), "zone2_block_%d_x", idx);
        std::snprintf(py, sizeof(py), "zone2_block_%d_y", idx);
        std::snprintf(ps, sizeof(ps), "zone2_block_%d_z", idx);
        ctx_.zone2_blocks[idx].x = declare_parameter<double>(px, 0.0);
        ctx_.zone2_blocks[idx].y = declare_parameter<double>(py, 0.0);
        ctx_.zone2_blocks[idx].z = declare_parameter<double>(ps, 0.0);
    }
    // grab_scene hardcoded
    ctx_.zone2_blocks[0].grab_scene = 2;
    ctx_.zone2_blocks[1].grab_scene = 1;
    ctx_.zone2_blocks[2].grab_scene = 2;
    ctx_.zone2_blocks[3].grab_scene = 1;
    ctx_.zone2_blocks[4].grab_scene = 2;
    ctx_.zone2_blocks[5].grab_scene = 3;
    ctx_.zone2_blocks[6].grab_scene = 2;
    ctx_.zone2_blocks[7].grab_scene = 3;
    ctx_.zone2_blocks[8].grab_scene = 2;
    ctx_.zone2_blocks[9].grab_scene = 1;
    ctx_.zone2_blocks[10].grab_scene = 2;
    ctx_.zone2_blocks[11].grab_scene = 1;

    // Timeout params
    ctx_.zone1_max_time_s = declare_parameter<double>("zone1_max_time_s", 120.0);
    ctx_.scene_confirm_timeout_s = declare_parameter<double>("scene_confirm_timeout_s", 5.0);
    ctx_.dock_timeout_s = declare_parameter<double>("dock_timeout_s", 15.0);

    // Odometry fine-tune target point (configurable, to be measured)
    ctx_.fine_tune_target_x = declare_parameter<double>("fine_tune_target_x", 0.0);
    ctx_.fine_tune_target_y = declare_parameter<double>("fine_tune_target_y", 0.0);
    ctx_.fine_tune_target_yaw = declare_parameter<double>("fine_tune_target_yaw", 0.0);

    // Odometry fine-tune params (bang-bang)
    ctx_.fine_tune_xy_threshold = declare_parameter<double>("fine_tune_xy_threshold", 0.01);
    ctx_.fine_tune_yaw_threshold = declare_parameter<double>("fine_tune_yaw_threshold", 0.05);
    ctx_.fine_tune_stable_required = declare_parameter<int>("fine_tune_stable_required", 5);
    ctx_.fine_tune_timeout_s = declare_parameter<double>("fine_tune_timeout_s", 15.0);
    ctx_.fine_tune_speed_x = declare_parameter<double>("fine_tune_speed_x", 0.05);
    ctx_.fine_tune_speed_y = declare_parameter<double>("fine_tune_speed_y", 0.05);
    ctx_.fine_tune_speed_yaw = declare_parameter<double>("fine_tune_speed_yaw", 0.2);

    // MF exit
    ctx_.mf_exit_x = declare_parameter<double>("mf_exit_x", 3.2);
    ctx_.mf_exit_y = declare_parameter<double>("mf_exit_y", 0.0);
    ctx_.mf_exit_z = declare_parameter<double>("mf_exit_z", 0.0);

    // Zone2 fixed route
    ctx_.use_fixed_zone2_route = declare_parameter<bool>("use_fixed_zone2_route", true);
    ctx_.zone2_fixed_backoff = declare_parameter<double>("zone2_fixed_backoff", 0.1);
    ctx_.zone2_fixed_count = declare_parameter<int>("zone2_fixed_count", 6);
    if (ctx_.zone2_fixed_count < 0) ctx_.zone2_fixed_count = 0;
    if (ctx_.zone2_fixed_count > kMaxZone2FixedPoints) ctx_.zone2_fixed_count = kMaxZone2FixedPoints;

    for (int i = 0; i < kMaxZone2FixedPoints; ++i)
    {
        char px[32], py[32], pyaw[32], pqx[32], pqy[32], pqz[32], pqw[32];
        char rqx[32], rqy[32], rqz[32], rqw[32], ruse[32];
        std::snprintf(px, sizeof(px), "zone2_fixed_%d_x", i);
        std::snprintf(py, sizeof(py), "zone2_fixed_%d_y", i);
        std::snprintf(pyaw, sizeof(pyaw), "zone2_fixed_%d_z", i);
        std::snprintf(pqx, sizeof(pqx), "zone2_fixed_%d_qx", i);
        std::snprintf(pqy, sizeof(pqy), "zone2_fixed_%d_qy", i);
        std::snprintf(pqz, sizeof(pqz), "zone2_fixed_%d_qz", i);
        std::snprintf(pqw, sizeof(pqw), "zone2_fixed_%d_qw", i);
        std::snprintf(ruse, sizeof(ruse), "zone2_fixed_%d_use_rotate", i);
        std::snprintf(rqx, sizeof(rqx), "zone2_fixed_%d_rqx", i);
        std::snprintf(rqy, sizeof(rqy), "zone2_fixed_%d_rqy", i);
        std::snprintf(rqz, sizeof(rqz), "zone2_fixed_%d_rqz", i);
        std::snprintf(rqw, sizeof(rqw), "zone2_fixed_%d_rqw", i);
        ctx_.zone2_fixed[i].x = declare_parameter<double>(px, 0.0);
        ctx_.zone2_fixed[i].y = declare_parameter<double>(py, 0.0);
        ctx_.zone2_fixed[i].z = declare_parameter<double>(pyaw, 0.0);
        ctx_.zone2_fixed[i].qx = declare_parameter<double>(pqx, 0.0);
        ctx_.zone2_fixed[i].qy = declare_parameter<double>(pqy, 0.0);
        ctx_.zone2_fixed[i].qz = declare_parameter<double>(pqz, 0.0);
        ctx_.zone2_fixed[i].qw = declare_parameter<double>(pqw, 1.0);
        ctx_.zone2_fixed[i].use_rotate = declare_parameter<bool>(ruse, false);
        ctx_.zone2_fixed[i].rqx = declare_parameter<double>(rqx, 0.0);
        ctx_.zone2_fixed[i].rqy = declare_parameter<double>(rqy, 0.0);
        ctx_.zone2_fixed[i].rqz = declare_parameter<double>(rqz, 0.0);
        ctx_.zone2_fixed[i].rqw = declare_parameter<double>(rqw, 1.0);

        char appx[32], appy[32], bth[32], sth[32], stair[32];
        std::snprintf(appx, sizeof(appx), "zone2_fixed_%d_approach_x", i);
        std::snprintf(appy, sizeof(appy), "zone2_fixed_%d_approach_y", i);
        std::snprintf(bth, sizeof(bth), "zone2_fixed_%d_block_height", i);
        std::snprintf(sth, sizeof(sth), "zone2_fixed_%d_stand_height", i);
        std::snprintf(stair, sizeof(stair), "zone2_fixed_%d_stair_cmd", i);
        ctx_.zone2_fixed[i].approach_x = declare_parameter<double>(appx, 0.0);
        ctx_.zone2_fixed[i].approach_y = declare_parameter<double>(appy, 0.0);
        ctx_.zone2_fixed[i].block_height = static_cast<uint8_t>(declare_parameter<int>(bth, 0));
        ctx_.zone2_fixed[i].stand_height = static_cast<uint8_t>(declare_parameter<int>(sth, 0));
        ctx_.zone2_fixed[i].stair_cmd = static_cast<int8_t>(declare_parameter<int>(stair, 0));

        char rotx[32], roty[32];
        std::snprintf(rotx, sizeof(rotx), "zone2_fixed_%d_rotate_x", i);
        std::snprintf(roty, sizeof(roty), "zone2_fixed_%d_rotate_y", i);
        ctx_.zone2_fixed[i].rotate_x = declare_parameter<double>(rotx, 0.0);
        ctx_.zone2_fixed[i].rotate_y = declare_parameter<double>(roty, 0.0);
    }

    // Entry grab params
    ctx_.entry_approach_x = declare_parameter<double>("entry_approach_x", 1.6);
    ctx_.entry_block0_x = declare_parameter<double>("entry_block0_x", 2.0);
    ctx_.entry_block0_y = declare_parameter<double>("entry_block0_y", 0.289);
    ctx_.entry_block0_is_finsh = static_cast<uint8_t>(declare_parameter<int>("entry_block0_is_finsh", 2));
    ctx_.entry_block2_x = declare_parameter<double>("entry_block2_x", 3.0);
    ctx_.entry_block2_y = declare_parameter<double>("entry_block2_y", 1.41);
    ctx_.entry_block2_is_finsh = static_cast<uint8_t>(declare_parameter<int>("entry_block2_is_finsh", 1));
    ctx_.entry_stair1_x = declare_parameter<double>("entry_stair1_x", 1.8);
    ctx_.entry_stair1_y = declare_parameter<double>("entry_stair1_y", 1.41);
    ctx_.entry_rotate_x = declare_parameter<double>("entry_rotate_x", 3.0);

    // Sim mode
    ctx_.sim_mode = declare_parameter<bool>("sim_mode", false);

    // ── start state machine ───────────────────────────────────
    fsm_.start(std::make_unique<BootState>(), ctx_, actions_);

    RCLCPP_INFO(get_logger(), "R2 Decision Node Started (sim_mode=%d)", ctx_.sim_mode);
}

void R2DecisionNode::tick()
{
    actions_.tick(ctx_);
    fsm_.tick(ctx_, actions_);
    processEvents();
}

void R2DecisionNode::postEvent(EventType type, bool success, uint8_t command)
{
    event_queue_.push_back(Event{type, success, command});
}

void R2DecisionNode::processEvents()
{
    while (!event_queue_.empty())
    {
        Event e = event_queue_.front();
        event_queue_.pop_front();
        fsm_.handleEvent(ctx_, actions_, e);
    }
}
