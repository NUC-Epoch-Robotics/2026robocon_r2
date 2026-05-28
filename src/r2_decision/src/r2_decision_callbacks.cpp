#include "r2_decision/r2_decision_node.hpp"

void R2DecisionNode::onUpperAck(const robot_serial::msg::Juece::SharedPtr msg)
{
    // zhuangtai==1 表示上位机应答, status_bit 携带指令值
    if (msg->zhuangtai != 1) return;

    // 根据当前等待的是 arm command 还是 spearhead command 分发
    if (actions_.isWaitingSpearheadAck())
        actions_.handleSpearheadAck(msg->status_bit);
    else
        actions_.handleAck(msg->status_bit);
}

void R2DecisionNode::onUpperDone(const robot_serial::msg::Juece::SharedPtr msg)
{
    // zhuangtai==1 表示上位机完成, status_bit 携带指令值, is_finsh!=0 成功
    if (msg->zhuangtai != 1) return;

    // spearhead_active_ 在 sendSpearheadCommand 到 handleSpearheadDone 之间为 true
    if (actions_.hasPendingSpearhead())
    {
        actions_.handleSpearheadDone(msg->status_bit, msg->is_finsh != 0);
        postEvent(EventType::ARM_DONE, msg->is_finsh != 0, msg->status_bit);
        return;
    }

    actions_.handleArmDone(msg->status_bit, msg->is_finsh != 0);
    postEvent(EventType::ARM_DONE, msg->is_finsh != 0, msg->status_bit);
}

void R2DecisionNode::onUpJuece(const robot_serial::msg::Juece::SharedPtr msg)
{
    if (msg->zhuangtai != 1 || ctx_.grab_context == GrabContext::NONE) return;
    postEvent(EventType::UP_JUECE_DONE);
}

void R2DecisionNode::onDownJuece(const robot_serial::msg::Juece::SharedPtr msg)
{
    if (msg->zhuangtai != 1 || !actions_.isStairActive()) return;
    postEvent(EventType::DOWN_JUECE_DONE);
}

void R2DecisionNode::onSpearExists(const std_msgs::msg::Bool::SharedPtr msg)
{
    ctx_.spearhead_exists = msg->data;
}

void R2DecisionNode::onLightboardMap(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    ctx_.lightboard_map.assign(msg->data.begin(), msg->data.end());
    ctx_.lightboard_map_received = true;
}

void R2DecisionNode::onGrabSceneReady(const std_msgs::msg::Bool::SharedPtr msg)
{
    ctx_.grab_scene_ready = msg->data;
    if (msg->data)
    {
        RCLCPP_INFO(get_logger(), "Scene CONFIRMED!");
        postEvent(EventType::SCENE_READY);
    }
}

void R2DecisionNode::onButtonState(const std_msgs::msg::UInt8::SharedPtr msg)
{
    auto now_time = now();
    if (msg->data == ctx_.last_button_state &&
        (now_time - ctx_.last_button_event_time).nanoseconds() < kButtonDebounceMs * 1'000'000)
        return;

    ctx_.last_button_state = msg->data;
    ctx_.last_button_event_time = now_time;
    auto button = static_cast<ButtonState>(msg->data);

    if (button == ButtonState::START)        postEvent(EventType::START_PRESSED);
    if (button == ButtonState::ZONE1_RETRY)  postEvent(EventType::ZONE1_RETRY);
    if (button == ButtonState::ZONE2_RETRY)  postEvent(EventType::ZONE2_RETRY);
    if (button == ButtonState::ZONE3_RETRY)  postEvent(EventType::ZONE3_RETRY);
}
