#include "r2_decision/r2_decision_node.hpp"

void R2DecisionNode::onUpperAck(const robot_serial::msg::Juece::SharedPtr msg)
{
    // zhuangtai=1 → 吸到块
    if (msg->zhuangtai == 1)
    {
        postEvent(EventType::UP_JUECE_DONE);
        return;
    }

    // is_finsh=1 → 上台阶完成, is_finsh=2 → 下台阶完成
    if (msg->is_finsh == 1 || msg->is_finsh == 2)
    {
        actions_.stopStair();
        postEvent(EventType::DOWN_JUECE_DONE);
        return;
    }
}

void R2DecisionNode::onUpperDone(const robot_serial::msg::Juece::SharedPtr msg)
{
    // zhuangtai==1 表示上位机完成, status_bit 携带指令值, is_finsh!=0 成功
    if (msg->zhuangtai != 1) return;

    // 台阶完成 → DOWN_JUECE_DONE (优先级最高)
    if (actions_.isStairActive())
    {
        actions_.stopStair();
        postEvent(EventType::DOWN_JUECE_DONE);
        return;
    }

    // spearhead_active_ 在 sendSpearheadCommand 到 handleSpearheadDone 之间为 true.
    // handleSpearheadDone 严格校验 cmd 匹配: 迟到的上一条 DONE 会被丢弃, 不再误触发 ARM_DONE.
    if (actions_.hasPendingSpearhead())
    {
        if (actions_.handleSpearheadDone(msg->status_bit, msg->is_finsh != 0))
            postEvent(EventType::ARM_DONE, msg->is_finsh != 0);
        return;
    }

    actions_.handleArmDone(msg->status_bit, msg->is_finsh != 0);
    postEvent(EventType::ARM_DONE, msg->is_finsh != 0, msg->status_bit);
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

void R2DecisionNode::onDt35Location(const robot_serial::msg::Location::SharedPtr msg)
{
    ctx_.dt35_x = static_cast<double>(msg->x);
    ctx_.dt35_y = static_cast<double>(msg->y);

    if (!ctx_.dt35_received)
    {
        ctx_.dt35_received = true;
        RCLCPP_INFO(get_logger(), "DT35 location received: x=%.3f y=%.3f",
                    ctx_.dt35_x, ctx_.dt35_y);
    }
}

