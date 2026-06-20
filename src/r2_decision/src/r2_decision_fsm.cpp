#include "r2_decision/r2_decision_node.hpp"

#include <cmath>

/*
 * ============================================================================
 * 整体状态流转图
 * ============================================================================
 *
 *   Boot ─→ WaitStart ─→ Zone1 ─→ Zone2 ─→ Exit ─→ Done
 *                            │         │
 *                            │         └── 固定路线 FINISH → Done
 *                            │            动态路线 FINISH → Exit → Done
 *                            │
 *                            └── 超时 → FINISH → Zone2
 *
 *   Zone1 新流程:
 *     EXTEND_SUCTION → NAV_POINT(走X) → ROTATE_90_CW(Nav2原地转90°)
 *     → NAV_POINT_Y(走Y) → OPERATE(zhuangtai=1抓) → ROTATE_180(Nav2转180°)
 *     → DOCKING(zhuangtai=2/3对接) → WAIT_5S(等5s) → DOCKING_DONE(zhuangtai=4)
 *     → 等5s → zhuangtai=0 + area=2 → FINISH → Zone2
 *
 *   每个大状态内部用 sub_ 枚举做子状态切换:
 *     enterSub()  = 进入子状态时执行动作 (发导航、发机械臂指令、启台阶)
 *     handleSubEvent() = 事件来了判断能不能转下一个子状态
 *     onTick()    = 每 20ms 检查旋转进度、等5s超时、全局超时
 *
 *   事件驱动模式:
 *     ROS回调 → 更新 Context + postEvent() → event_queue → handleEvent() → handleSubEvent()
 *     onTick 里的步骤推进: 导航到了(等NAV_DONE事件) / 台阶完成(等DOWN_JUECE_DONE事件)
 *                         抓取完成(等UP_JUECE_DONE事件) / 场景确认(等SCENE_READY事件)
 * ============================================================================
 */

// ==========================================================================
// StateMachine — 状态切换引擎
// 核心: transitionTo() 先调旧状态的 onExit, 再调新状态的 onEnter
//       如果 onEnter 又返回一个新状态, 就链式继续切 (Boot → WaitStart 就是这样)
// ==========================================================================

void StateMachine::start(std::unique_ptr<TopState> initial, Context &ctx, ActionDispatcher &act)
{
    transitionTo(std::move(initial), ctx, act);
}

void StateMachine::tick(Context &ctx, ActionDispatcher &act)
{
    if (!current_)
        return;
    auto next = current_->onTick(ctx, act);
    if (next)
        transitionTo(std::move(next), ctx, act);
}

void StateMachine::handleEvent(Context &ctx, ActionDispatcher &act, const Event &e)
{
    if (!current_)
        return;
    auto next = current_->handleEvent(ctx, act, e);
    if (next)
        transitionTo(std::move(next), ctx, act);
}

const char *StateMachine::currentName() const
{
    return current_ ? current_->name() : "none";
}

void StateMachine::transitionTo(std::unique_ptr<TopState> next, Context &ctx, ActionDispatcher &act)
{
    if (current_)
    {
        current_->onExit(ctx, act); // 离开旧状态, 清理资源
        if (ctx.sim_mode)
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "[SIM] %s -> %s", current_->name(), next->name());
    }
    current_ = std::move(next);
    if (current_)
    {
        // 链式切换: 如果 onEnter 直接返回下一个状态 (比如 Boot 直接跳到 WaitStart),
        // 就一直链下去, 直到某个状态返回 nullptr 表示"需要等事件"
        auto chained = current_->onEnter(ctx, act);
        while (chained)
        {
            if (ctx.sim_mode)
                RCLCPP_INFO(rclcpp::get_logger("fsm"), "[SIM] %s -> %s", current_->name(), chained->name());
            current_->onExit(ctx, act);
            current_ = std::move(chained);
            chained = current_->onEnter(ctx, act);
        }
    }
}

// ==========================================================================
// BootState — 启动瞬间, 立刻跳到 WaitStart
// ==========================================================================

std::unique_ptr<TopState> BootState::onEnter(Context &ctx, ActionDispatcher &act)
{
    (void)ctx;
    (void)act;
    return std::make_unique<WaitStartState>(); // 链式切换, 不等待任何事件
}

// ==========================================================================
// WaitStartState — 等按下 START 按钮
// 动作: 关掉所有传感器省电
// 转移: START_PRESSED 事件 → Zone1
// ==========================================================================

std::unique_ptr<TopState> WaitStartState::onEnter(Context &ctx, ActionDispatcher &act)
{
    act.enableSpear(false);
    act.enableLightboard(false);
    RCLCPP_INFO(rclcpp::get_logger("fsm"), "Waiting START button...");
    (void)ctx;
    return nullptr; // 返回 nullptr = 停在这里等事件
}

std::unique_ptr<TopState> WaitStartState::handleEvent(Context &ctx, ActionDispatcher &act, const Event &e)
{
    if (e.type == EventType::START_PRESSED)
    {
        // 发一次 area=1, 告诉串口我们进入一区
        ctx.area = 1;
        act.publishCmd(0, 0, 0, 1);  // status_bit=0, is_finsh=0, zhuangtai=0, area=1
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "START pressed, area=1 sent");

        // 重置 Zone1 进度, 记录开始时间 (用于超时)
        ctx.zone1_index = 0;
        ctx.dock_success_count = 0;
        ctx.zone1_arm_retry = 0;
        ctx.zone1_start_time = rclcpp::Clock().now();
        return std::make_unique<Zone1State>();
    }
    return nullptr;
}

/*
 * ============================================================================
 * Zone1State — 一区: 矛头抓取 + 撤离流程
 * ============================================================================
 *
 *   子状态流转:
 *     EXTEND_SUCTION → NAV_POINT(走X) → ROTATE_90_CW(odom自转90°)
 *     → NAV_POINT_Y(走Y) → OPERATE(zhuangtai=1抓) → ROTATE_180(转180°)
 *     → DOCKING(zhuangtai=2/3对接) → WAIT_5S(等5s) → DOCKING_DONE(zhuangtai=4)
 *     → 等5s → zhuangtai=0 + area=2 → FINISH → Zone2
 *
 *   NAV_POINT:    导航到矛头点x坐标 (y保持, 全局坐标系)
 *   ROTATE_90_CW: Nav2原地顺时针转90°
 *   NAV_POINT_Y:  导航到矛头点y坐标 (x已到位, 全局坐标系)
 *   OPERATE:      发 zhuangtai=1 抓矛头
 *   ROTATE_180:   Nav2原地转180°
 *   DOCKING:      发 zhuangtai=2/3 矛头对接
 *   WAIT_5S:      原地等待5秒
 *   DOCKING_DONE: 发 zhuangtai=4, 等5秒, 然后发 zhuangtai=0 + area=2 复位
 *   FINISH:       构建Zone2任务, 切换到Zone2
 * ============================================================================
 */

const char *Zone1State::name() const
{
    switch (sub_)
    {
    case Sub::EXTEND_SUCTION: return "Zone1/ExtendSuction";
    case Sub::NAV_POINT:      return "Zone1/NavPoint";
    case Sub::ROTATE_90_CW:   return "Zone1/Rotate90CW";
    case Sub::NAV_POINT_Y:    return "Zone1/NavPointY";
    case Sub::OPERATE:        return "Zone1/Operate";
    case Sub::ROTATE_180:     return "Zone1/Rotate180";
    case Sub::DOCKING:        return "Zone1/Docking";
    case Sub::WAIT_5S:        return "Zone1/Wait5s";
    case Sub::DOCKING_DONE:   return "Zone1/DockingDone";
    case Sub::FINISH:         return "Zone1/Finish";
    }
    return "Zone1";
}

// 进入 Zone1
std::unique_ptr<TopState> Zone1State::onEnter(Context &ctx, ActionDispatcher &act)
{
    sub_ = Sub::EXTEND_SUCTION;
    enterSub(ctx, act);
    return nullptr;
}

// Zone1 每 20ms 的检查:
//   1. 旋转子状态: odom + cmd_vel 驱动原地旋转
//   2. WAIT_5S: 计时5秒
//   3. 全局超时检查
//   4. FINISH: 构建 Zone2 任务列表, 切换到 Zone2
std::unique_ptr<TopState> Zone1State::onTick(Context &ctx, ActionDispatcher &act)
{
    checkTimeLimit(ctx, act);

    // WAIT_5S: 计时5秒后 → DOCKING_DONE (发 zhuangtai=4)
    if (sub_ == Sub::WAIT_5S)
    {
        auto elapsed = (rclcpp::Clock().now() - ctx.wait_5s_start_time).seconds();
        if (elapsed > 5.0)
        {
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: WAIT_5S done → DOCKING_DONE");
            sub_ = Sub::DOCKING_DONE;
            enterSub(ctx, act);
        }
    }

    // DOCKING_DONE: zhuangtai=4 → 等5s → zhuangtai=0 area=1 → 等5s → zhuangtai=0 area=2 → FINISH
    if (sub_ == Sub::DOCKING_DONE)
    {
        auto elapsed = (rclcpp::Clock().now() - ctx.wait_5s_start_time).seconds();
        if (ctx.zone1_dock_step == 0 && elapsed > 5.0)
        {
            // zhuangtai=4 的 5s 到了, 发 zhuangtai=0 area=1
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: DOCKING_DONE → zhuangtai=0 area=1");
            act.setHoldCmd(0);  // 清掉 hold, 允许发 zhuangtai=0
            act.publishCmdWithArea(0, 0, 0);  // zhuangtai=0, area=1
            ctx.wait_5s_start_time = rclcpp::Clock().now();
            ctx.zone1_dock_step = 1;
        }
        else if (ctx.zone1_dock_step == 1 && elapsed > 5.0)
        {
            // 5s 到了, 发 zhuangtai=0 area=2 切二区
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: DOCKING_DONE → zhuangtai=0 area=2");
            ctx.area = 2;
            act.publishCmd(0, 0, 0, 2);  // zhuangtai=0, area=2
            sub_ = Sub::FINISH;
            enterSub(ctx, act);
        }
    }

    if (sub_ == Sub::FINISH)
    {
        if (ctx.use_fixed_zone2_route)
            R2DecisionNode::buildZone2FixedRoute(ctx);
        else
            R2DecisionNode::buildZone2Route(ctx, ctx.latest_lightboard_map);

        if (ctx.zone2_tasks.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone2: no tasks, mission done");
            return std::make_unique<DoneState>();
        }
        return std::make_unique<Zone2State>();
    }
    return nullptr;
}

// Zone1 的全局事件: 重试按钮 / 超时
// 其他事件 (NAV_DONE, ARM_DONE...) 交给 handleSubEvent 按子状态分发
std::unique_ptr<TopState> Zone1State::handleEvent(Context &ctx, ActionDispatcher &act, const Event &e)
{
    if (e.type == EventType::ZONE1_RETRY)
    {
        // 重试: 回到第一个矛头点, 清空所有进度
        if (ctx.nav_chain_in_progress)
            return nullptr; // 导航中不能重试
        ctx.zone1_index = 0;
        ctx.dock_success_count = 0;
        ctx.zone1_arm_retry = 0;
        ctx.zone1_start_time = rclcpp::Clock().now();
        act.enableSpear(true);
        sub_ = Sub::EXTEND_SUCTION;
        enterSub(ctx, act);
        return nullptr;
    }
    if (e.type == EventType::ZONE1_TIMEOUT)
    {
        // 超时: 关传感器, 直接结束一区进二区
        act.enableSpear(false);
        act.enableLightboard(false);
        act.stopCmdVel();
        sub_ = Sub::FINISH;
        enterSub(ctx, act);
        return nullptr;
    }
    return handleSubEvent(ctx, act, e);
}

void Zone1State::onExit(Context &ctx, ActionDispatcher &act)
{
    (void)ctx;
    act.stopCmdVel();
    act.stopStair();
    act.enableSpear(false);
    act.enableLightboard(false);
}

/*
 * enterSub — 进入子状态时执行的动作
 * 每个 case 对应一个子状态, 做一件事: 发导航 / 发机械臂指令 / 启台阶
 */
void Zone1State::enterSub(Context &ctx, ActionDispatcher &act)
{
    switch (sub_)
    {
    case Sub::EXTEND_SUCTION:
    {
        // 吸盘有问题，跳过伸吸盘，直接走X
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: EXTEND_SUCTION skipped (suction disabled)");
        sub_ = Sub::NAV_POINT;
        enterSub(ctx, act);
        return;
    }
    case Sub::NAV_POINT:
    {
        // 所有矛头点都走完了?
        if (ctx.zone1_index >= ctx.zone1_route_ids.size())
        {
            sub_ = Sub::FINISH;
            enterSub(ctx, act);
            return;
        }
        const int pid = ctx.zone1_route_ids[ctx.zone1_index];
        auto it = ctx.point_table.find(pid);
        if (it == ctx.point_table.end())
        {
            RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1: missing point id=%d, skip", pid);
            ++ctx.zone1_index;
            enterSub(ctx, act);
            return;
        }
        {
            const auto &t = it->second;
            // 第一段: 只变x (全局坐标系), y保持当前位置
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: nav point %d x=%.2f (y stays %.2f)", t.id, t.x, ctx.current_y);
            act.sendNavigateWithQuat(t.x, ctx.current_y, t.z, 0, 0, 0, 1, ctx);
        }
        break;
    }
    case Sub::ROTATE_90_CW:
    {
        // 从当前朝向顺时针转90度 (相对旋转, 不是绝对角度)
        double target_yaw = ctx.odom_yaw - M_PI_2;
        while (target_yaw > M_PI)  target_yaw -= 2.0 * M_PI;
        while (target_yaw < -M_PI) target_yaw += 2.0 * M_PI;
        double qz = std::sin(target_yaw / 2.0);
        double qw = std::cos(target_yaw / 2.0);
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: ROTATE_90_CW at (%.2f, %.2f) target_yaw=%.3f (current=%.3f)",
                    ctx.current_x, ctx.current_y, target_yaw, ctx.odom_yaw);
        act.sendNavigateWithQuat(ctx.current_x, ctx.current_y, 0, 0, 0, qz, qw, ctx);
        break;
    }
    case Sub::NAV_POINT_Y:
    {
        // 第二段: 变y到矛头点 (全局坐标系), x已经在上一步到位了
        // 保持转90°后的朝向 (qz=-0.707, qw=0.707), 不让Nav2偷偷转回去
        const int pid = ctx.zone1_route_ids[ctx.zone1_index];
        auto it = ctx.point_table.find(pid);
        const auto &t = it->second;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: nav point %d y=%.2f (x=%.2f, keep -90deg)", t.id, t.y, t.x);
        act.sendNavigateWithQuat(t.x, t.y, t.z, 0, 0, -0.707, 0.707, ctx);
        break;
    }
    case Sub::OPERATE:
    {
        // 导航到了矛头点, 发 zhuangtai=1 开始抓
        const int pid = ctx.zone1_route_ids[ctx.zone1_index];
        auto it = ctx.point_table.find(pid);
        if (it == ctx.point_table.end())
        {
            RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1: OPERATE missing point %d, skip", pid);
            sub_ = Sub::ROTATE_180;
            enterSub(ctx, act);
            return;
        }
        ctx.zone1_arm_retry = 0;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1 point %d: GRAB zhuangtai=1 (docking_cmd=%d)", pid, it->second.docking_cmd);
        act.setHoldCmd(1);  // 抓取后保持 zhuangtai=1 不松夹爪
        act.sendSpearheadCommand(1);  // zhuangtai=1: 开始抓
        break;
    }
    case Sub::ROTATE_180:
    {
        // 从当前朝向转180度: 当前-90° → +90° (qz=0.707, qw=0.707)
        // 如果当前是其他朝向, 同理加π
        double target_yaw = ctx.odom_yaw + M_PI;
        while (target_yaw > M_PI)  target_yaw -= 2.0 * M_PI;
        while (target_yaw < -M_PI) target_yaw += 2.0 * M_PI;
        double qz = std::sin(target_yaw / 2.0);
        double qw = std::cos(target_yaw / 2.0);
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: ROTATE_180 at (%.2f, %.2f) target_yaw=%.3f",
                    ctx.current_x, ctx.current_y, target_yaw);
        act.sendNavigateWithQuat(ctx.current_x, ctx.current_y, 0, 0, 0, qz, qw, ctx);
        break;
    }
    case Sub::DOCKING:
    {
        // 转180°完成, 发矛头对接指令 (zhuangtai=2 或 3)
        // zone1_index 已经在 OPERATE 完成时 +1 了, 这里用 index-1 拿刚抓的矛头
        const int pid = ctx.zone1_route_ids[ctx.zone1_index - 1];
        auto it = ctx.point_table.find(pid);
        if (it == ctx.point_table.end())
        {
            RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1: DOCKING missing point %d, skip", pid);
            sub_ = Sub::WAIT_5S;
            enterSub(ctx, act);
            return;
        }
        uint8_t cmd = it->second.docking_cmd;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: DOCKING zhuangtai=%d (point %d)", cmd, pid);
        act.setHoldCmd(cmd);  // 对接后保持 zhuangtai=2/3 不松夹爪
        act.sendSpearheadCommand(cmd);
        break;
    }
    case Sub::WAIT_5S:
    {
        ctx.wait_5s_start_time = rclcpp::Clock().now();
        act.stopCmdVel();
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: WAIT_5S (等5s后发zhuangtai=4)");
        break;
    }
    case Sub::DOCKING_DONE:
    {
        // 发 zhuangtai=4, 然后等5s → zhuangtai=0 area=1 → 等5s → zhuangtai=0 area=2
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: DOCKING_DONE zhuangtai=4");
        act.setHoldCmd(4);  // 等待期间保持 zhuangtai=4 不松夹爪
        act.sendSpearheadCommand(4);
        ctx.wait_5s_start_time = rclcpp::Clock().now();
        ctx.zone1_dock_step = 0;
        break;
    }
    case Sub::FINISH:
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: FINISH");
        break;
    }
}

/*
 * handleSubEvent — 事件来了, 根据当前子状态决定下一步
 * 模式: 收到事件 → 切换 sub_ → 调 enterSub 执行新动作
 */
std::unique_ptr<TopState> Zone1State::handleSubEvent(Context &ctx, ActionDispatcher &act, const Event &e)
{
    switch (sub_)
    {
    case Sub::EXTEND_SUCTION:
        // 伸吸盘完成 → 走第一段x
        if (e.type == EventType::ARM_DONE)
        {
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Extend suction failed, continue anyway");
            sub_ = Sub::NAV_POINT;
            enterSub(ctx, act);
        }
        break;

    case Sub::NAV_POINT:
        // 第一段x完成 → 顺时针转90度 (odom + cmd_vel)
        if (e.type == EventType::NAV_DONE)
        {
            if (!e.success)
            {
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1: nav x failed, skip");
                ++ctx.zone1_index;
                sub_ = Sub::NAV_POINT;
            }
            else
            {
                sub_ = Sub::ROTATE_90_CW;
            }
            enterSub(ctx, act);
        }
        break;

    case Sub::ROTATE_90_CW:
        // 顺时针90度完成 → 走第二段y
        if (e.type == EventType::NAV_DONE)
        {
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1: rotate failed, continue anyway");
            sub_ = Sub::NAV_POINT_Y;
            enterSub(ctx, act);
        }
        break;

    case Sub::NAV_POINT_Y:
        // 第二段y完成 → 抓矛头
        if (e.type == EventType::NAV_DONE)
        {
            if (!e.success)
            {
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1: nav y failed, skip");
                ++ctx.zone1_index;
                sub_ = Sub::NAV_POINT;
            }
            else
            {
                sub_ = Sub::OPERATE;
            }
            enterSub(ctx, act);
        }
        break;

    case Sub::OPERATE:
        // 抓矛头完成 (is_finsh=1 的 ARM_DONE) → 转180度
        if (e.type == EventType::ARM_DONE)
        {
            // 失败且还有重试次数 → 重发 is_finsh=1
            if (!e.success && ctx.zone1_arm_retry < kZone1ArmMaxRetry)
            {
                ++ctx.zone1_arm_retry;
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1 grab retry %d/%d",
                            ctx.zone1_arm_retry, kZone1ArmMaxRetry);
                act.sendSpearheadCommand(1);
                break;
            }
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1 grab failed after retry, continue");

            ctx.zone1_arm_retry = 0;
            ++ctx.zone1_index;

            // 抓好 → 转180度
            sub_ = Sub::ROTATE_180;
            enterSub(ctx, act);
        }
        break;

    case Sub::ROTATE_180:
        // 转180度完成 → 发矛头对接指令 (is_finsh=2/3)
        if (e.type == EventType::NAV_DONE)
        {
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1: rotate180 failed, continue anyway");
            sub_ = Sub::DOCKING;
            enterSub(ctx, act);
        }
        break;

    case Sub::DOCKING:
        // 对接指令完成 (is_finsh=2/3 的 ARM_DONE) → 等5秒
        if (e.type == EventType::ARM_DONE)
        {
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1: docking failed, continue anyway");
            sub_ = Sub::WAIT_5S;
            enterSub(ctx, act);
        }
        break;

    case Sub::WAIT_5S:
        // WAIT_5S 由 onTick 计时驱动
        break;

    case Sub::DOCKING_DONE:
        // zhuangtai=4 的 ARM_DONE, 清 spearhead_active_ 等 onTick 5s 到期
        if (e.type == EventType::ARM_DONE)
        {
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: DOCKING_DONE ack, wait 5s");
        }
        break;

    case Sub::FINISH:
        break;
    }
    return nullptr;
}

// 全局一区超时: 超过 zone1_max_time_s (默认120s) 发出 ZONE1_TIMEOUT 事件
void Zone1State::checkTimeLimit(Context &ctx, ActionDispatcher &act)
{
    if (sub_ == Sub::WAIT_5S || sub_ == Sub::DOCKING || sub_ == Sub::DOCKING_DONE || sub_ == Sub::FINISH || sub_ == Sub::EXTEND_SUCTION)
        return; // 已经在收尾步骤了, 不打断

    auto elapsed = (rclcpp::Clock().now() - ctx.zone1_start_time).seconds();
    if (elapsed > ctx.zone1_max_time_s)
    {
        RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1 time limit reached (%.1fs)", elapsed);
        act.postEvent({EventType::ZONE1_TIMEOUT});
    }
}

/*
 * ============================================================================
 * Zone2State — 二区: 梅花林 4×3 网格, 抓取 R2 方块
 * ============================================================================
 *
 *   固定路线 (use_fixed_zone2_route = true):
 *     ENTRY_GRAB ─→ NAV_POINT ─→ GRAB/ROTATE/STAIRS ─→ NAV_POINT ─→ ... ─→ FINISH → Done
 *         │                        │
 *         │                        ├── has_grab → GRAB
 *         │                        ├── use_rotate → ROTATE
 *         │                        ├── stair_cmd=1/2 → UP/DOWN_STAIRS
 *         │                        └── else → FINISH
 *         │
 *         └── 入口抓取两个块 (block0 is_finsh=2, block2 is_finsh=1)
 *
 *   动态路线 (use_fixed_zone2_route = false):
 *     NAV_POINT ─→ WAIT_SCENE ─→ GRAB (arm cmd) ─→ NAV_POINT ─→ ... ─→ FINISH → Exit → Done
 *                     │               │
 *                     │ timeout → skip │ ARM_DONE → ++index
 *
 *   Point0 (梅花林第一个抓取点, id==0) 有特殊的多步序列:
 *     抓完 → 后退 → 上台阶#1 → NAV到旋转点 → 右转 → 上台阶#2 → 转回 → 上台阶#3 → 下一个点
 * ============================================================================
 */

const char *Zone2State::name() const
{
    switch (sub_)
    {
    case Sub::ENTRY_GRAB:
        return "Zone2/EntryGrab";
    case Sub::NAV_POINT:
        return "Zone2/NavPoint";
    case Sub::ROTATE:
        return "Zone2/Rotate";
    case Sub::ROTATE_GRAB:
        return "Zone2/RotateGrab";
    case Sub::WAIT_SCENE:
        return "Zone2/WaitScene";
    case Sub::GRAB:
        return "Zone2/Grab";
    case Sub::UP_STAIRS:
        return "Zone2/UpStairs";
    case Sub::DOWN_STAIRS:
        return "Zone2/DownStairs";
    case Sub::FINISH:
        return "Zone2/Finish";
    }
    return "Zone2";
}

std::unique_ptr<TopState> Zone2State::onEnter(Context &ctx, ActionDispatcher &act)
{
    ctx.zone2_post_grab_nav_pending = false;
    ctx.zone2_index = 0;

    if (ctx.zone2_tasks.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone2: no tasks, mission done");
        return std::make_unique<DoneState>();
    }

    // 固定路线: 先做入口导航 (approach → block), 再逐点导航
    // 动态路线: 直接从第一个任务点开始
    if (ctx.use_fixed_zone2_route)
    {
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: %zu tasks, start entry nav", ctx.zone2_tasks.size());
        ctx.entry_grab_step = 0;
        sub_ = Sub::ENTRY_GRAB;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: %zu tasks", ctx.zone2_tasks.size());
        sub_ = Sub::NAV_POINT;
    }
    enterSub(ctx, act);
    return nullptr;
}

std::unique_ptr<TopState> Zone2State::onTick(Context &ctx, ActionDispatcher &act)
{
    // 入口抓取: 完全由事件驱动 (handleSubEvent → tickEntryGrab), onTick 不介入
    // 固定路线抓取: zone2_grab_step 驱动 (backoff → forward → 等 UP_JUECE)
    if (sub_ == Sub::GRAB && ctx.use_fixed_zone2_route)
        tickGrab(ctx, act);
    // 动态路线场景确认超时检查
    if (sub_ == Sub::WAIT_SCENE)
        checkSceneTimeout(ctx, act);

    if (sub_ == Sub::FINISH)
    {
        act.enableGrabScene(false);
        // 固定路线结束 → Done, 动态路线结束 → 开车去出口 → Done
        if (ctx.use_fixed_zone2_route)
            return std::make_unique<DoneState>();
        else
            return std::make_unique<ExitState>();
    }
    return nullptr;
}

std::unique_ptr<TopState> Zone2State::handleEvent(Context &ctx, ActionDispatcher &act, const Event &e)
{
    return handleSubEvent(ctx, act, e);
}

void Zone2State::onExit(Context &ctx, ActionDispatcher &act)
{
    act.enableGrabScene(false);
    act.stopEntryGrab();
    act.stopZone2Grab();
    act.stopStair();
    ctx.grab_context = GrabContext::NONE;
    ctx.zone2_point0_sequence_active = false;
    ctx.zone2_point0_substep = 0;
    ctx.zone2_post_grab_nav_pending = false;
}

/*
 * Zone2 enterSub — 进入每个子状态时执行的动作
 */
void Zone2State::enterSub(Context &ctx, ActionDispatcher &act)
{
    switch (sub_)
    {
    case Sub::ENTRY_GRAB:
        // 入口抓取: 4 步序列由 tickEntryGrab 驱动
        tickEntryGrab(ctx, act);
        break;

    case Sub::NAV_POINT:
    {
        // 所有任务点都走完了?
        if (ctx.zone2_index >= ctx.zone2_tasks.size())
        {
            sub_ = Sub::FINISH;
            enterSub(ctx, act);
            return;
        }
        const auto &t = ctx.zone2_tasks[ctx.zone2_index];
        // 有 approach → 先去 approach, 没有 → 直接去目标点
        bool has_approach = (t.approach_x != 0.0 || t.approach_y != 0.0);
        double nx = has_approach ? t.approach_x : t.x;
        double ny = has_approach ? t.approach_y : t.y;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: nav to point %d (%.2f,%.2f) z=%.3f stair=%d rot=%d",
                    t.id, nx, ny, t.z, t.stair_cmd, t.use_rotate);
        ctx.zone2_stair_pending = has_approach;  // 标记: 到了 approach 后还要去 block
        act.sendNavigateWithQuat(nx, ny, t.z, t.qx, t.qy, t.qz, t.qw, ctx);
        break;
    }

    case Sub::ROTATE:
    {
        // 有旋转点 → 导航到旋转点+旋转 (一个 Nav2 goal)
        // 没旋转点 → 原地旋转
        const auto &t = ctx.zone2_tasks[ctx.zone2_index];
        double rx = (t.rotate_x != 0.0) ? t.rotate_x : ctx.current_x;
        double ry = (t.rotate_y != 0.0) ? t.rotate_y : ctx.current_y;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: ROTATE → (%.2f,%.2f) q=(%.3f,%.3f,%.3f,%.3f)",
                    rx, ry, t.rqx, t.rqy, t.rqz, t.rqw);
        act.sendNavigateWithQuat(rx, ry, t.z, t.rqx, t.rqy, t.rqz, t.rqw, ctx);
        break;
    }

    case Sub::ROTATE_GRAB:
    {
        // 动态路线: 转向拿相邻格KFS
        const auto &t = ctx.zone2_tasks[ctx.zone2_index];
        int adj = t.grab_adjacent_block;
        if (adj >= 0 && adj < 12)
        {
            // 计算朝向相邻格的四元数
            double dx = ctx.zone2_blocks[adj].x - t.x;
            double dy = ctx.zone2_blocks[adj].y - t.y;
            double yaw = std::atan2(dy, dx);
            double qz = std::sin(yaw / 2.0);
            double qw = std::cos(yaw / 2.0);
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: ROTATE_GRAB block %d → adj %d yaw=%.2f",
                        t.id, adj, yaw);
            act.sendNavigateWithQuat(t.x, t.y, t.z, 0, 0, qz, qw, ctx);
        }
        else
        {
            // 无效的相邻块，跳过
            RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone2: ROTATE_GRAB invalid adj=%d, skip", adj);
            ++ctx.zone2_index;
            sub_ = Sub::NAV_POINT;
            enterSub(ctx, act);
        }
        break;
    }

    case Sub::WAIT_SCENE:
    {
        // 动态路线: 开抓取场景摄像头, 等视觉确认当前场景匹配
        const auto &t = ctx.zone2_tasks[ctx.zone2_index];
        act.enableGrabScene(true, t.grab_scene);
        ctx.scene_confirm_start_time = rclcpp::Clock().now();
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2 block %d: waiting scene %d...", t.id, t.grab_scene);
        break;
    }

    case Sub::GRAB:
    {
        const auto &t = ctx.zone2_tasks[ctx.zone2_index];
        if (ctx.use_fixed_zone2_route)
        {
            // ── 固定路线抓取 ──
            // Point0 的多步序列 (上台阶/旋转), 已激活则继续
            if (t.id == 0 && ctx.zone2_point0_sequence_active)
            {
                handlePoint0Substep(ctx, act);
                return;
            }
            // Point0 第一步: 发抓取指令, 导航冲向方块
            if (t.id == 0)
            {
                if (ctx.zone2_grab_step == 0)
                {
                    if (ctx.grab_context != GrabContext::ZONE2_FIXED)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 grab: START is_finsh=%d", t.grab_is_finsh);
                        act.startZone2Grab(t.grab_is_finsh, ctx);
                    }
                    RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 grab: forward to block (%.2f,%.2f)", t.x, t.y);
                    ctx.zone2_grab_step = 1;
                    act.sendNavigateWithQuat(t.x, t.y, 0, t.qx, t.qy, t.qz, t.qw, ctx);
                }
                return;
            }
            // 非 Point0 固定路线抓取: backoff → grab_start
            if (ctx.grab_context != GrabContext::ZONE2_FIXED)
            {
                // step 0: 后退一小段 (zone2_fixed_backoff=0.1m)
                if (ctx.zone2_grab_step == 0)
                {
                    double yaw = ActionDispatcher::yawFromQuat(t.qx, t.qy, t.qz, t.qw);
                    double bx = t.approach_x - std::cos(yaw) * ctx.zone2_fixed_backoff;
                    double by = t.approach_y - std::sin(yaw) * ctx.zone2_fixed_backoff;
                    RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2Grab point %d: backoff to (%.2f,%.2f)", t.id, bx, by);
                    ctx.zone2_grab_step = 1;
                    act.sendNavigateWithQuat(bx, by, 0, t.qx, t.qy, t.qz, t.qw, ctx);
                    return;
                }
                // step 1: 后退到了, 开始抓取
                if (ctx.zone2_grab_step == 1)
                {
                    RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2Grab point %d: START is_finsh=%d", t.id, t.grab_is_finsh);
                    act.startZone2Grab(t.grab_is_finsh, ctx);
                    ctx.zone2_grab_step = 2;
                }
            }
            return;
        }
        else
        {
            // ── 动态路线: 场景确认了, 发机械臂指令抓取 ──
            act.enableGrabScene(false);
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2 block %d: scene confirmed, arm cmd=%d", t.id, t.arm_command);
            ctx.zone2_arm_retry = 0;
            act.sendArmCommand(t.arm_command);
        }
        break;
    }

    case Sub::UP_STAIRS:
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: UP_STAIRS");
        act.startStair(1, ctx);
        break;

    case Sub::DOWN_STAIRS:
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: DOWN_STAIRS");
        act.startStair(2, ctx);
        break;

    case Sub::FINISH:
        break;
    }
}

/*
 * Zone2 handleSubEvent — 根据子状态和事件类型决定下一步
 */
std::unique_ptr<TopState> Zone2State::handleSubEvent(Context &ctx, ActionDispatcher &act, const Event &e)
{
    switch (sub_)
    {
    case Sub::ENTRY_GRAB:
        // step 1: approach NAV_DONE → 导航到方块位置 (不抓)
        if (e.type == EventType::NAV_DONE && ctx.entry_grab_step == 1)
        {
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "EntryGrab: nav to approach failed");
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "EntryGrab: approach reached, forward to block (%.2f,%.2f)",
                        ctx.entry_block0_x, ctx.entry_block0_y);
            ctx.entry_grab_step = 2;
            act.sendNavigateWithQuat(ctx.entry_block0_x, ctx.entry_block0_y, 0, 0, 0, 0, 1, ctx);
        }
        // step 2: 到达方块位置 → 直接进入 NAV_POINT (跳过抓取)
        else if (e.type == EventType::NAV_DONE && ctx.entry_grab_step == 2)
        {
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "EntryGrab: forward nav failed");
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "EntryGrab: block reached, start zone2 nav");
            ctx.entry_grab_step = 0;
            sub_ = Sub::NAV_POINT;
            enterSub(ctx, act);
        }
        break;

    case Sub::NAV_POINT:
        // 导航到了 → 根据任务属性决定下一步
        if (e.type == EventType::NAV_DONE)
        {
            if (!e.success)
            {
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone2: nav failed, skip");
                ++ctx.zone2_index;
                sub_ = Sub::NAV_POINT;
                enterSub(ctx, act);
                break;
            }
            const auto &t = ctx.zone2_tasks[ctx.zone2_index];

            if (ctx.use_fixed_zone2_route)
            {
                // 刚到 approach? → 先去 block (2.0, 1.41)
                if (ctx.zone2_stair_pending)
                {
                    ctx.zone2_stair_pending = false;
                    RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: approach reached, nav to block (%.2f,%.2f)", t.x, t.y);
                    act.sendNavigateWithQuat(t.x, t.y, t.z, t.qx, t.qy, t.qz, t.qw, ctx);
                    break;
                }
                // 到了 block → 台阶 > 旋转 > 下一个点
                if (t.stair_cmd == 1)
                    sub_ = Sub::UP_STAIRS;
                else if (t.stair_cmd == 2)
                    sub_ = Sub::DOWN_STAIRS;
                else if (t.use_rotate)
                    sub_ = Sub::ROTATE;
                else
                {
                    ++ctx.zone2_index;
                    sub_ = Sub::NAV_POINT;
                }
                enterSub(ctx, act);
            }
            else
            {
                const auto &t = ctx.zone2_tasks[ctx.zone2_index];
                if (t.grab_adjacent_block >= 0)
                    sub_ = Sub::ROTATE_GRAB;
                else
                    sub_ = Sub::WAIT_SCENE;
                enterSub(ctx, act);
            }
        }
        break;

    case Sub::ROTATE_GRAB:
        // 转向完成 → 发机械臂指令抓取相邻格KFS → 等 ARM_DONE
        if (e.type == EventType::NAV_DONE)
        {
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone2: rotate_grab nav failed");
            const auto &t = ctx.zone2_tasks[ctx.zone2_index];
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: ROTATE_GRAB done, grab_adj=%d grab_finsh=%d",
                        t.grab_adjacent_block, t.grab_is_finsh);
            ctx.zone2_arm_retry = 0;
            act.sendArmCommand(t.grab_is_finsh);
            sub_ = Sub::GRAB;  // 等 ARM_DONE，由 GRAB case 处理后续
        }
        break;

    case Sub::WAIT_SCENE:
        // 场景确认 → 去抓取; 超时 → 跳过这个块
        if (e.type == EventType::SCENE_READY)
        {
            sub_ = Sub::GRAB;
            enterSub(ctx, act);
        }
        else if (e.type == EventType::SCENE_CONFIRM_TIMEOUT)
        {
            RCLCPP_WARN(rclcpp::get_logger("fsm"), "Scene confirm timeout, skip block");
            act.enableGrabScene(false);
            ++ctx.zone2_index;
            sub_ = Sub::NAV_POINT;
            enterSub(ctx, act);
        }
        break;

    case Sub::GRAB:
        // ── Point0 多步序列中的导航完成 (substep 1→2, 2→3, 4→5) ──
        if (e.type == EventType::NAV_DONE && ctx.use_fixed_zone2_route && ctx.zone2_point0_sequence_active)
        {
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Point0: nav failed");
            int ss = ctx.zone2_point0_substep;
            if (ss == 1)
                ctx.zone2_point0_substep = 2;
            else if (ss == 2)
                ctx.zone2_point0_substep = 3;
            else if (ss == 4)
                ctx.zone2_point0_substep = 5;
            ctx.point0_nav_sent = false;
            tickGrab(ctx, act);
        }
        // ── 固定路线非 Point0: backoff/forward 导航完成 ──
        else if (e.type == EventType::NAV_DONE && ctx.use_fixed_zone2_route)
        {
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone2Grab: nav failed");
            tickGrab(ctx, act);
        }
        // ── 动态路线: 抓取完成, 导航到方块位置后的 NAV_DONE ──
        else if (e.type == EventType::NAV_DONE && !ctx.use_fixed_zone2_route && ctx.zone2_post_grab_nav_pending)
        {
            ctx.zone2_post_grab_nav_pending = false;
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone2: nav to block after grab failed");
            const auto &t = ctx.zone2_tasks[ctx.zone2_index];
            if (t.use_rotate)
                sub_ = Sub::ROTATE;
            else if (t.stair_cmd == 1)
                sub_ = Sub::UP_STAIRS;
            else if (t.stair_cmd == 2)
                sub_ = Sub::DOWN_STAIRS;
            else
                sub_ = Sub::FINISH;
            enterSub(ctx, act);
        }
        // ── 固定路线抓取完成 (UP_JUECE_DONE) ──
        else if (e.type == EventType::UP_JUECE_DONE && ctx.use_fixed_zone2_route)
        {
            act.stopZone2Grab();
            ctx.grab_context = GrabContext::NONE;
            ctx.zone2_grab_step = 0;
            const auto &t = ctx.zone2_tasks[ctx.zone2_index];

            // Point0 抓完 → 后退, 然后进入多步序列 (上台阶/旋转)
            if (t.id == 0)
            {
                ctx.zone2_grab_step = 3; // 标记: 下一步是 Point0 序列
                RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 grab done, retreat to approach (%.2f,%.2f)",
                            t.approach_x, t.approach_y);
                act.sendNavigateWithQuat(t.approach_x, t.approach_y, 0, t.qx, t.qy, t.qz, t.qw, ctx);
            }
            // 非 Point0: 抓完 → 台阶/旋转/下一个
            else if (t.stair_cmd == 1)
            {
                sub_ = Sub::UP_STAIRS;
                enterSub(ctx, act);
            }
            else if (t.stair_cmd == 2)
            {
                sub_ = Sub::DOWN_STAIRS;
                enterSub(ctx, act);
            }
            else if (t.use_rotate)
            {
                sub_ = Sub::ROTATE;
                enterSub(ctx, act);
            }
            else
            {
                sub_ = Sub::FINISH;
                enterSub(ctx, act);
            }
        }
        // ── 动态路线: 机械臂动作完成 ──
        else if (e.type == EventType::ARM_DONE && !ctx.use_fixed_zone2_route)
        {
            // 失败重试
            if (!e.success && ctx.zone2_arm_retry < kZone2ArmMaxRetry)
            {
                ++ctx.zone2_arm_retry;
                const auto &t = ctx.zone2_tasks[ctx.zone2_index];
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone2 arm retry %d/%d",
                            ctx.zone2_arm_retry, kZone2ArmMaxRetry);
                act.sendArmCommand(t.arm_command);
                break;
            }
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone2 arm failed after retry, skip");

            ctx.zone2_arm_retry = 0;

            // 检查当前任务是否有台阶需要走
            {
                const auto &t = ctx.zone2_tasks[ctx.zone2_index];
                if (t.stair_cmd == 1 || t.stair_cmd == 2)
                {
                    RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: grab done, stair_cmd=%d", t.stair_cmd);
                    sub_ = (t.stair_cmd == 1) ? Sub::UP_STAIRS : Sub::DOWN_STAIRS;
                    enterSub(ctx, act);
                    break;
                }
            }

            ++ctx.zone2_index;
            sub_ = Sub::NAV_POINT;
            enterSub(ctx, act);
        }
        break;

    case Sub::UP_STAIRS:
    case Sub::DOWN_STAIRS:
        // 到达旋转点 → 开始旋转
        if (e.type == EventType::NAV_DONE && ctx.zone2_stair_pending)
        {
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Nav to rotate point failed, still rotate");
            ctx.zone2_stair_pending = false;
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Rotate point reached, now rotate");
            sub_ = Sub::ROTATE;
            enterSub(ctx, act);
        }
        // 台阶完成 (DOWN_JUECE_DONE)
        else if (e.type == EventType::DOWN_JUECE_DONE)
        {
            act.stopStair();
            ctx.zone2_stair_pending = false;
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Stair done");

            // Point0 序列中的台阶: 推进 substep, 回到 GRAB 继续下一步
            if (ctx.zone2_point0_sequence_active)
            {
                ++ctx.zone2_point0_substep;
                sub_ = Sub::GRAB;
                enterSub(ctx, act);
            }
            else
            {
                const auto &t = ctx.zone2_tasks[ctx.zone2_index];
                // 上完台阶 → 有旋转点? 先导航到旋转点 > 直接下一个点
                if (t.use_rotate && (t.rotate_x != 0.0 || t.rotate_y != 0.0))
                {
                    // 导航到旋转点 (3.0, 1.41), 不带旋转四元数
                    RCLCPP_INFO(rclcpp::get_logger("fsm"), "Stair done, nav to rotate point (%.2f,%.2f)",
                                t.rotate_x, t.rotate_y);
                    ctx.zone2_stair_pending = true;  // 标记: 到了旋转点后要转
                    act.sendNavigateWithQuat(t.rotate_x, t.rotate_y, t.z, t.qx, t.qy, t.qz, t.qw, ctx);
                }
                else if (t.use_rotate)
                {
                    // 有旋转但没旋转点 → 原地转
                    RCLCPP_INFO(rclcpp::get_logger("fsm"), "Stair done, rotate in place");
                    sub_ = Sub::ROTATE;
                    enterSub(ctx, act);
                }
                else
                {
                    ++ctx.zone2_index;
                    sub_ = Sub::NAV_POINT;
                    enterSub(ctx, act);
                }
            }
        }
        break;

    case Sub::ROTATE:
        // 旋转完成 (DOWN_JUECE_DONE 或 NAV_DONE)
        if (e.type == EventType::DOWN_JUECE_DONE || e.type == EventType::NAV_DONE)
        {
            if (e.type == EventType::NAV_DONE && !e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone2: rotate nav failed");
            if (e.type == EventType::DOWN_JUECE_DONE)
                act.stopStair();
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Rotate done");
            ++ctx.zone2_index;
            sub_ = Sub::NAV_POINT;
            enterSub(ctx, act);
        }
        break;

    case Sub::FINISH:
        break;
    }
    return nullptr;
}

/*
 * tickEntryGrab — 入口抓取 4 步序列
 *   step 0: 导航到 approach 位置 (入口前方)
 *   step 1: 启动抓取, 导航冲向方块
 *   step 2: 等待 UP_JUECE_DONE 事件 (抓取完成)
 *   step 3: 导航撤退回 approach, 然后收起机械臂, 进入正常导航
 */
void Zone2State::tickEntryGrab(Context &ctx, ActionDispatcher &act)
{
    switch (ctx.entry_grab_step)
    {
    case 0:
    {
        // 导航到 approach 位置, 前进到 step 1, 等 NAV_DONE
        double by = ctx.entry_block0_y;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "EntryGrab step0: nav approach (%.2f,%.2f)",
                    ctx.entry_approach_x, by);
        ctx.entry_grab_step = 1;  // 立即前进, onTick 不会重入
        act.sendNavigateWithQuat(ctx.entry_approach_x, by, 0, 0, 0, 0, 1, ctx);
        break;
    }
    case 1:
        // 等待 approach NAV_DONE — 什么都不做, 由 handleSubEvent 推进到 step 2
        break;
    case 2:
        // 等待 forward NAV_DONE / UP_JUECE_DONE — 由 handleSubEvent 推进到 step 3
        break;
    case 3:
        // 撤退完成, 收起机械臂, 进入正常 Zone2 导航
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "EntryGrab step3: retreat done, retract");
        act.stopEntryGrab();
        ctx.entry_grab_step = 0;
        sub_ = Sub::NAV_POINT;
        enterSub(ctx, act);
        break;
    }
}

/*
 * tickGrab — 固定路线抓取的步骤推进
 *   非 Point0: step 0(backoff) → step 1(grab start) → step 2(wait UP_JUECE)
 *   Point0:    step 0(grab + forward) → step 1(wait UP_JUECE) → step 3(retreat → Point0序列)
 */
void Zone2State::tickGrab(Context &ctx, ActionDispatcher &act)
{
    if (ctx.zone2_point0_sequence_active)
    {
        handlePoint0Substep(ctx, act); // Point0 序列激活, 继续推进 substep
        return;
    }

    const auto &t = ctx.zone2_tasks[ctx.zone2_index];

    // Point0 抓取完成后退到了 approach → 启动多步序列
    if (t.id == 0 && ctx.zone2_grab_step == 3)
    {
        ctx.zone2_point0_sequence_active = true;
        ctx.zone2_grab_step = 0;
        ctx.zone2_point0_substep = 0;
        handlePoint0Substep(ctx, act);
        return;
    }

    // 非 Point0: step 0/1 在 enterSub 中处理, step 2 等 UP_JUECE
}

/*
 * Point0 多步序列 (抓完第一个方块后的特殊流程):
 *   substep 0: 上台阶 #1
 *   substep 1: 导航到旋转点
 *   substep 2: 原地右转 (绕 Z 轴 -90°)
 *   substep 3: 上台阶 #2
 *   substep 4: 转回去 (恢复朝向)
 *   substep 5: 上台阶 #3
 *   substep 6: 全部完成, 进入下一个任务点
 */
void Zone2State::handlePoint0Substep(Context &ctx, ActionDispatcher &act)
{
    const auto &t = ctx.zone2_tasks[ctx.zone2_index];
    double rx = (t.rotate_x != 0.0) ? t.rotate_x : t.approach_x;
    double ry = (t.rotate_y != 0.0) ? t.rotate_y : t.approach_y;

    switch (ctx.zone2_point0_substep)
    {
    case 0:
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 substep 0: UP_STAIRS #1");
        act.startStair(1, ctx, StairContext::POINT0);
        break;
    case 1:
        if (ctx.point0_nav_sent || ctx.nav_chain_in_progress)
            break;
        ctx.point0_nav_sent = true;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 substep 1: NAV to (%.2f,%.2f)", rx, ry);
        act.sendNavigateWithQuat(rx, ry, 0, 0, 0, 0, 1, ctx);
        break;
    case 2:
        if (ctx.nav_chain_in_progress)
            break;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 substep 2: ROTATE right (-90 deg yaw)");
        act.sendNavigateWithQuat(rx, ry, 0, t.rqx, t.rqy, t.rqz, t.rqw, ctx);
        break;
    case 3:
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 substep 3: UP_STAIRS #2");
        act.startStair(1, ctx, StairContext::POINT0);
        break;
    case 4:
        if (ctx.nav_chain_in_progress)
            break;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 substep 4: ROTATE back (restore orientation)");
        act.sendNavigateWithQuat(rx, ry, 0, t.qx, t.qy, t.qz, t.qw, ctx);
        break;
    case 5:
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 substep 5: UP_STAIRS #3");
        act.startStair(1, ctx, StairContext::POINT0);
        break;
    case 6:
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 all substeps done, advance to next point");
        ctx.zone2_point0_sequence_active = false;
        ctx.zone2_point0_substep = 0;
        ctx.point0_nav_sent = false;
        ++ctx.zone2_index;
        sub_ = Sub::NAV_POINT;
        enterSub(ctx, act);
        break;
    }
}

// 动态路线场景确认超时
void Zone2State::checkSceneTimeout(Context &ctx, ActionDispatcher &act)
{
    auto elapsed = (rclcpp::Clock().now() - ctx.scene_confirm_start_time).seconds();
    if (elapsed > ctx.scene_confirm_timeout_s)
        act.postEvent({EventType::SCENE_CONFIRM_TIMEOUT});
}

// ==========================================================================
// ExitState — 动态路线结束: 导航到梅花林出口
//   导航完成 → Done
// ==========================================================================

std::unique_ptr<TopState> ExitState::onEnter(Context &ctx, ActionDispatcher &act)
{
    act.enableGrabScene(false);
    RCLCPP_INFO(rclcpp::get_logger("fsm"), "Exit: nav to MF exit (%.2f, %.2f)", ctx.mf_exit_x, ctx.mf_exit_y);
    act.sendNavigateWithQuat(ctx.mf_exit_x, ctx.mf_exit_y, ctx.mf_exit_z, 0, 0, 0, 1, ctx);
    return nullptr;
}

std::unique_ptr<TopState> ExitState::handleEvent(Context &ctx, ActionDispatcher &act, const Event &e)
{
    (void)ctx;
    (void)act;
    if (e.type == EventType::NAV_DONE)
    {
        if (!e.success)
            RCLCPP_WARN(rclcpp::get_logger("fsm"), "Exit nav failed");
        return std::make_unique<DoneState>();
    }
    return nullptr;
}

// ==========================================================================
// DoneState — 终点: 关掉所有传感器, 不接受任何事件
// ==========================================================================

std::unique_ptr<TopState> DoneState::onEnter(Context &ctx, ActionDispatcher &act)
{
    act.enableSpear(false);
    act.enableLightboard(false);
    act.enableGrabScene(false);
    RCLCPP_INFO(rclcpp::get_logger("fsm"), "=== MISSION COMPLETE ===");
    (void)ctx;
    return nullptr;
}

std::unique_ptr<TopState> DoneState::handleEvent(Context &ctx, ActionDispatcher &act, const Event &e)
{
    (void)ctx;
    (void)act;
    (void)e;
    return nullptr; // 终态, 不接受任何事件
}
