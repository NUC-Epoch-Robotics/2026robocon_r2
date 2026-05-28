#include "r2_decision/r2_decision_node.hpp"

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
 *                            └── 超时 → UP_STAIRS → DOWN_STAIRS → Zone2
 *
 *   每个大状态内部用 sub_ 枚举做子状态切换:
 *     enterSub()  = 进入子状态时执行动作 (发导航、发机械臂指令、启台阶)
 *     handleSubEvent() = 事件来了判断能不能转下一个子状态
 *     onTick()    = 每 20ms 检查超时、推进不需要等事件的步骤
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
    if (!current_) return;
    auto next = current_->onTick(ctx, act);
    if (next)
        transitionTo(std::move(next), ctx, act);
}

void StateMachine::handleEvent(Context &ctx, ActionDispatcher &act, const Event &e)
{
    if (!current_) return;
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
        current_->onExit(ctx, act);  // 离开旧状态, 清理资源
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
    return std::make_unique<WaitStartState>();  // 链式切换, 不等待任何事件
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
    return nullptr;  // 返回 nullptr = 停在这里等事件
}

std::unique_ptr<TopState> WaitStartState::handleEvent(Context &ctx, ActionDispatcher &act, const Event &e)
{
    (void)act;
    if (e.type == EventType::START_PRESSED)
    {
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
 * Zone1State — 一区: 6 个矛头点 + 对接站流程
 * ============================================================================
 *
 *   子状态流转 (固定路线):
 *     NAV_POINT ─→ OPERATE ─→ DOCK ─→ NAV_POINT (下一个矛头点)
 *                    ↑                    │
 *                    └── 跳过对接 ←───────┘
 *                                         所有点完成
 *                                            ↓
 *     FINISH ─→ Zone2                     UP_STAIRS ─→ DOWN_STAIRS ─→ Zone2
 *
 *   NAV_POINT:  导航到矛头点坐标
 *   OPERATE:    发机械臂指令打矛头 (arm_command)
 *   DOCK:       导航到对接站, 等 spearhead 消失 = 对接成功
 *   UP_STAIRS:  上台阶 (耗时步骤, 脱离一区)
 *   DOWN_STAIRS: 下台阶
 *   FINISH:     固定路线直接结束, 动态路线走上/下台阶
 * ============================================================================
 */

const char *Zone1State::name() const
{
    switch (sub_)
    {
    case Sub::NAV_POINT:           return "Zone1/NavPoint";
    case Sub::OPERATE:             return "Zone1/Operate";
    case Sub::DOCK:                return "Zone1/Dock";
    case Sub::SPEARHEAD_POST_DOCK: return "Zone1/SpearheadPostDock";
    case Sub::UP_STAIRS:           return "Zone1/UpStairs";
    case Sub::DOWN_STAIRS:         return "Zone1/DownStairs";
    case Sub::FINISH:              return "Zone1/Finish";
    }
    return "Zone1";
}

// 进入 Zone1: 从第一个矛头点开始
std::unique_ptr<TopState> Zone1State::onEnter(Context &ctx, ActionDispatcher &act)
{
    sub_ = Sub::NAV_POINT;
    enterSub(ctx, act);
    return nullptr;
}

// Zone1 每 20ms 的检查:
//   1. DOCK 子状态: 监控 spearhead 是否消失 (对接成功)
//   2. 全局超时检查: Zone1 超过 zone1_max_time_s 就强制结束
//   3. FINISH 子状态: 构建 Zone2 任务列表, 切换到 Zone2
std::unique_ptr<TopState> Zone1State::onTick(Context &ctx, ActionDispatcher &act)
{
    if (sub_ == Sub::DOCK)
        checkDockTransition(ctx, act);   // 等导航到对接站后, 监控 spearhead

    // 抓矛头对接后: step1 等 5s → 发 zhuangtai=0
    if (sub_ == Sub::SPEARHEAD_POST_DOCK && ctx.spearhead_post_dock_step == 1)
    {
        auto elapsed = (rclcpp::Clock().now() - ctx.spearhead_post_dock_start).seconds();
        if (elapsed > 5.0)
        {
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Spearhead post-dock: 5s elapsed, send zhuangtai=0");
            ctx.spearhead_post_dock_step = 2;
            act.sendSpearheadCommand(0);
        }
    }

    checkTimeLimit(ctx, act);            // 全局 120s 超时

    if (sub_ == Sub::FINISH)
    {
        // 决策树: 根据固定/动态路线构建 Zone2 任务列表
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
        if (ctx.nav_chain_in_progress) return nullptr;  // 导航中不能重试
        ctx.zone1_index = 0;
        ctx.dock_success_count = 0;
        ctx.zone1_arm_retry = 0;
        ctx.zone1_start_time = rclcpp::Clock().now();
        act.enableSpear(true);
        sub_ = Sub::NAV_POINT;
        enterSub(ctx, act);
        return nullptr;
    }
    if (e.type == EventType::ZONE1_TIMEOUT)
    {
        // 超时: 关传感器, 直接上台阶离开一区
        act.enableSpear(false);
        act.enableLightboard(false);
        act.stopStair();
        sub_ = Sub::UP_STAIRS;
        enterSub(ctx, act);
        return nullptr;
    }
    return handleSubEvent(ctx, act, e);
}

void Zone1State::onExit(Context &ctx, ActionDispatcher &act)
{
    (void)ctx;
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
    case Sub::NAV_POINT:
    {
        // 所有矛头点都走完了?
        if (ctx.zone1_index >= ctx.zone1_route_ids.size())
        {
            // 固定路线 → FINISH (在 onTick 中构建 Zone2 任务)
            // 动态路线 → UP_STAIRS → DOWN_STAIRS
            sub_ = ctx.use_fixed_zone2_route ? Sub::FINISH : Sub::UP_STAIRS;
            enterSub(ctx, act);
            return;
        }
        // 从 point_table 取当前点的坐标
        const int pid = ctx.zone1_route_ids[ctx.zone1_index];
        auto it = ctx.point_table.find(pid);
        if (it == ctx.point_table.end())
        {
            RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1: missing point id=%d, skip", pid);
            ++ctx.zone1_index;
            enterSub(ctx, act);
            return;
        }
        const auto &t = it->second;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: nav to point %d (%.2f, %.2f)", t.id, t.x, t.y);
        act.sendNavigateWithQuat(t.x, t.y, t.z, 0, 0, 0, 1, ctx);
        break;
    }
    case Sub::OPERATE:
    {
        // 导航到了矛头点, 判断是抓矛头还是打矛头
        const int pid = ctx.zone1_route_ids[ctx.zone1_index];
        auto it = ctx.point_table.find(pid);
        const auto &t = it->second;

        // use_spearhead: 抓矛头流程 (zhuangtai=1)
        if (t.use_spearhead)
        {
            ctx.zone1_arm_retry = 0;
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1 point %d: spearhead grab START", pid);
            act.sendSpearheadCommand(1);  // zhuangtai=1 → 开始抓矛头等对接
            break;
        }

        // arm_command==0 表示这个点不需要打 (纯导航点)
        if (t.arm_command == 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1 point %d: arm cmd=0, skip", pid);
            ++ctx.zone1_index;
            sub_ = Sub::NAV_POINT;
            enterSub(ctx, act);
            return;
        }
        // 摄像头没检测到矛头 → 跳过
        if (!t.skip_dock && !ctx.spearhead_exists)
        {
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1 point %d: no spearhead, skip", pid);
            ++ctx.zone1_index;
            sub_ = Sub::NAV_POINT;
            enterSub(ctx, act);
            return;
        }
        ctx.zone1_arm_retry = 0;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1 point %d: arm cmd=%d", pid, t.arm_command);
        act.sendArmCommand(t.arm_command);
        break;
    }
    case Sub::DOCK:
    {
        // 打完矛头 → 导航到对接站
        ctx.dock_arrived = false;
        act.enableLightboard(true);  // 开灯板摄像头, 记录灯板数据
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: nav to R1 dock (%.2f, %.2f)",
                    ctx.dock_r1_x, ctx.dock_r1_y);
        act.sendNavigateWithQuat(ctx.dock_r1_x, ctx.dock_r1_y, ctx.dock_r1_z, 0, 0, 0, 1, ctx);
        break;
    }
    case Sub::SPEARHEAD_POST_DOCK:
    {
        // 对接完成后: 发 zhuangtai=2, 等 ARM_DONE 后进入 5s 倒计时
        ctx.spearhead_post_dock_step = 0;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Spearhead post-dock: send zhuangtai=2");
        act.sendSpearheadCommand(2);
        break;
    }
    case Sub::UP_STAIRS:
    {
        // 上台阶 → 持续发 stair_cmd=1
        act.enableSpear(false);
        act.enableLightboard(false);
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: UP_STAIRS");
        act.startStair(1, ctx);
        break;
    }
    case Sub::DOWN_STAIRS:
    {
        // 下台阶 → 持续发 stair_cmd=2
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone1: DOWN_STAIRS");
        act.startStair(2, ctx);
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
    case Sub::NAV_POINT:
        // 导航完成 → 进入操作子状态 (打矛头)
        if (e.type == EventType::NAV_DONE)
        {
            if (!e.success)
            {
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1: nav to point failed, skip");
                ++ctx.zone1_index;       // 导航失败, 跳过这个点
            }
            else
            {
                sub_ = Sub::OPERATE;
            }
            enterSub(ctx, act);
        }
        break;

    case Sub::OPERATE:
        // 机械臂动作完成 (抓矛头 DONE 也走这里, 事件类型同为 ARM_DONE)
        if (e.type == EventType::ARM_DONE)
        {
            const int pid = ctx.zone1_route_ids[ctx.zone1_index];
            auto it = ctx.point_table.find(pid);
            const bool skip_dock = (it != ctx.point_table.end()) && it->second.skip_dock;
            const bool use_spearhead = (it != ctx.point_table.end()) && it->second.use_spearhead;

            // 失败且还有重试次数 → 重发指令 (区分 arm/spearhead)
            if (!e.success && ctx.zone1_arm_retry < kZone1ArmMaxRetry)
            {
                ++ctx.zone1_arm_retry;
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1 retry %d/%d (pid=%d spearhead=%d)",
                            ctx.zone1_arm_retry, kZone1ArmMaxRetry, pid, use_spearhead);
                if (use_spearhead)
                    act.sendSpearheadCommand(1);
                else
                    act.sendArmCommand(e.command);
                break;
            }
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone1 %s failed after retry, skip",
                            use_spearhead ? "spearhead" : "arm");

            ctx.zone1_arm_retry = 0;
            // skip_dock 的点 (纯导航点) 直接下一个, 否则去对接站
            if (skip_dock)
            {
                ++ctx.zone1_index;
                sub_ = Sub::NAV_POINT;
            }
            else
            {
                sub_ = Sub::DOCK;
            }
            enterSub(ctx, act);
        }
        break;

    case Sub::DOCK:
        // 对接成功: spearhead 消失
        if (e.type == EventType::DOCK_SUCCESS)
        {
            ++ctx.dock_success_count;
            RCLCPP_INFO(rclcpp::get_logger("fsm"), ">>> DOCK SUCCESS #%d <<<", ctx.dock_success_count);

            // 保存灯板数据 (给动态路线用)
            if (ctx.lightboard_map_received)
            {
                ctx.latest_lightboard_map = ctx.lightboard_map;
                RCLCPP_INFO(rclcpp::get_logger("fsm"), "Lightboard map captured: %zu values",
                            ctx.latest_lightboard_map.size());
            }

            // 检查当前点是否抓矛头 → 进入对接后流程 (发2→等5s→发0)
            const int pid = ctx.zone1_route_ids[ctx.zone1_index];
            auto it = ctx.point_table.find(pid);
            bool is_spearhead = (it != ctx.point_table.end()) && it->second.use_spearhead;

            // 对接次数 < 3 且还有下一个点 → 继续; 否则离开
            bool keep_going = (ctx.dock_success_count < kMaxDocks) &&
                              (ctx.zone1_index + 1 < ctx.zone1_route_ids.size());
            if (keep_going)
            {
                ++ctx.zone1_index;
                sub_ = Sub::NAV_POINT;
            }
            else if (is_spearhead)
            {
                // 抓矛头对接完成 → 发2→等5s→发0 → 然后再上台阶
                sub_ = Sub::SPEARHEAD_POST_DOCK;
            }
            else
            {
                sub_ = Sub::UP_STAIRS;
            }
            enterSub(ctx, act);
        }
        // 对接超时: 跳过这个点, 继续下一个
        else if (e.type == EventType::DOCK_TIMEOUT)
        {
            RCLCPP_WARN(rclcpp::get_logger("fsm"), "Dock timeout, skip");
            ++ctx.zone1_index;
            sub_ = Sub::NAV_POINT;
            enterSub(ctx, act);
        }
        break;

    case Sub::SPEARHEAD_POST_DOCK:
        // 对接后抓矛头步骤: step0→发2后等ARM_DONE, step1→等5s(onTick), step2→发0后等ARM_DONE
        if (e.type == EventType::ARM_DONE)
        {
            if (ctx.spearhead_post_dock_step == 0)
            {
                // zhuangtai=2 完成 → 开始等 5s
                ctx.spearhead_post_dock_step = 1;
                ctx.spearhead_post_dock_start = rclcpp::Clock().now();
                RCLCPP_INFO(rclcpp::get_logger("fsm"), "Spearhead post-dock: zhuangtai=2 done, waiting 5s...");
            }
            else if (ctx.spearhead_post_dock_step == 2)
            {
                // zhuangtai=0 完成 → 直接进 Zone2 入口抓取
                ctx.spearhead_post_dock_step = 0;
                RCLCPP_INFO(rclcpp::get_logger("fsm"), "Spearhead post-dock: zhuangtai=0 done, go Zone2");
                sub_ = Sub::FINISH;
                enterSub(ctx, act);
            }
        }
        break;

    case Sub::UP_STAIRS:
        // 上台阶完成 → 下台阶
        if (e.type == EventType::DOWN_JUECE_DONE)
        {
            sub_ = Sub::DOWN_STAIRS;
            enterSub(ctx, act);
        }
        break;

    case Sub::DOWN_STAIRS:
        // 下台阶完成 → 构建 Zone2 任务, 切换到 Zone2
        if (e.type == EventType::DOWN_JUECE_DONE)
        {
            act.stopStair();
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
        break;

    case Sub::FINISH:
        break;
    }
    return nullptr;
}

/*
 * checkDockTransition — 在 onTick 中持续监控对接状态
 * 逻辑: 导航到了对接站 → 记录 spearhead 存在 → 等它消失或超时
 */
void Zone1State::checkDockTransition(Context &ctx, ActionDispatcher &act)
{
    if (ctx.nav_chain_in_progress) return;  // 还没到对接站

    if (!ctx.dock_arrived)
    {
        // 刚到达对接站, 记录初始状态
        ctx.dock_arrived = true;
        ctx.spearhead_was_present = ctx.spearhead_exists;
        ctx.dock_start_time = rclcpp::Clock().now();
        RCLCPP_INFO(rclcpp::get_logger("fsm"),
                    "Arrived at R1 dock. spearhead=%d", ctx.spearhead_was_present);
        return;
    }

    // spearhead 从有变无 = 对接成功
    if (ctx.spearhead_was_present && !ctx.spearhead_exists)
    {
        act.postEvent({EventType::DOCK_SUCCESS});
        return;
    }

    // 超时
    auto elapsed = (rclcpp::Clock().now() - ctx.dock_start_time).seconds();
    if (elapsed > ctx.dock_timeout_s)
    {
        act.postEvent({EventType::DOCK_TIMEOUT});
    }
}

// 全局一区超时: 超过 zone1_max_time_s (默认120s) 发出 ZONE1_TIMEOUT 事件
void Zone1State::checkTimeLimit(Context &ctx, ActionDispatcher &act)
{
    if (sub_ == Sub::UP_STAIRS || sub_ == Sub::DOWN_STAIRS || sub_ == Sub::FINISH ||
        sub_ == Sub::SPEARHEAD_POST_DOCK)
        return;  // 已经在离开步骤了, 不打断

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
    case Sub::ENTRY_GRAB: return "Zone2/EntryGrab";
    case Sub::NAV_POINT:  return "Zone2/NavPoint";
    case Sub::ROTATE:     return "Zone2/Rotate";
    case Sub::WAIT_SCENE: return "Zone2/WaitScene";
    case Sub::GRAB:       return "Zone2/Grab";
    case Sub::UP_STAIRS:  return "Zone2/UpStairs";
    case Sub::DOWN_STAIRS:return "Zone2/DownStairs";
    case Sub::FINISH:     return "Zone2/Finish";
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

    // 固定路线: 先做入口抓取 (两个块), 再逐点导航
    // 动态路线: 直接从第一个任务点开始
    if (ctx.use_fixed_zone2_route)
    {
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: %zu tasks, start entry grab", ctx.zone2_tasks.size());
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
    // 入口抓取: 4 步序列由 tickEntryGrab 驱动 (部分步骤不需要等事件)
    if (sub_ == Sub::ENTRY_GRAB)
        tickEntryGrab(ctx, act);
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
        // 有 approach 坐标 = 需要抓取, 先导航到 approach 位置而非最终位置
        bool has_grab = (t.approach_x != 0.0 || t.approach_y != 0.0);
        double nx = has_grab ? t.approach_x : t.x;
        double ny = has_grab ? t.approach_y : t.y;

        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: nav to point %d (%.2f,%.2f) z=%.3f stair=%d grab=%d",
                    t.id, nx, ny, t.z, t.stair_cmd, has_grab);
        act.sendNavigateWithQuat(nx, ny, t.z, t.qx, t.qy, t.qz, t.qw, ctx);
        break;
    }

    case Sub::ROTATE:
    {
        // 原地旋转: 导航到同一位置, 但用旋转四元数 (rqx,rqy,rqz,rqw)
        const auto &t = ctx.zone2_tasks[ctx.zone2_index];
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Zone2: ROTATE @ (%.2f,%.2f) q=(%.3f,%.3f,%.3f,%.3f)",
                    t.x, t.y, t.rqx, t.rqy, t.rqz, t.rqw);
        act.sendNavigateWithQuat(t.x, t.y, t.z, t.rqx, t.rqy, t.rqz, t.rqw, ctx);
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
        // 入口抓取: NAV_DONE 推进 tickEntryGrab 的步骤
        if (e.type == EventType::NAV_DONE)
        {
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "EntryGrab: nav failed");
            tickEntryGrab(ctx, act);
        }
        // UP_JUECE_DONE = 抓取完成, 后退回 approach 位置
        else if (e.type == EventType::UP_JUECE_DONE)
        {
            ctx.grab_context = GrabContext::NONE;
            ctx.entry_grab_step = 3;  // 跳到撤退步骤
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "EntryGrab: done, retreat to approach");
            act.sendNavigateWithQuat(ctx.entry_approach_x, ctx.entry_block0_y, 0, 0, 0, 0, 1, ctx);
        }
        break;

    case Sub::NAV_POINT:
        // 导航到了下一个点 → 根据任务属性决定下一步
        if (e.type == EventType::NAV_DONE)
        {
            if (!e.success)
            {
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone2: nav failed, skip");
                ++ctx.zone2_index;
                enterSub(ctx, act);
                break;
            }
            const auto &t = ctx.zone2_tasks[ctx.zone2_index];
            bool has_grab = (t.approach_x != 0.0 || t.approach_y != 0.0);

            if (ctx.use_fixed_zone2_route)
            {
                // 决策: 有 approach → 抓取; 有 rotate → 旋转; 有 stair → 台阶
                if (has_grab)
                {
                    ctx.zone2_grab_step = 0;
                    ctx.zone2_point0_substep = 0;
                    sub_ = Sub::GRAB;
                }
                else if (t.use_rotate)
                    sub_ = Sub::ROTATE;
                else if (t.stair_cmd == 1)
                    sub_ = Sub::UP_STAIRS;
                else if (t.stair_cmd == 2)
                    sub_ = Sub::DOWN_STAIRS;
                else
                    sub_ = Sub::FINISH;
            }
            else
            {
                // 动态路线: 导航到了 → 等场景确认
                sub_ = Sub::WAIT_SCENE;
            }
            enterSub(ctx, act);
        }
        break;

    case Sub::ROTATE:
        // 旋转完 → 有台阶做台阶, 没台阶下一个点
        if (e.type == EventType::NAV_DONE)
        {
            const auto &t = ctx.zone2_tasks[ctx.zone2_index];
            if (!e.success)
                RCLCPP_WARN(rclcpp::get_logger("fsm"), "Zone2: rotate nav failed");
            if (t.stair_cmd == 1 || t.stair_cmd == 2)
            {
                ctx.zone2_post_rotate_stairs_done = true;
                sub_ = (t.stair_cmd == 1) ? Sub::UP_STAIRS : Sub::DOWN_STAIRS;
            }
            else
            {
                ++ctx.zone2_index;
                sub_ = Sub::NAV_POINT;
            }
            enterSub(ctx, act);
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
            if (ss == 1)       ctx.zone2_point0_substep = 2;
            else if (ss == 2)  ctx.zone2_point0_substep = 3;
            else if (ss == 4)  ctx.zone2_point0_substep = 5;
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
                ctx.zone2_grab_step = 3;  // 标记: 下一步是 Point0 序列
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
            ++ctx.zone2_index;
            sub_ = Sub::NAV_POINT;
            enterSub(ctx, act);
        }
        break;

    case Sub::UP_STAIRS:
    case Sub::DOWN_STAIRS:
        // 台阶完成
        if (e.type == EventType::DOWN_JUECE_DONE)
        {
            act.stopStair();
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "Stair done");

            // 旋转+台阶组合: 台阶是旋转之后做的, 做完了继续下一个点
            if (ctx.zone2_post_rotate_stairs_done)
            {
                ctx.zone2_post_rotate_stairs_done = false;
                ++ctx.zone2_index;
                sub_ = Sub::NAV_POINT;
            }
            else
            {
                const auto &t = ctx.zone2_tasks[ctx.zone2_index];
                // Point0 序列中的台阶: 推进 substep, 回到 GRAB 继续下一步
                if (ctx.zone2_point0_sequence_active)
                {
                    ++ctx.zone2_point0_substep;
                    sub_ = Sub::GRAB;
                }
                // 台阶后需要旋转
                else if (t.use_rotate)
                {
                    sub_ = Sub::ROTATE;
                }
                // 纯台阶: 下一个点
                else
                {
                    ++ctx.zone2_index;
                    sub_ = Sub::NAV_POINT;
                }
            }
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
        // 导航到 approach 位置
        double by = ctx.entry_block0_y;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "EntryGrab step0: nav approach (%.2f,%.2f)",
                    ctx.entry_approach_x, by);
        ctx.entry_grab_step = 1;
        act.sendNavigateWithQuat(ctx.entry_approach_x, by, 0, 0, 0, 0, 1, ctx);
        break;
    }
    case 1:
        // 导航到 approach 的 NAV_DONE 回来后进入这里
        if (ctx.grab_context != GrabContext::ENTRY && ctx.entry_grab_step == 1)
        {
            double bx = ctx.entry_block0_x;
            double by = ctx.entry_block0_y;
            RCLCPP_INFO(rclcpp::get_logger("fsm"), "EntryGrab step1: START is_finsh=%d, forward to block",
                        ctx.entry_block0_is_finsh);
            act.startEntryGrab(ctx.entry_block0_is_finsh, ctx);
            ctx.entry_grab_step = 2;
            act.sendNavigateWithQuat(bx, by, 0, 0, 0, 0, 1, ctx);
        }
        break;
    case 2:
        // 等 UP_JUECE_DONE, 在 handleSubEvent 中处理
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
        handlePoint0Substep(ctx, act);  // Point0 序列激活, 继续推进 substep
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
        if (ctx.point0_nav_sent || ctx.nav_chain_in_progress) break;
        ctx.point0_nav_sent = true;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 substep 1: NAV to (%.2f,%.2f)", rx, ry);
        act.sendNavigateWithQuat(rx, ry, 0, 0, 0, 0, 1, ctx);
        break;
    case 2:
        if (ctx.nav_chain_in_progress) break;
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 substep 2: ROTATE right (-90 deg yaw)");
        act.sendNavigateWithQuat(rx, ry, 0, t.rqx, t.rqy, t.rqz, t.rqw, ctx);
        break;
    case 3:
        RCLCPP_INFO(rclcpp::get_logger("fsm"), "Point0 substep 3: UP_STAIRS #2");
        act.startStair(1, ctx, StairContext::POINT0);
        break;
    case 4:
        if (ctx.nav_chain_in_progress) break;
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
    return nullptr;  // 终态, 不接受任何事件
}
