#include <algorithm>

#include "r2_decision/r2_decision_node.hpp"

namespace
{
// ── 邻接表 ──
constexpr int kAdj[12][4] = {
    {1, 3, -1, -1},  // 0 (右列)
    {0, 2, 4, -1},   // 1 (中列, 入口)
    {1, 5, -1, -1},  // 2 (左列)
    {0, 4, 6, -1},   // 3 (右列)
    {1, 3, 5, 7},    // 4 (中列)
    {2, 4, 8, -1},   // 5 (左列)
    {3, 7, 9, -1},   // 6 (右列)
    {4, 6, 8, 10},   // 7 (中列)
    {5, 7, 11, -1},  // 8 (左列)
    {6, 10, -1, -1}, // 9 (右列, 出口)
    {7, 9, 11, -1},  // 10 (中列)
    {8, 10, -1, -1}, // 11 (左列, 出口)
};

// ── 高度表 ──
//  右0(2)  1(1)  2(2)左
//  右3(3)  4(2)  5(1)左
//  右6(2)  7(3)  8(2)左
//  右9(1)  10(2) 11(1)左
constexpr int kHeight[12] = {2, 1, 2, 3, 2, 1, 2, 3, 2, 1, 2, 1};

// ── 硬编码动作表 ──
// rotation: -1=逆时针90°, 0=不转, 1=顺时针90°
// stair: 0=不需要, 1=上台阶, 2=下台阶
// grab: 0=收回, 1=低抓高差1层, 2=低抓高差2层, 3=高抓低
struct BlockAction
{
    int target;     // 目标格子
    int rotation;   // 旋转方向
    int stair;      // 台阶指令
    int grab;       // 抓取指令
};

// 每个格子的动作表
constexpr BlockAction kBlock0Actions[] = {
    {1, -1, 2, 3},  // 格子1: 逆时针, 下台阶, 高抓低
    {3,  0, 1, 1},  // 格子3: 不转, 上台阶, 低抓高差1层
};
constexpr BlockAction kBlock1Actions[] = {
    {0,  1, 1, 1},  // 格子0: 顺时针, 上台阶, 低抓高差1层
    {2, -1, 1, 1},  // 格子2: 逆时针, 上台阶, 低抓高差1层
    {4,  0, 1, 1},  // 格子4: 不转, 上台阶, 低抓高差1层
};
constexpr BlockAction kBlock2Actions[] = {
    {1,  1, 2, 3},  // 格子1: 顺时针, 下台阶, 高抓低
    {5,  0, 2, 3},  // 格子5: 不转, 下台阶, 高抓低
};
constexpr BlockAction kBlock3Actions[] = {
    {4, -1, 2, 3},  // 格子4: 逆时针, 下台阶, 高抓低
    {6,  0, 2, 3},  // 格子6: 不转, 下台阶, 高抓低
};
constexpr BlockAction kBlock4Actions[] = {
    {3,  1, 1, 1},  // 格子3: 顺时针, 上台阶, 低抓高差1层
    {5, -1, 2, 3},  // 格子5: 逆时针, 下台阶, 高抓低
    {7,  0, 1, 1},  // 格子7: 不转, 上台阶, 低抓高差1层
};
constexpr BlockAction kBlock5Actions[] = {
    {4,  1, 1, 1},  // 格子4: 顺时针, 上台阶, 低抓高差1层
    {8,  0, 1, 1},  // 格子8: 不转, 上台阶, 低抓高差1层
};
constexpr BlockAction kBlock6Actions[] = {
    {7, -1, 1, 1},  // 格子7: 逆时针, 上台阶, 低抓高差1层
    {9,  0, 2, 3},  // 格子9: 不转, 下台阶, 高抓低
};
constexpr BlockAction kBlock7Actions[] = {
    {6,  1, 2, 3},  // 格子6: 顺时针, 下台阶, 高抓低
    {8, -1, 2, 3},  // 格子8: 逆时针, 下台阶, 高抓低
    {10, 0, 2, 3},  // 格子10: 不转, 下台阶, 高抓低
};
constexpr BlockAction kBlock8Actions[] = {
    {7,  1, 1, 1},  // 格子7: 顺时针, 上台阶, 低抓高差1层
    {11, 0, 2, 3},  // 格子11: 不转, 下台阶, 高抓低
};
constexpr BlockAction kBlock9Actions[] = {
    {10, -1, 1, 1}, // 格子10: 逆时针, 上台阶, 低抓高差1层
};
constexpr BlockAction kBlock10Actions[] = {
    {9,  1, 2, 3},  // 格子9: 顺时针, 下台阶, 高抓低
    {11, -1, 2, 3}, // 格子11: 逆时针, 下台阶, 高抓低
};
constexpr BlockAction kBlock11Actions[] = {
    {10, 1, 1, 1},  // 格子10: 顺时针, 上台阶, 低抓高差1层
};

// 指向每个格子的动作表
constexpr const BlockAction* kBlockActions[12] = {
    kBlock0Actions,  kBlock1Actions,  kBlock2Actions,
    kBlock3Actions,  kBlock4Actions,  kBlock5Actions,
    kBlock6Actions,  kBlock7Actions,  kBlock8Actions,
    kBlock9Actions,  kBlock10Actions, kBlock11Actions,
};
constexpr int kBlockActionCount[12] = {2, 3, 2, 2, 3, 2, 2, 3, 2, 1, 2, 1};

// 查找从 current 到 target 的动作
const BlockAction* findAction(int current, int target)
{
    for (int i = 0; i < kBlockActionCount[current]; ++i)
    {
        if (kBlockActions[current][i].target == target)
            return &kBlockActions[current][i];
    }
    return nullptr;
}

bool isEntryBlock(int idx) { return idx == 1; }  // 入口只有格子1
bool isExitBlock(int idx) { return idx == 9 || idx == 11; }

int findFirstStep(int pos, int target, const bool passable[12])
{
    if (pos < 0 && isEntryBlock(target))
        return -1;

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
            int nb = kAdj[cur][ni];
            if (nb < 0 || prev[nb] >= -1 || !passable[nb])
                continue;
            prev[nb] = cur;
            queue[tail++] = nb;
        }
    }

    if (!found)
        return target;

    int step = target;
    while (prev[step] >= 0)
        step = prev[step];

    return step;
}

bool isAdjacent(int a, int b)
{
    if (b < 0)
        return isEntryBlock(a);
    for (int ni = 0; ni < 4; ++ni)
        if (kAdj[a][ni] == b)
            return true;
    return false;
}

bool isAlreadyPlanned(const std::vector<int> &plan, int idx)
{
    for (int p : plan)
        if (p == idx)
            return true;
    return false;
}
} // namespace

void R2DecisionNode::buildZone2FixedRoute(Context &ctx)
{
    ctx.zone2_tasks.clear();

    for (int i = 0; i < ctx.zone2_fixed_count; ++i)
    {
        Zone2Task t;
        t.id = i;
        t.x = ctx.zone2_fixed[i].x;
        t.y = ctx.zone2_fixed[i].y;
        t.z = ctx.zone2_fixed[i].z;
        t.qx = ctx.zone2_fixed[i].qx;
        t.qy = ctx.zone2_fixed[i].qy;
        t.qz = ctx.zone2_fixed[i].qz;
        t.qw = ctx.zone2_fixed[i].qw;
        t.use_rotate = ctx.zone2_fixed[i].use_rotate;
        t.rqx = ctx.zone2_fixed[i].rqx;
        t.rqy = ctx.zone2_fixed[i].rqy;
        t.rqz = ctx.zone2_fixed[i].rqz;
        t.rqw = ctx.zone2_fixed[i].rqw;
        t.approach_x = ctx.zone2_fixed[i].approach_x;
        t.approach_y = ctx.zone2_fixed[i].approach_y;
        t.block_height = ctx.zone2_fixed[i].block_height;
        t.stand_height = ctx.zone2_fixed[i].stand_height;
        t.rotate_x = ctx.zone2_fixed[i].rotate_x;
        t.rotate_y = ctx.zone2_fixed[i].rotate_y;
        t.grab_scene = 0;
        t.arm_command = 0;
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

        if (ctx.zone2_fixed[i].stair_cmd != 0)
        {
            t.stair_cmd = ctx.zone2_fixed[i].stair_cmd;
        }
        else if (i + 1 < ctx.zone2_fixed_count)
        {
            double dz = ctx.zone2_fixed[i + 1].z - ctx.zone2_fixed[i].z;
            t.stair_cmd = (dz > 0) ? 1 : 2;
        }
        else
        {
            t.stair_cmd = 0;
        }

        ctx.zone2_tasks.push_back(t);
    }

    RCLCPP_INFO(rclcpp::get_logger("planner"), "buildZone2FixedRoute: %zu tasks:", ctx.zone2_tasks.size());
    for (size_t i = 0; i < ctx.zone2_tasks.size(); ++i)
    {
        const auto &t = ctx.zone2_tasks[i];
        RCLCPP_INFO(rclcpp::get_logger("planner"),
                    "  #%zu (%.2f,%.2f) z=%.3f stair=%d app=(%.2f,%.2f) "
                    "h_stand=%d h_block=%d is_finsh=%d rot=%d",
                    i, t.x, t.y, t.z, t.stair_cmd, t.approach_x, t.approach_y,
                    t.stand_height, t.block_height, t.grab_is_finsh, t.use_rotate);
    }
}

void R2DecisionNode::buildZone2Route(Context &ctx, const std::vector<uint8_t> &lightboard_map)
{
    ctx.zone2_tasks.clear();

    if (lightboard_map.size() != 12)
    {
        RCLCPP_WARN(rclcpp::get_logger("planner"),
                    "buildZone2Route: no lightboard data (size=%zu), Zone2 skipped",
                    lightboard_map.size());
        return;
    }

    // ── parse lightboard ──
    //  0=empty  1=R1 KFS(passable)  2=R2 KFS(target)  3=fake(obstacle)
    bool passable[12];
    bool has_r2[12];
    for (int i = 0; i < 12; ++i)
    {
        has_r2[i] = (lightboard_map[i] == 2);
        passable[i] = (lightboard_map[i] != 3);
    }

    // ── count R2 KFS per column ──
    //  右列=0,3,6,9  中列=1,4,7,10  左列=2,5,8,11
    int col_r2[3] = {0, 0, 0};
    for (int i = 0; i < 12; ++i)
        if (has_r2[i])
            col_r2[i % 3]++;

    // ── choose primary column (most R2 KFS) ──
    int primary_col = 0;
    if (col_r2[1] > col_r2[primary_col]) primary_col = 1;
    if (col_r2[2] > col_r2[primary_col]) primary_col = 2;

    // ── build path using greedy + lookup table ──
    int pos = 1;  // entry at block 1
    std::vector<int> path;
    std::unordered_map<int, int> adjacent_grabs;  // block → adjacent block to grab
    path.push_back(pos);
    if (has_r2[pos]) has_r2[pos] = false;

    // ── move to primary column in row 0 ──
    int cur_col = pos % 3;
    if (cur_col != primary_col)
    {
        int step = (primary_col > cur_col) ? 1 : -1;
        for (int c = cur_col + step; ; c += step)
        {
            int block = c;
            if (!passable[block]) break;
            path.push_back(block);
            pos = block;
            if (has_r2[pos]) has_r2[pos] = false;
            if (c == primary_col) break;
        }
    }

    // ── descend row by row ──
    int target_col = primary_col;
    for (int row = 0; row < 3; ++row)
    {
        int next_row = row + 1;
        int next_block = next_row * 3 + target_col;

        // check passable + can reach via lookup table
        const BlockAction* action = findAction(pos, next_block);
        bool can_descend = passable[next_block] && (action != nullptr);

        if (!can_descend)
        {
            // try adjacent columns
            bool found = false;
            for (int dc : {-1, 1})
            {
                int alt_col = target_col + dc;
                if (alt_col < 0 || alt_col > 2) continue;
                int alt_block = next_row * 3 + alt_col;
                if (!passable[alt_block]) continue;

                const BlockAction* alt_action = findAction(pos, alt_block);
                if (!alt_action) continue;

                // move horizontally to alt_col if needed
                int cur = pos % 3;
                if (cur != alt_col)
                {
                    int s = (alt_col > cur) ? 1 : -1;
                    for (int c = cur + s; ; c += s)
                    {
                        int block = row * 3 + c;
                        if (!passable[block]) break;
                        path.push_back(block);
                        pos = block;
                        if (has_r2[pos]) has_r2[pos] = false;
                        if (c == alt_col) break;
                    }
                }
                // descend
                path.push_back(alt_block);
                pos = alt_block;
                if (has_r2[pos]) has_r2[pos] = false;
                target_col = alt_col;
                found = true;
                break;
            }
            if (!found) break;
        }
        else
        {
            // mark adjacent R2 KFS for rotation grab
            cur_col = pos % 3;
            for (int dc : {-1, 1})
            {
                int adj_col = cur_col + dc;
                if (adj_col < 0 || adj_col > 2) continue;
                int adj_block = row * 3 + adj_col;
                if (has_r2[adj_block] && passable[adj_block])
                {
                    adjacent_grabs[pos] = adj_block;
                    has_r2[adj_block] = false;
                }
            }

            // descend
            path.push_back(next_block);
            pos = next_block;
            if (has_r2[pos]) has_r2[pos] = false;
        }
    }

    // ── exit row: mark adjacent R2 KFS for rotation grab ──
    cur_col = pos % 3;
    for (int dc : {-1, 1})
    {
        int adj_col = cur_col + dc;
        if (adj_col < 0 || adj_col > 2) continue;
        int adj_block = 9 + adj_col;
        if (has_r2[adj_block] && passable[adj_block])
        {
            adjacent_grabs[pos] = adj_block;
            has_r2[adj_block] = false;
        }
    }

    // ── exit at 11 or 9 ──
    for (int exit_block : {11, 9})
    {
        if (pos == exit_block) break;
        if (!passable[exit_block]) continue;
        const BlockAction* exit_action = findAction(pos, exit_block);
        if (exit_action)
        {
            path.push_back(exit_block);
            pos = exit_block;
            if (has_r2[pos]) has_r2[pos] = false;
            break;
        }
    }

    // ── build tasks using lookup table ──
    for (size_t i = 0; i < path.size(); ++i)
    {
        int idx = path[i];
        Zone2Task t;
        t.id = idx;
        t.x = ctx.zone2_blocks[idx].x;
        t.y = ctx.zone2_blocks[idx].y;
        t.z = ctx.zone2_blocks[idx].z;
        t.grab_scene = ctx.zone2_blocks[idx].grab_scene;
        t.arm_command = 0;

        // look up action for next block
        if (i + 1 < path.size())
        {
            int next_idx = path[i + 1];
            const BlockAction* action = findAction(idx, next_idx);
            if (action)
            {
                t.stair_cmd = action->stair;
            }
        }

        // check for adjacent grab
        auto it = adjacent_grabs.find(idx);
        if (it != adjacent_grabs.end())
        {
            t.grab_adjacent_block = it->second;
            const BlockAction* grab_action = findAction(idx, it->second);
            if (grab_action)
                t.grab_is_finsh = grab_action->grab;
        }

        ctx.zone2_tasks.push_back(t);
    }

    RCLCPP_INFO(rclcpp::get_logger("planner"),
                "buildZone2Route: %zu tasks (primary_col=%d):", ctx.zone2_tasks.size(), primary_col);
    for (size_t i = 0; i < ctx.zone2_tasks.size(); ++i)
    {
        const auto &t = ctx.zone2_tasks[i];
        const char *pos_str = isEntryBlock(t.id) ? "[ENTRY]" : isExitBlock(t.id) ? "[EXIT]" : "";
        if (t.grab_adjacent_block >= 0)
            RCLCPP_INFO(rclcpp::get_logger("planner"),
                        "  #%zu block %d %s (%.2f,%.2f) stair=%d grab_adj=%d grab=%d",
                        i, t.id, pos_str, t.x, t.y, t.stair_cmd,
                        t.grab_adjacent_block, t.grab_is_finsh);
        else
            RCLCPP_INFO(rclcpp::get_logger("planner"),
                        "  #%zu block %d %s (%.2f,%.2f) stair=%d",
                        i, t.id, pos_str, t.x, t.y, t.stair_cmd);
    }
}
