#include <algorithm>

#include "r2_decision/r2_decision_node.hpp"

namespace
{
constexpr int kAdj[12][4] = {
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

bool isEntryBlock(int idx) { return idx == 0 || idx == 1 || idx == 2; }
bool isExitBlock(int idx) { return idx == 9 || idx == 10 || idx == 11; }

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

    // ── height map (fixed per block) ──
    //  2  1  2     row 0
    //  3  2  1     row 1
    //  2  3  2     row 2
    //  1  2  1     row 3
    constexpr int kHeight[12] = {2, 1, 2, 3, 2, 1, 2, 3, 2, 1, 2, 1};

    // ── parse lightboard ──
    //  0=empty  1=R1 KFS(passable)  2=R2 KFS(target)  3=fake(obstacle)
    bool passable[12];
    bool has_r2[12];
    for (int i = 0; i < 12; ++i)
    {
        has_r2[i] = (lightboard_map[i] == 2);
        passable[i] = (lightboard_map[i] != 3);  // only fake is obstacle
    }

    // ── count R2 KFS per column ──
    int col_r2[3] = {0, 0, 0};
    for (int i = 0; i < 12; ++i)
        if (has_r2[i])
            col_r2[i % 3]++;

    // ── choose primary column (most R2 KFS) ──
    int primary_col = 0;
    if (col_r2[1] > col_r2[primary_col]) primary_col = 1;
    if (col_r2[2] > col_r2[primary_col]) primary_col = 2;

    // ── entry at block 1 ──
    int pos = 1;
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
            int block = c;  // row 0
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

        // check passable + height constraint: descent ≤ 1
        bool can_descend = passable[next_block] && (kHeight[pos] - kHeight[next_block] <= 1);
        if (!can_descend)
        {
            // can't descend here, try adjacent columns
            bool found = false;
            for (int dc : {-1, 1})
            {
                int alt_col = target_col + dc;
                if (alt_col < 0 || alt_col > 2) continue;
                int alt_block = next_row * 3 + alt_col;
                if (!passable[alt_block]) continue;
                if (kHeight[pos] - kHeight[alt_block] > 1) continue;

                // move horizontally to alt_col
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
            // mark adjacent R2 KFS for rotation grab (no physical move)
            cur_col = pos % 3;
            for (int dc : {-1, 1})
            {
                int adj_col = cur_col + dc;
                if (adj_col < 0 || adj_col > 2) continue;
                int adj_block = row * 3 + adj_col;
                if (has_r2[adj_block] && passable[adj_block])
                {
                    // record: grab adjacent block's KFS by rotating at current pos
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
        // check adjacency
        bool adj = false;
        for (int ni = 0; ni < 4; ++ni)
            if (kAdj[pos][ni] == exit_block)
                adj = true;
        if (adj)
        {
            path.push_back(exit_block);
            pos = exit_block;
            if (has_r2[pos]) has_r2[pos] = false;
            break;
        }
    }

    // ── build tasks ──
    for (int idx : path)
    {
        Zone2Task t;
        t.id = idx;
        t.x = ctx.zone2_blocks[idx].x;
        t.y = ctx.zone2_blocks[idx].y;
        t.z = ctx.zone2_blocks[idx].z;
        t.grab_scene = ctx.zone2_blocks[idx].grab_scene;
        t.arm_command = 0;
        auto it = adjacent_grabs.find(idx);
        if (it != adjacent_grabs.end())
            t.grab_adjacent_block = it->second;
        ctx.zone2_tasks.push_back(t);
    }

    RCLCPP_INFO(rclcpp::get_logger("planner"),
                "buildZone2Route: %zu tasks (primary_col=%d):", ctx.zone2_tasks.size(), primary_col);
    for (size_t i = 0; i < ctx.zone2_tasks.size(); ++i)
    {
        const auto &t = ctx.zone2_tasks[i];
        const char *pos_str = isEntryBlock(t.id) ? "[ENTRY]" : isExitBlock(t.id) ? "[EXIT]" : "";
        if (t.grab_adjacent_block >= 0)
            RCLCPP_INFO(rclcpp::get_logger("planner"), "  #%zu block %d %s (%.2f,%.2f) grab_adj=%d",
                        i, t.id, pos_str, t.x, t.y, t.grab_adjacent_block);
        else
            RCLCPP_INFO(rclcpp::get_logger("planner"), "  #%zu block %d %s (%.2f,%.2f)",
                        i, t.id, pos_str, t.x, t.y);
    }
}
