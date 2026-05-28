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

    std::vector<int> r2_targets;
    bool passable[12] = {false};

    for (int i = 0; i < 12; ++i)
    {
        if (lightboard_map[i] == 2)
            r2_targets.push_back(i);
        passable[i] = (lightboard_map[i] == 0 || lightboard_map[i] == 2);
    }

    if (r2_targets.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("planner"),
                    "buildZone2Route: no R2 blocks on lightboard");
        return;
    }

    int dist[12];
    std::fill_n(dist, 12, -1);
    {
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
                int nb = kAdj[cur][ni];
                if (nb < 0 || dist[nb] >= 0 || !passable[nb])
                    continue;
                dist[nb] = dist[cur] + 1;
                queue[tail++] = nb;
            }
        }
    }

    std::sort(r2_targets.begin(), r2_targets.end(),
              [&](int a, int b)
              {
                  bool ae = isEntryBlock(a), be = isEntryBlock(b);
                  if (ae != be) return ae > be;
                  if (dist[a] != dist[b]) return dist[a] < dist[b];
                  return a < b;
              });

    int current_pos = -1;
    std::vector<int> plan;

    for (int target : r2_targets)
    {
        if (dist[target] < 0) continue;

        int step = findFirstStep(current_pos, target, passable);
        if (step >= 0 && step != target && !isAlreadyPlanned(plan, step))
            plan.push_back(step);
        plan.push_back(target);
        passable[target] = true;
        current_pos = target;
    }

    for (int idx : plan)
    {
        Zone2Task t;
        t.id = idx;
        t.x = ctx.zone2_blocks[idx].x;
        t.y = ctx.zone2_blocks[idx].y;
        t.z = ctx.zone2_blocks[idx].z;
        t.grab_scene = ctx.zone2_blocks[idx].grab_scene;
        t.arm_command = 0; // sceneToArmCmd placeholder
        ctx.zone2_tasks.push_back(t);
    }

    RCLCPP_INFO(rclcpp::get_logger("planner"),
                "buildZone2Route: %zu tasks:", ctx.zone2_tasks.size());
    int task_idx = 0;
    for (const auto &t : ctx.zone2_tasks)
    {
        const char *pos = isEntryBlock(t.id) ? "[ENTRY]" : isExitBlock(t.id) ? "[EXIT]" : "";
        RCLCPP_INFO(rclcpp::get_logger("planner"), "  #%d block %d %s (%.2f,%.2f) scene=%d cmd=%d",
                    task_idx++, t.id, pos, t.x, t.y, t.grab_scene, t.arm_command);
    }
}
