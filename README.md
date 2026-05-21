# TurtleBot3 Waffle — Autonomous Exploration Workspace

Competition task: autonomous frontier exploration of an unknown indoor environment
using TurtleBot3 Waffle + LiDAR + SLAM + Nav2 + custom explore. Shortest time wins.

---

## Setup

```bash
# Clone with all submodules (ld08_driver, coin_d4_driver)
git clone --recursive <repo-url> turtlebot_ws
cd turtlebot_ws

# If you already cloned without --recursive:
git submodule update --init --recursive

# Install pixi environment (PC)
pixi install

# Clone and build explore_lite (PC only)
pixi run setup
pixi run build
```

Submodules:
- `src/ld08_driver` — branch `jazzy` (Pi LiDAR driver)
- `src/coin_d4_driver` — branch `jazzy` (Pi LiDAR driver)

---

## Quick Start

```bash
Terminal 1: pixi run router        # Zenoh middleware — start first
Terminal 2: pixi run sim-comp      # Gazebo with comp.world
Terminal 3: pixi run slam          # SLAM Toolbox — wait for "Registering sensor"
Terminal 4: pixi run nav2          # Nav2 stack — wait for "Managed nodes are active"
Terminal 5: pixi run explore       # custom autonomous frontier exploration
Terminal 6: pixi run rviz          # visualisation (optional)
```

---

## comp.world Arena Layout

```
Scale: 1 cell ≈ 0.25m  | = vertical wall  _ = horizontal wall  + = junction

         ←─1.08m─→←──────2.59m──────→←0.98m→

 +____+______________+                      ← y=+1.53m  ↑
 |    |              |                              |
 |    |              |                            0.97m
 |    |              |                              ↓
 |    +______   |    |                      ← y=+0.41m  (Wall_26, 1.75m)
 |    |         |    |                              ↑
 |    |         |    |                            1.70m
 |    |    |    |    |                              |
 |         |    |    |  ← doorway (Wall_16 ends)    |
 |         |    |    |    @ y=−0.51m                ↓
 |         |    |    |
 +_________+    |    |                      ← y=−1.43m  (Wall_22, 2.75m)
 |              |    |                              ↑
 |              |    |                            0.99m
 |              |    |                              ↓
 +______________+____+                      ← y=−2.57m

 ↑    ↑              ↑    ↑
x=  x=             x=   x=
−2.55 −1.32       +1.42 +2.55
```

Key dimensions:
- Overall outer: 5.25m wide × 4.10m tall
- All walls: 0.15m thick × 2.50m high
- Wall_8: vertical, starts at y=+0.37, runs to bottom at x=+1.42
- Wall_28: short vertical stub y=−0.51m → y=−1.43m (inside middle section)
- Wall_16: vertical from top down to y=−0.51m (doorway at bottom)

Key passages:
- Wall_16 bottom doorway: ~1.08m wide, at y=−0.51m (left corridor → middle)
- Wall_26 → Wall_8 gap: ~0.99m wide (middle → right corridor D)
- Wall_22 right end → Wall_8 gap: ~1.2m wide (upper → lower room E)

Robot spawns at (0.0, 0.5) — inside the middle section, upper area.

Save map after exploration:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

---

## Measured Baselines (recorded 2026-05-16)

### Topic Rates (from `ros2 topic hz`)

| Topic | Measured Rate | Expected | Notes |
|-------|--------------|----------|-------|
| `/scan` | ~8.6 Hz | 5–10 Hz | LiDAR laser scan from Gazebo bridge |
| `/odom` | ~44 Hz | 30–50 Hz | Wheel odometry from Gazebo |
| `/clock` | ~700–880 Hz | >100 Hz | Sim clock from Gazebo — healthy |
| `/map` | ~1 Hz | ~1 Hz | SLAM Toolbox occupancy grid |
| `/cmd_vel` | on demand | — | TwistStamped (robot expects stamped) |

### TF Tree (from `ros2 run tf2_tools view_frames`)

```
map  (published by: slam_toolbox @ 10.24 Hz)
└── odom  (@ 50.24 Hz)
    └── base_footprint  (@ 50.24 Hz)
        └── base_link  (static)
            ├── base_scan       (static) ← LiDAR frame
            ├── imu_link        (static)
            ├── wheel_left_link  (@ 20.24 Hz)
            ├── wheel_right_link (@ 20.24 Hz)
            ├── caster_back_left_link  (static)
            ├── caster_back_right_link (static)
            └── camera_link     (static)
                ├── camera_depth_frame
                │   └── camera_depth_optical_frame
                └── camera_rgb_frame
                    └── camera_rgb_optical_frame
```

All required frames present. Critical chain: `map → odom → base_footprint → base_link → base_scan`

---

## Nav2 Tuning

### The tuning sequence (always in this order)

```
1. Sensors         verify topics publishing at correct rates
2. TF tree         verify full chain map→odom→base_link→base_scan exists
3. Costmap         tune inflation_radius — most impactful param
4. Planner         tune tolerance, allow_unknown
5. Controller      tune speed, progress checker
6. explore_lite    tune frontier selection after Nav2 is solid
```

Never skip ahead. A wrong costmap makes the controller look broken.
Fix upstream stages first.

### Costmap — what to look for in RViz

Add `/global_costmap/costmap` and `/local_costmap/costmap` to RViz:

- **Black cells** = lethal obstacle (wall)
- **Blue/purple gradient** = inflation zone — robot treats this as dangerous
- **White cells** = free space — robot can path here
- **Grey cells** = unknown space

If a corridor appears all blue/purple with no white, the robot cannot plan through
it regardless of controller tuning. Fix the inflation radius first.

### Key params and where they live

All params are in `config/nav2_params.yaml` (your local editable copy).
`pixi run nav2` and `pixi run nav2-real` both point here.

#### Inflation (most critical)

```yaml
# Applies to BOTH local_costmap and global_costmap sections
inflation_layer:
  inflation_radius: 0.25     # metres — bubble around every obstacle
  cost_scaling_factor: 5.0   # dropoff rate — higher = sharper = robot can get closer
```

Rule: `inflation_radius = robot_actual_radius + safety_margin`
TB3 Waffle actual radius = 0.14 m
- Conservative: 0.35–0.40 m (avoids walls, misses narrow corridors)
- Balanced:     0.25–0.30 m (recommended start)
- Aggressive:   0.15–0.20 m (navigates tight spaces, higher collision risk)

#### Progress checker

```yaml
controller_server:
  progress_checker:
    required_movement_radius: 0.1    # must move this many metres...
    movement_time_allowance: 20.0    # ...within this many seconds
```

If the robot is physically navigating a tight corner, 10 s can be too short.
Increase to 20–30 s before blaming the controller.

#### Controller speed (DWB)

```yaml
FollowPath:
  max_vel_x: 0.4         # m/s forward speed (TB3 max is ~0.5)
  max_speed_xy: 0.4      # must match max_vel_x
  max_vel_theta: 1.5     # rad/s rotation speed
  sim_time: 1.5          # seconds DWB simulates ahead — increase for faster speeds
```

#### Planner

```yaml
planner_server:
  GridBased:
    tolerance: 0.5        # how close to goal is "close enough" in metres
    use_astar: false      # false = Dijkstra (better near obstacles), true = A* (faster open space)
    allow_unknown: true   # MUST be true for exploration
```

`allow_unknown: true` is mandatory for exploration — without it the planner
refuses to path through unmapped cells and the robot boxes itself in.

#### explore_lite frontier params

```yaml
# src/m-explore-ros2/explore/config/params.yaml
planner_frequency: 0.15     # Hz — how often to search for next frontier
progress_timeout: 60.0      # seconds before blacklisting an unreachable frontier
potential_scale: 3.0        # higher = prefer closer frontiers
gain_scale: 1.0             # higher = prefer larger frontiers
min_frontier_size: 0.5      # metres — ignore frontiers smaller than this
return_to_init: true        # return to start when exploration complete
```

Scoring formula: `cost = potential_scale × distance − gain_scale × size`
Lower cost = frontier chosen first.

---

## Tuning Log

Changes from original waffle.yaml — recorded here so we know what was changed, why, and what the result was.

| # | Date | File | Param | Original | New | Reason | Result |
|---|------|------|-------|----------|-----|--------|--------|
| 1 | 2026-05-16 | config/nav2_params.yaml | `local_costmap.inflation_layer.inflation_radius` | 0.5 | 0.25 | 0.5m inflation on both sides of a 1m corridor = 1.0m consumed, zero free cells, planner blocked. Formula: robot_radius(0.15) + safety(0.10) = 0.25 | TBD |
| 2 | 2026-05-16 | config/nav2_params.yaml | `global_costmap.inflation_layer.inflation_radius` | 0.5 | 0.25 | Same as above — global costmap used by planner for path finding | TBD |
| 3 | 2026-05-16 | config/nav2_params.yaml | `controller_server.progress_checker.movement_time_allowance` | 10.0 s | 20.0 s | 10s too short for tight corridor navigation. Robot was physically navigating but progress checker declared "stuck" and aborted, causing explore_lite to blacklist reachable frontiers | TBD |
| 4 | 2026-05-16 | config/nav2_params.yaml | `controller_server.FollowPath.plugin` | `dwb_core::DWBLocalPlanner` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | DWB GoalAlign critic rotated robot toward final goal at L-corners instead of following the planned path — robot faced wall and stalled. RPP follows path curvature and rotates in-place at sharp corners via `use_rotate_to_heading: true` | TBD |

> Fill in the **Result** column after testing each change. If it didn't help, revert and document that too.

---


---

## Autonomous Exploration Logic (explorer.py)

The `scripts/explorer.py` node implements autonomous frontier-based exploration for TurtleBot3 using ROS 2. Key logic:

- **Frontier Detection:**
  - Identifies free map cells adjacent to unknown space as frontiers.
  - Only considers frontiers wide enough for the robot and with a safe, non-inflated costmap value at the centroid.

- **Navigation Strategy:**
  - Selects the nearest valid frontier by path distance (using the costmap for reachability).
  - While navigating, commits to the current frontier unless it disappears or is no longer ahead.
  - If the current frontier clears, prefers a new frontier within ±45° of the robot's heading; otherwise, picks the global nearest.
  - Navigation goals are set to the nearest costmap-free cell near the frontier centroid.

- **Visualization:**
  - Publishes frontiers as markers in RViz on `/explore/frontiers`.
    - Green: Valid frontier waypoints
    - Yellow: Currently navigating target

- **ROS 2 Integration:**
  - Subscribes to map, map updates, and costmap topics.
  - Uses Nav2's `NavigateToPose` action for movement.
  - Uses TF2 to get the robot's current pose.

- **Parameters:**
  - `min_frontier_cells`: Minimum number of cells for a valid frontier.
  - `planner_frequency`: How often to replan (Hz).
  - `heading_cone_deg`: Angle for heading commitment (default 45°).

See `scripts/explorer.py` for implementation details.
