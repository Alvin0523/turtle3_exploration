#!/usr/bin/env python3
"""
Frontier explorer.

Waypoint rules:
  1. Free cell (0) adjacent to unknown (-1) on SLAM map
  2. Costmap value = 0 at centroid — no inflation, safely navigable
  3. Opening width >= MIN_OPENING (fits robot)

Navigation:
  - Pick nearest valid frontier by navigable path distance
  - Keep going until that frontier disappears from the map
  - When it clears, prefer a frontier within ±45° of current heading
    to continue in the same direction before falling back to global nearest

Colours in RViz (/explore/frontiers):
  GREEN  = valid frontier waypoint
  YELLOW = currently navigating
"""

import math
import numpy as np
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros
from tf2_ros import TransformException
from scipy.ndimage import label, binary_dilation

FREE    =  0
UNKNOWN = -1
MIN_OPENING = 0.50   # m — min passage width for robot to pass


class Explorer(Node):

    def __init__(self):
        super().__init__('dfs_explorer')

        self.declare_parameter('min_frontier_cells', 3)
        self.declare_parameter('planner_frequency',  0.5)
        self.declare_parameter('heading_cone_deg',   45)

        self.map_info      = None
        self.map_grid      = None
        self.costmap_info  = None
        self.costmap_grid  = None

        self.navigating        = False
        self.frontier_ids      = {}     # (fx,fy) → persistent int label
        self._next_fid         = 1      # never reused
        self.current_frontier  = None   # (fx, fy) yellow

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

        self.create_subscription(OccupancyGrid,       '/map',                      self._map_cb,      1)
        self.create_subscription(OccupancyGridUpdate, '/map_updates',              self._update_cb,   10)
        self.create_subscription(OccupancyGrid,       '/global_costmap/costmap',   self._costmap_cb,  1)

        self.frontier_pub = self.create_publisher(MarkerArray, '/explore/frontiers', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        hz = self.get_parameter('planner_frequency').value
        self.create_timer(1.0 / hz, self._plan)
        self.get_logger().info('Explorer ready.')

    # ── map callbacks ─────────────────────────────────────────────────────────

    def _map_cb(self, msg):
        self.map_info = msg.info
        self.map_grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def _update_cb(self, msg):
        if self.map_grid is None:
            return
        patch = np.array(msg.data, dtype=np.int8).reshape(msg.height, msg.width)
        self.map_grid[msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = patch

    def _costmap_cb(self, msg):
        self.costmap_info = msg.info
        self.costmap_grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    # ── plan ──────────────────────────────────────────────────────────────────

    def _plan(self):
        if self.map_grid is None or self.costmap_grid is None:
            return

        pose = self._robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        frontiers = self._find_frontiers()

        cone_deg = float(self.get_parameter('heading_cone_deg').value)

        # Commitment (heading-based): while navigating, only switch targets if we
        # can keep moving forward (within ±cone_deg of current heading).
        if self.navigating:
            cfx, cfy = self.current_frontier if self.current_frontier else (None, None)
            current_still_exists = False
            if cfx is not None:
                current_still_exists = any(
                    math.hypot(cfx - fx, cfy - fy) < 0.35 for fx, fy, _ in frontiers
                )

            # If the committed frontier still exists AND is still roughly ahead,
            # keep going (don’t churn goals mid-hallway).
            if current_still_exists:
                diff = abs(
                    math.atan2(
                        math.sin(math.atan2(cfy - ry, cfx - rx) - ryaw),
                        math.cos(math.atan2(cfy - ry, cfx - rx) - ryaw),
                    )
                )
                if diff <= math.radians(cone_deg):
                    self._publish(frontiers)
                    return

            # Otherwise, try to preempt to the closest frontier that is ahead.
            ahead = self._frontiers_in_heading_cone(rx, ry, ryaw, frontiers, deg=cone_deg)
            target = self._closest_frontier(rx, ry, ahead, exclude=self.current_frontier) if ahead else None
            if target is None:
                # No forward option — keep current Nav2 goal running.
                self._publish(frontiers)
                return

            fx, fy, _ = target
            gx, gy = self._nearest_costmap_free(fx, fy)

            self.get_logger().info(
                f'[plan] (preempt) frontier=({fx:.2f},{fy:.2f}) goal=({gx:.2f},{gy:.2f})')
            self.current_frontier = (fx, fy)
            self._publish(frontiers)
            self._send_goal(gx, gy, math.atan2(fy - ry, fx - rx))
            return

        if not frontiers:
            self.get_logger().info('[plan] no frontiers — done!')
            self._publish([])
            return

        # Always prefer heading direction first (cone check covers both:
        #   nav2 SUCCEEDED normally, and frontier cleared while navigating)
        ahead = self._frontiers_in_heading_cone(rx, ry, ryaw, frontiers, deg=cone_deg)
        target = self._closest_frontier(rx, ry, ahead, exclude=None) if ahead else None
        if target:
            self.get_logger().info('[plan] continuing in heading direction')
        else:
            target = self._closest_frontier(rx, ry, frontiers, exclude=None)

        if target is None:
            self.get_logger().warn('[plan] no reachable frontier')
            self._publish(frontiers)
            return

        fx, fy, _ = target

        # Find nearest costmap=0 cell to the frontier centroid
        # so Nav2 gets a clean navigable goal, not an inflated boundary cell
        gx, gy = self._nearest_costmap_free(fx, fy)

        self.get_logger().info(f'[plan] frontier=({fx:.2f},{fy:.2f}) goal=({gx:.2f},{gy:.2f})')
        self.current_frontier = (fx, fy)
        self.navigating = True
        self._publish(frontiers)
        self._send_goal(gx, gy, math.atan2(fy - ry, fx - rx))

    # ── frontier detection ────────────────────────────────────────────────────

    def _find_frontiers(self):
        """Return [(wx, wy, size)] — valid frontier centroids.

        Valid = free cell touching unknown, opening >= MIN_OPENING,
                centroid not inside costmap inflation zone.
        """
        grid = self.map_grid
        res  = self.map_info.resolution
        ox   = self.map_info.origin.position.x
        oy   = self.map_info.origin.position.y

        frontier_mask = (grid == FREE) & binary_dilation(grid == UNKNOWN, iterations=1)
        frontier_mask = binary_dilation(frontier_mask, iterations=1)  # merge diagonals
        labeled, n    = label(frontier_mask)
        min_cells     = self.get_parameter('min_frontier_cells').value

        out = []
        for i in range(1, n + 1):
            cells = np.argwhere(labeled == i)
            if len(cells) < min_cells:
                continue

            rows, cols = cells[:, 0], cells[:, 1]
            opening = max(rows.max() - rows.min() + 1, cols.max() - cols.min() + 1) * res
            if opening < MIN_OPENING:
                continue

            wx = ox + cols.mean() * res
            wy = oy + rows.mean() * res

            cval = self._costmap_value(wx, wy)
            if cval is not None and cval > 0:
                self.get_logger().info(f'[frontier] ({wx:.2f},{wy:.2f}) SKIP inflated cval={cval}')
                continue

            self.get_logger().info(
                f'[frontier] ({wx:.2f},{wy:.2f}) OK size={len(cells)} opening={opening:.2f}m')
            out.append((wx, wy, len(cells)))
        return out

    def _costmap_value(self, wx, wy):
        """Return costmap value at world position, or None if unavailable."""
        if self.costmap_grid is None or self.costmap_info is None:
            return None
        res = self.costmap_info.resolution
        row = int((wy - self.costmap_info.origin.position.y) / res)
        col = int((wx - self.costmap_info.origin.position.x) / res)
        if 0 <= row < self.costmap_grid.shape[0] and 0 <= col < self.costmap_grid.shape[1]:
            return int(self.costmap_grid[row, col])
        return None

    def _nearest_costmap_free(self, wx, wy):
        """BFS outward from (wx,wy) to find nearest costmap=0 cell for Nav2 goal."""
        if self.costmap_grid is None or self.costmap_info is None:
            return wx, wy
        res = self.costmap_info.resolution
        ox  = self.costmap_info.origin.position.x
        oy  = self.costmap_info.origin.position.y
        sr  = int((wy - oy) / res)
        sc  = int((wx - ox) / res)

        visited, queue = set(), deque([(sr, sc)])
        while queue:
            r, c = queue.popleft()
            if (r, c) in visited or len(visited) > 200:
                break
            visited.add((r, c))
            if not (0 <= r < self.costmap_grid.shape[0] and 0 <= c < self.costmap_grid.shape[1]):
                continue
            if int(self.costmap_grid[r, c]) == 0:
                return ox + c * res, oy + r * res
            queue.extend((r+dr, c+dc) for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]
                         if (r+dr, c+dc) not in visited)
        return wx, wy

    # ── heading cone check ───────────────────────────────────────────────────

    def _frontiers_in_heading_cone(self, rx, ry, ryaw, frontiers, deg=45):
        """Return frontiers within ±deg° of the robot's current heading.
        Robot is already facing the direction it traveled — live heading is
        more accurate than a stored commit heading."""
        cone = math.radians(deg)
        result = []
        for item in frontiers:
            fx, fy = item[0], item[1]
            angle_to = math.atan2(fy - ry, fx - rx)
            diff = abs(math.atan2(math.sin(angle_to - ryaw),
                                  math.cos(angle_to - ryaw)))
            if diff <= cone:
                result.append(item)
        return result

    # ── selection ─────────────────────────────────────────────────────────────

    def _closest_frontier(self, rx, ry, frontiers, exclude):
        """Pick nearest frontier by BFS navigable distance."""
        distances = self._path_distances(rx, ry, frontiers)
        best, best_d = None, math.inf
        for fx, fy, size in frontiers:
            if exclude and math.hypot(fx - exclude[0], fy - exclude[1]) < 0.35:
                continue
            d = distances.get((fx, fy), math.inf)
            if d < best_d:
                best_d, best = d, (fx, fy, size)
        return best

    # ── BFS navigable distance ────────────────────────────────────────────────

    def _path_distances(self, rx, ry, frontiers):
        # Use COSTMAP for reachability — same grid Nav2 uses for planning.
        # Frontiers unreachable via costmap get distance=inf → never selected → no abort.
        if self.costmap_grid is not None and self.costmap_info is not None:
            grid = self.costmap_grid
            res  = self.costmap_info.resolution
            ox   = self.costmap_info.origin.position.x
            oy   = self.costmap_info.origin.position.y
            def passable(r, c):
                return 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1] \
                       and int(grid[r, c]) < 253   # not lethal
        else:
            grid = self.map_grid
            res  = self.map_info.resolution
            ox   = self.map_info.origin.position.x
            oy   = self.map_info.origin.position.y
            def passable(r, c):
                return 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1] \
                       and grid[r, c] <= 50

        rc = int((rx - ox) / res)
        rr = int((ry - oy) / res)
        if not (0 <= rr < grid.shape[0] and 0 <= rc < grid.shape[1]):
            return {}

        targets = {}
        for fx, fy, _ in frontiers:
            fc = int((fx - ox) / res)
            fr = int((fy - oy) / res)
            if 0 <= fr < grid.shape[0] and 0 <= fc < grid.shape[1]:
                targets[(fr, fc)] = (fx, fy)

        visited = set()
        queue   = deque([(rr, rc, 0)])
        result  = {}

        while queue:
            r, c, d = queue.popleft()
            if (r, c) in visited:
                continue
            visited.add((r, c))

            for (tr, tc), (fx, fy) in list(targets.items()):
                if abs(r - tr) <= 1 and abs(c - tc) <= 1:
                    if (fx, fy) not in result:
                        result[(fx, fy)] = d * res
                    del targets[(tr, tc)]

            if not targets:
                break

            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nr, nc = r + dr, c + dc
                if (nr, nc) not in visited and passable(nr, nc):
                    queue.append((nr, nc, d + 1))

        return result

    # ── Nav2 ──────────────────────────────────────────────────────────────────

    def _send_goal(self, x, y, heading):
        if not self.nav.wait_for_server(timeout_sec=1.0):
            self.navigating = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id    = 'map'
        goal_msg.pose.header.stamp       = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x    = x
        goal_msg.pose.pose.position.y    = y
        goal_msg.pose.pose.orientation.z = math.sin(heading / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(heading / 2.0)

        def on_accepted(fut):
            h = fut.result()
            if not h.accepted:
                self.get_logger().warn('[nav2] REJECTED')
                self.navigating = False
                return
            self.get_logger().info(f'[nav2] ACCEPTED ({x:.2f},{y:.2f})')
            def on_done(rfut):
                res    = rfut.result()
                status = res.status if res else 0
                name   = {4:'SUCCEEDED', 5:'CANCELLED', 6:'ABORTED'}.get(status, str(status))
                self.get_logger().info(f'[nav2] {name}')
                self.navigating = False
                if status == 6:
                    self.current_frontier = None
            h.get_result_async().add_done_callback(on_done)

        self.nav.send_goal_async(goal_msg).add_done_callback(on_accepted)

    # ── visualization ─────────────────────────────────────────────────────────

    def _publish(self, frontiers):
        arr   = MarkerArray()
        del_m = Marker(); del_m.action = Marker.DELETEALL
        arr.markers.append(del_m)

        # Assign persistent IDs — match existing frontiers by proximity, assign new if unseen
        matched = {}
        for fx, fy, size in frontiers:
            best_key = None
            best_d   = 0.4   # match radius
            for key in self.frontier_ids:
                d = math.hypot(fx - key[0], fy - key[1])
                if d < best_d:
                    best_d, best_key = d, key
            if best_key:
                matched[(fx, fy)] = self.frontier_ids[best_key]
            else:
                matched[(fx, fy)] = self._next_fid
                self._next_fid += 1
        # Prune IDs no longer visible
        self.frontier_ids = {(fx, fy): matched[(fx, fy)] for fx, fy, _ in frontiers}

        for i, (fx, fy, size) in enumerate(frontiers):
            # Yellow = currently navigating, everything else green
            is_yellow = (self.current_frontier and
                         math.hypot(fx - self.current_frontier[0],
                                    fy - self.current_frontier[1]) < 0.35)
            color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0) if is_yellow \
                else ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

            # Fixed-size sphere
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns = 'f'; m.id = i; m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = fx; m.pose.position.y = fy; m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.20   # fixed size
            m.color = color
            arr.markers.append(m)

        self.frontier_pub.publish(arr)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _robot_pose(self):
        try:
            t   = self.tf_buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x   = t.transform.translation.x
            y   = t.transform.translation.y
            q   = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
            return x, y, yaw
        except TransformException:
            return None


def main():
    rclpy.init()
    rclpy.spin(Explorer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
