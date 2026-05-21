#!/bin/bash
# Kill all ROS2, Gazebo, SLAM, Nav2, Zenoh processes

echo "Killing everything..."

pkill -9 -f "gz sim|gzserver|gzclient|ign gazebo|ruby.*gazebo" 2>/dev/null
pkill -9 -f "async_slam_toolbox|slam_toolbox" 2>/dev/null
pkill -9 -f "controller_server|planner_server|bt_navigator|behavior_server" 2>/dev/null
pkill -9 -f "lifecycle_manager|smoother_server|route_server|opennav_docking" 2>/dev/null
pkill -9 -f "velocity_smoother|collision_monitor|waypoint_follower" 2>/dev/null
pkill -9 -f "rmw_zenohd|zenoh" 2>/dev/null
pkill -9 -f "dfs_explorer|explore_node|waypoint_explorer" 2>/dev/null
pkill -9 -f "rviz2|rqt" 2>/dev/null

sleep 2

# Stop ROS2 daemon
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null

echo "Done. All ROS2/Gazebo/Zenoh processes killed."
