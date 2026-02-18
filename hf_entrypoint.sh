#!/bin/bash
set -e

# 1. Source ROS 2 environments
source /opt/ros/humble/setup.bash
source /app/robotics/ros2_ws/install/setup.bash

# 2. Start the Mock Robot Simulation in the background
echo "--- [DEPLOY] Starting Mock Robot Simulation ---"
ros2 run simulation_manager mock_robot --ros-args -p robot_id:=robot_1 &

# 3. Start the Nav2 Stack (Minimal) in the background
# We run Nav2 in a lighter mode for the free tier
echo "--- [DEPLOY] Starting Nav2 Stack ---"
ros2 launch amr_navigation amr_nav.launch.py use_sim_time:=False &

# 4. Start the Fastify Gateway (Main Process)
# This process stays in the foreground to keep the container alive
echo "--- [DEPLOY] Starting Telemetry Gateway on Port 7860 ---"
cd /app/apps/backend
npm run dev
