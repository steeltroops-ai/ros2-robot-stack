#!/bin/bash
set -e

# 1. Source ROS 2 environments
# We use -f to check if the file exists to prevent script crashes in edge cases
[ -f "/opt/ros/humble/setup.bash" ] && source /opt/ros/humble/setup.bash
[ -f "/app/robotics/ros2_ws/install/setup.bash" ] && source /app/robotics/ros2_ws/install/setup.bash

echo "--- [SYSTEM] ROS 2 Humble Environment Sourced ---"

# 2. Start the Mock Robot Simulation
echo "--- [DEPLOY] Starting Mock Robot Simulation (robot_1) ---"
ros2 run simulation_manager mock_robot --ros-args -p robot_id:=robot_1 &

# 3. Start the Nav2 Stack (Minimal)
echo "--- [DEPLOY] Starting Nav2 Stack (Navigation Brain) ---"
ros2 launch amr_navigation amr_nav.launch.py use_sim_time:=False &

# 4. Start the Fastify Gateway using BUN (Main Process)
# We use 'bun run dev' which is much faster than node/npm
echo "--- [DEPLOY] Starting Bun-Native Telemetry Gateway (Port: 7860) ---"
cd /app/apps/backend
# Ensure we are using the Production entrypoint
exec bun run dev
