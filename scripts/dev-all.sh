#!/bin/bash

# Configuration
WSL_PROJECT_PATH="/root/projects/ros2-robot-stack"
BUN_PATH="/root/.bun/bin/bun"
ROS_SETUP="/opt/ros/humble/setup.bash"

# Function to kill child processes on exit
trap "kill 0" EXIT

echo "Launching Robotics Stack from WSL Performance Layer..."

# 1. Start ROS 2 Mock Robot
echo "[1/3] Starting ROS 2 Simulation Node..."
(
    source $ROS_SETUP
    cd $WSL_PROJECT_PATH/robotics/ros2_ws
    source install/setup.bash 2>/dev/null || echo "Warning: install/setup.bash not found, running with system ROS only."
    python3 src/simulation_manager/src/mock_robot.py
) &

# 2. Start Backend (Requires ROS Environment for rclnodejs)
echo "[2/3] Starting Backend Bridge..."
(
    source $ROS_SETUP
    cd $WSL_PROJECT_PATH/apps/backend
    $BUN_PATH run dev
) &

# 3. Start Frontend
echo "[3/3] Starting Frontend Dashboard..."
(
    cd $WSL_PROJECT_PATH/apps/frontend
    $BUN_PATH run dev
) &

echo "✅ All systems initialized."
echo "Dashboard: http://localhost:3000"
echo "Backend: http://localhost:4000"
echo "Press CTRL+C to stop all services."

wait
