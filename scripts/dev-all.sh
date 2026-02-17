#!/bin/bash

# Configuration
SESSION_NAME="robot_stack"
WSL_PROJECT_PATH="/mnt/c/Omniverse/Projects/ros2-robot-stack" # Or ~/projects/ros2-robot-stack if you moved it

# Function to kill child processes on exit
trap "kill 0" EXIT

echo "Launching Robotics Stack..."

# 1. Start ROS 2 Mock Robot
echo "[1/3] Starting ROS 2 Simulation..."
(
    source /opt/ros/humble/setup.bash
    cd $WSL_PROJECT_PATH/robotics/ros2_ws
    # Build first to be safe
    colcon build --packages-select simulation_manager
    source install/setup.bash
    # Run Node
    python3 src/simulation_manager/src/mock_robot.py
) &

# 2. Start Backend
echo "[2/3] Starting Backend..."
(
    cd $WSL_PROJECT_PATH/apps/backend
    bun run dev
) &

# 3. Start Frontend
echo "[3/3] Starting Frontend..."
(
    cd $WSL_PROJECT_PATH/apps/frontend
    bun run dev
) &

# Wait for user input to exit
echo "✅ All systems go! Open http://localhost:3000"
echo "Press CTRL+C to stop everything."
wait
