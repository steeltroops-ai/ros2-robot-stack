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
    # Source local workspace (CRITICAL for Nav2 and custom msgs)
    source install/setup.bash || { echo "❌ Failed to source install/setup.bash. Did you run 'colcon build'?"; exit 1; }

    # 1. Start Robot 1 (The Hero Unit)
    echo "   -> Launching Robot 1 (Mock + Nav2)..."
    (python3 src/simulation_manager/src/mock_robot.py --ros-args -r __node:=mock_robot_robot_1 -p robot_id:=robot_1 -p initial_x:=2.0 -p initial_y:=2.0 -p publish_map_tf:=False) &
    
    # 1b. Launch Nav2 for Robot 1 (Wait 2s for robot to spawn)
    sleep 2
    (ros2 launch amr_navigation amr_nav.launch.py namespace:=robot_1 use_sim_time:=False) &

    # 2. Start Robot 2 (Mock Only)
    (python3 src/simulation_manager/src/mock_robot.py --ros-args -r __node:=mock_robot_robot_2 -p robot_id:=robot_2 -p initial_x:=5.0 -p initial_y:=3.0) &
    
    # 3. Start Robot 3 (Mock Only)
    (python3 src/simulation_manager/src/mock_robot.py --ros-args -r __node:=mock_robot_robot_3 -p robot_id:=robot_3 -p initial_x:=8.0 -p initial_y:=5.0) &
    
    wait
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
