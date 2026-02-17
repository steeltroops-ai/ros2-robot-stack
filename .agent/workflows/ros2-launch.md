---
description: Launch ROS 2 robots (WSL 2)
---

**Prerequisite**: Run in WSL Terminal.

1. Source workspace

   ```bash
   cd robotics/ros2_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

2. Launch Robot
   ```bash
   # Syntax: ros2 launch [package] [launch_file] namespace:=[id]
   ros2 launch mobile_bot_bringup main.launch.py namespace:=robot_1
   ```

**Troubleshooting (Windows/WSL)**:

- If Gazebo does not open: Ensure you are on Windows 11 (native GUI support) or have Vcxsrv running on Windows 10.
- Check `DISPLAY` variable: `echo $DISPLAY`.
