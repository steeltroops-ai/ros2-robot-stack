# Architectural Decision: Python Environments in ROS 2

## The Conflict

You asked: _"Don't we need venv in robotics?"_
The short answer: **Usually, No.** (But with a catch).

## The Technical Reason (`rclpy`)

1.  **Binary Bindings**: ROS 2 core libraries (like `rclpy`, `std_msgs`) are **C++ binaries** wrapped in Python. They are installed via `apt` into `/opt/ros/humble/lib/python3.10/site-packages`.
2.  **The Venv Trap**: A standard Python `venv` is designed to be **isolated**. It explicitly _hides_ the system packages.
    - Result: If you activate a venv and run a ROS node, it crashes with `ModuleNotFoundError: No module named 'rclpy'`.
3.  **The Workaround**: You _can_ create a venv with `--system-site-packages`, but this defeats the purpose of isolation (you inherit all the system junk anyway).

## The Decision

We apply a **Split Strategy**:

| Layer                                    | Environment        | Strategy                                                                                                                                                                                            |
| :--------------------------------------- | :----------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **ML Service** (`apps/ml-service`)       | **Strict `.venv`** | It's a standard Python web app. It needs precise versions of `torch`, `fastapi`, etc. It communicates with ROS via network/bridge, so it doesn't strictly need `rclpy` (unless it's a direct node). |
| **ROS 2 Workspace** (`robotics/ros2_ws`) | **System Python**  | We rely on `rosdep` and the OS package manager (`apt`). We do **NOT** use a venv here. **Docker** provides the isolation layer for this part of the stack.                                          |

## Why this is "Professional"

In production, your robot runs a **Docker Container**.

- Inside that container, there is only **one** python environment (the system one).
- Therefore, developing against the System Python in WSL mirrors the production Docker environment more closely than a synthetic venv.

## Exception

If you need a specific Python library (e.g., `requests`) for a ROS node:

1.  Check if there is a ROS package for it: `sudo apt install python3-requests`.
2.  If not, use `pip install --user` (be careful) or add it to `package.xml` and let `rosdep` handle it.
