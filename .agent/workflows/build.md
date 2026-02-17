---
description: Build the complete stack (Optimized for WSL 2 + Bun + UV)
---

**Context**: Execute this in a **WSL 2 Terminal** (Bash).

1. **Install Dependencies (Parallel)**

   ```bash
   # Frontend & Backend & Shared (Bun)
   cd apps/frontend && bun install &
   cd apps/backend && bun install &
   cd packages/shared-types && bun install &
   cd packages/protocol-definitions && bun install &

   # ML Service (UV + Venv)
   cd apps/ml-service
   # Create venv if not exists
   if [ ! -d ".venv" ]; then
       uv venv
   fi
   # Install deps
   source .venv/bin/activate
   uv pip install -r requirements.txt &

   wait
   echo "Dependencies installed."
   ```

2. **Build Shared Libraries (Critical Prerequisite)**

   ```bash
   cd packages/shared-types && bun run build
   cd ../protocol-definitions && bun run build
   echo "Shared libraries built."
   ```

3. **Build Services**

   ```bash
   # Backend
   cd apps/backend && bun run build

   # Frontend
   cd apps/frontend && bun run build
   ```

4. **Build Robotics (Colcon)**
   ```bash
   cd robotics/ros2_ws
   # Source ROS 2 (Humble)
   source /opt/ros/humble/setup.bash
   # Build with symlinks for dev speed
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
