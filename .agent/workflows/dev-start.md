---
description: Run the Development Stack (Hybrid Mode with Bun & UV)
---

**Context**: This workflow assumes you want to run the **Full Stack** for development.

1. **Start Infrastructure (Docker)**
   _Runs Postgres and Redis in the background._

   ```bash
   cd infra
   docker compose up -d postgres redis
   ```

2. **Start Backend (WSL)**
   _Orchestrates the platform._

   ```bash
   # New Terminal Tab
   cd apps/backend
   bun run dev
   ```

3. **Start Frontend (WSL)**
   _UI Dashboard._

   ```bash
   # New Terminal Tab
   cd apps/frontend
   bun run dev
   ```

4. **Start ML Service (WSL)**
   _Inference Engine (Isolated)._

   ```bash
   # New Terminal Tab
   cd apps/ml-service
   source .venv/bin/activate
   uvicorn main:app --reload --port 8000
   ```

5. **Start Simulation (WSL)**
   _Gazebo + ROS 2 Core._
   ```bash
   # New Terminal Tab
   cd robotics/ros2_ws
   source install/setup.bash
   ros2 launch simulation_bringup world.launch.py
   ```
