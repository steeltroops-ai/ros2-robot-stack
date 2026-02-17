# Multi-Layer Robotics Platform

A production-grade fleet management system with strict separation of concerns, simulating a real-world robotics deployment environment.

## 🚀 Quick Start (WSL 2 Only)

**WARNING**: This project uses a hybrid workflow. ROS 2 commands **must** run in WSL 2 or Docker.

### 1. Prerequisities

- **WSL 2** (Ubuntu 22.04 recommended)
- **Bun** (`curl -fsSL https://bun.sh/install | bash`)
- **UV** (Python Manager: `curl -LsSf https://astral.sh/uv/install.sh | sh`)
- **ROS 2 Humble** (Installed in WSL)
- **Docker Desktop** (With WSL 2 backend enabled)

### 2. How to Access WSL 2

If you are on Windows:

1.  Open PowerShell.
2.  Type `wsl` and hit Enter.
3.  You are now in Linux. Navigate to your project (best if cloned inside `~`):
    ```bash
    cd ~/projects/ros2-robot-stack
    ```

### 3. Initialize & Build

We use `bun` as the universal task runner.

```bash
# Install all dependencies (Frontend, Backend, Shared)
bun install

# Install Python deps for ML Service
cd apps/ml-service
uv venv
source .venv/bin/activate
uv pip install -r requirements.txt

# Return to root
cd ../..

# Build Shared Types & Apps
bun run build:all
```

### 4. Running the Stack

Run each service in a separate terminal tab (inside WSL).

**Terminal 1: Infrastructure**

```bash
cd infra
docker compose up -d postgres redis
```

**Terminal 2: Backend**

```bash
cd apps/backend
bun run dev
```

**Terminal 3: Frontend**

```bash
cd apps/frontend
bun run dev
```

**Terminal 4: ML Service**

```bash
cd apps/ml-service
source .venv/bin/activate
uvicorn main:app --reload
```

**Terminal 5: ROS 2 Simulation**

```bash
cd robotics/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch simulation_manager main.launch.py
```

## 📂 Architecture

| Service        | Location           | Runtime       | Purpose                   |
| :------------- | :----------------- | :------------ | :------------------------ |
| **Frontend**   | `apps/frontend`    | Next.js (Bun) | Operator Dashboard        |
| **Backend**    | `apps/backend`     | Node.js (Bun) | Auth, API, Robot Registry |
| **ML Service** | `apps/ml-service`  | Python (UV)   | Vision & Inference        |
| **Robotics**   | `robotics/ros2_ws` | ROS 2 Humble  | Simulation & Control      |
| **Shared**     | `packages/`        | TypeScript    | Source of Truth for Types |

## 🔗 Connectivity Model

All services run inside the **Same Network Namespace** (localhost) when using WSL 2.

- **Frontend (3000)** -> HTTP/WS -> **Backend (4000)**
- **Backend** -> TCP/DDS -> **ROS 2 Nodes**
- **ML Service (8000)** -> DDS (Topics) -> **ROS 2 Nodes**

This mimics a production cluster where services talk over a private mesh network.
