# Multi-robot fleet management platform.**

Simulate real hardware robots through ROS 2. Control them through a web-based 3D dashboard. Add AI perception for intelligent autonomy.

**Maintained by**: Mayank (steeltroops.ai@gmail.com)

---

## What This Is

A full-stack robotics platform that mirrors how real robotics companies build their infrastructure:

- **ROS 2 Humble** for robot simulation and control
- **Node.js Backend** bridging ROS 2 to the web via WebSocket
- **Next.js Dashboard** with real-time 3D visualization (React Three Fiber)
- **ML Service** for computer vision and intelligent perception (PyTorch + FastAPI)

```
Browser (3D Dashboard) ←→ Node.js Backend ←→ ROS 2 ←→ ML Service
```

## Architecture

| Service        | Location           | Runtime       | Purpose                     |
| :------------- | :----------------- | :------------ | :-------------------------- |
| **Frontend**   | `apps/frontend`    | Next.js 16    | 3D Operator Dashboard       |
| **Backend**    | `apps/backend`     | Fastify + tsx | ROS 2 Bridge, API, WS       |
| **ML Service** | `apps/ml-service`  | FastAPI + UV  | Vision & Inference          |
| **Robotics**   | `robotics/ros2_ws` | ROS 2 Humble  | Simulation & Control        |
| **Shared**     | `packages/`        | TypeScript    | Type Contracts              |

## Supported Robot Types

| Robot            | Phase | Status    |
| :--------------- | :---: | :-------- |
| AMR (Mobile)     |   1   | Working   |
| AMR + Nav2       |   3   | Planned   |
| 6-DOF Arm        |   4   | Planned   |
| Drone            |  7+   | Planned   |
| Swarm (10+ AMRs) |   7   | Planned   |

## Quick Start

### Prerequisites

- **WSL 2** (Ubuntu 22.04)
- **Bun** (`curl -fsSL https://bun.sh/install | bash`)
- **UV** (`curl -LsSf https://astral.sh/uv/install.sh | sh`)
- **ROS 2 Humble** (installed in WSL)

### Setup

```bash
# Clone into WSL
cd ~/projects
git clone <repo-url> ros2-robot-stack
cd ros2-robot-stack

# Install dependencies
bun install

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build ROS workspace (first time only)
cd robotics/ros2_ws
colcon build
source install/setup.bash
cd ../..
```

### Run the Full Stack

```bash
./scripts/dev-all.sh
```

This launches:
- **Frontend**: http://localhost:3000
- **Backend**: http://localhost:4000
- **ROS 2 Mock Robot**: Publishing telemetry on `/odom`, `/battery`, `/map`

### Manual Launch (Individual Services)

```bash
# Terminal 1: ROS 2 Simulation
source /opt/ros/humble/setup.bash
cd robotics/ros2_ws && source install/setup.bash
python3 src/simulation_manager/src/mock_robot.py

# Terminal 2: Backend
cd apps/backend
bun run dev

# Terminal 3: Frontend
cd apps/frontend
bun run dev
```

## Repository Structure

```
ros2-robot-stack/
├── apps/
│   ├── frontend/          # Next.js + React Three Fiber Dashboard
│   ├── backend/           # Fastify + Socket.IO + rclnodejs Bridge
│   └── ml-service/        # FastAPI + PyTorch ML Service
├── packages/
│   └── shared-types/      # TypeScript type contracts
├── robotics/
│   └── ros2_ws/           # ROS 2 workspace (simulation, nav, arm)
├── infra/
│   └── docker-compose.yml # Docker deployment
├── docs/
│   ├── VISION.md          # Project vision and goals
│   ├── ROADMAP.md         # Phase-by-phase delivery plan
│   ├── ARCHITECTURE.md    # System architecture
│   ├── guides/            # Design guides (3D, ROS, ML)
│   └── planning/          # RFCs and handoff docs
└── scripts/
    └── dev-all.sh         # Launch entire dev stack
```

## Roadmap

| Phase | Feature                          | Status    |
| :---: | :------------------------------- | :-------- |
|   1   | Basic AMR + Telemetry + 2D Map   | Complete  |
|   2   | 3D Visualization + Multi-Robot   | Next      |
|   3   | Autonomous Navigation (Nav2)     | Planned   |
|   4   | Robotic Arm + Joint Control      | Planned   |
|   5   | ML Perception + CV Overlay       | Planned   |
|   6   | LLM Natural Language Control     | Planned   |
|   7   | Swarm Robotics (10+ robots)      | Planned   |
|   8   | Production Hardening + Auth      | Planned   |

## Tech Stack

| Category        | Technology                                    |
| :-------------- | :-------------------------------------------- |
| Frontend        | Next.js 16, React 19, Tailwind v4, shadcn/ui  |
| 3D Rendering    | React Three Fiber, Three.js, @react-three/drei |
| Backend         | Fastify, Socket.IO, rclnodejs, tsx             |
| ML/AI           | FastAPI, PyTorch, OpenCV, YOLOv8               |
| Robotics        | ROS 2 Humble, Nav2, MoveIt2, Gazebo            |
| Package Manager | Bun (Node), UV (Python)                        |
| Infrastructure  | Docker, PostgreSQL, Redis                      |

## Documentation

- [Vision](docs/VISION.md) — What we are building and why
- [Roadmap](docs/ROADMAP.md) — Phase-by-phase delivery plan
- [Architecture](docs/ARCHITECTURE.md) — System architecture and data flow
- [3D Frontend Guide](docs/guides/3D_FRONTEND_DESIGN.md) — React Three Fiber patterns
- [ROS2 Package Design](docs/guides/ROS2_PACKAGE_DESIGN.md) — Topic naming and URDF structure
- [ML Perception](docs/guides/ML_PERCEPTION.md) — Computer vision pipeline

## License

**PROPRIETARY / SOURCE AVAILABLE**

This is **Private Property** of Mayank (steeltroops.ai).

- **Personal Use**: Free for learning, research, and contribution.
- **Commercial Use**: **Requires Partnership Agreement**.
  - To use this software for profit, selling, or business operations, you must obtain a commercial license.
  - Contact: **steeltroops.ai@gmail.com** for partnership inquiries.

See [LICENSE](LICENSE) for full terms.
