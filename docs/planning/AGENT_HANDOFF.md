# Agent Handoff: Systems Architecture & Project State

## 1. Project Identity

- **Name**: Multi-Layer Robotics Platform (ros2-robot-stack)
- **Objective**: Production-grade multi-robot fleet management platform with 3D web control.
- **Flagship**: Warehouse AMR Fleet with AI Perception.
- **Architecture**: Vertical Feature Slicing (RFC-001).
- **Environment**: WSL 2 Ubuntu 22.04.

## 2. Technical Stack

| Layer          | Identity     | Technology                                              |
| :------------- | :----------- | :------------------------------------------------------ |
| **Frontend**   | Dashboard    | Next.js 16, Tailwind v4, shadcn/ui, React Three Fiber   |
| **Backend**    | Orchestrator | Fastify, Socket.IO, rclnodejs (ROS 2 Bridge)            |
| **ML Service** | Perception   | Python (FastAPI, UV, PyTorch)                            |
| **Robotics**   | Hardware/Sim | ROS 2 Humble (Python mock, future Gazebo + Nav2)         |
| **Monorepo**   | Task Runner  | Bun (root), tsx (backend runtime)                        |

## 3. Current Phase: 1 (Foundation) — COMPLETE

### Implemented Features

| Feature      | Robotics                  | Backend                    | Frontend                  |
| :----------- | :------------------------ | :------------------------- | :------------------------ |
| Telemetry    | mock_robot.py → /odom     | rclnodejs sub → Socket.IO | useFleetTelemetry → Table |
| Teleop       | subscribe /cmd_vel        | Socket.IO → publish        | TeleopPanel buttons       |
| 2D Map       | publish /map (OccGrid)    | sub /map → Socket.IO       | MapDisplay (Canvas)       |
| Battery      | publish /battery          | sub → Socket.IO            | Battery column in table   |

### Key Architecture

```
mock_robot.py (10Hz) → ROS Topics → rclnodejs bridge → Socket.IO → Next.js Dashboard
```

## 4. Phase 2 Target: 3D Visualization + Multi-Robot

| Task                          | Layer     | Priority |
| :---------------------------- | :-------- | :------- |
| React Three Fiber 3D scene    | Frontend  | P0       |
| Multi-robot launch file       | Robotics  | P0       |
| Fleet sidebar + robot cards   | Frontend  | P0       |
| Dynamic robot discovery       | Backend   | P0       |
| glTF model loader             | Frontend  | P1       |
| Robot selection + highlight   | Frontend  | P1       |

## 5. Execution Context

- **Codebase**: `~/projects/ros2-robot-stack`
- **Master Script**: `./scripts/dev-all.sh`
- **Backend**: `tsx src/index.ts` (NOT Bun — native module issue)
- **Frontend**: `next dev` (Turbopack)
- **Styling**: Tailwind CSS v4 (`@import "tailwindcss"` syntax)
- **Ports**: Frontend=3000, Backend=4000, ML=8000

## 6. Critical Documents

| Document                              | Read When                      |
| :------------------------------------ | :----------------------------- |
| `docs/VISION.md`                      | Understanding project scope    |
| `docs/ROADMAP.md`                     | Planning next phase            |
| `docs/ARCHITECTURE.md`               | System structure questions     |
| `docs/guides/ROS2_PACKAGE_DESIGN.md` | Creating ROS packages/topics   |
| `docs/guides/3D_FRONTEND_DESIGN.md`  | Building 3D scene              |
| `docs/guides/ML_PERCEPTION.md`       | AI/ML integration              |
| `.agent/project-memory.md`           | Current state + known issues   |
| `.agent/rules/system-manifesto.md`   | Agent behavioral rules         |

## 7. Operational Constraints

- **Perspective**: System Architect (First-person "I have implemented").
- **No Emojis** in documentation.
- **Communication**: Professional, dense, technical.
- **Workflows**: Use `./scripts/dev-all.sh` for testing.
- **Backend**: MUST use tsx, NOT Bun, for rclnodejs compatibility.
- **Tailwind**: v4 syntax — use `@theme {}` for variables, `@import "tailwindcss"` at top.

## 8. Known Working Ports

| Service     | Port  | Status  |
| :---------- | :---- | :------ |
| Frontend    | 3000  | Active  |
| Backend     | 4000  | Active  |
| ML Service  | 8000  | Planned |
| PostgreSQL  | 5432  | Planned |
| Redis       | 6379  | Planned |
