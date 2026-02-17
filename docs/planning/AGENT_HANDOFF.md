# Agent Handoff: Systems Architecture & Project State

## 1. Project Objective

Build a production-grade multi-robot fleet management platform.

- **Goal**: High-frequency telemetry dashboard with control capabilities.
- **Architecture**: Vertical Feature Slicing (RFC-001).
- **Environment**: Windows 11 (Host) + WSL 2 Ubuntu 22.04 (Target).

## 2. Technical Stack

| Layer          | Identity     | Technology                                            |
| :------------- | :----------- | :---------------------------------------------------- |
| **Frontend**   | Dashboard    | Next.js 14+, TailwindCSS, shadcn/ui, Socket.io-client |
| **Backend**    | Orchestrator | Fastify, Socket.io, rclnodejs (ROS 2 Bridge)          |
| **ML Service** | Perception   | Python (FastAPI, UV, PyTorch placeholder)             |
| **Robotics**   | Hardware/Sim | ROS 2 Humble (C++ / Python)                           |
| **Monorepo**   | Task Runner  | Bun (Root package.json)                               |

## 3. Current Implementation Status (Feature: Real-Time Telemetry)

We have successfully implemented the "Hello World" of the robotics stack.

### A. Robotics Layer

- **Package**: `simulation_manager` (created via `ros2 pkg create`).
- **Script**: `mock_robot.py` (simulates a robot moving in a circle, publishing `/odom` and `/battery`).

### B. Backend Layer

- **Module**: `modules/telemetry` (RFC-002).
- **Gateway**: `telemetry.gateway.ts` (bridging ROS 2 topics to WebSockets with 20Hz throttling).

### C. Frontend Layer

- **Design System**: Industrial Enterprise (globals.css + shadcn/ui).
- **Hook**: `useFleetTelemetry.ts` (manages socket state).
- **Dashboard**: Live table showing robot coordinates, heading, and battery percentage.

## 4. Key Architectural Decisions (ADRs)

- **ADR-001**: Use scripts (`/scripts/`) and Bun commands instead of shell aliases.
- **ADR-002**: Use System Python for ROS 2 (no venv in `robotics/`) to ensure `rclnodejs` and `rclpy` binding access.
- **ADR-003**: Vertical Feature Slicing (Feature logic isolated across layers).
- **ADR-004**: Adopted `shadcn/ui` for professional enterprise components.

## 5. Execution Context

- **Codebase**: For performance, the project is cloned in WSL at `~/projects/ros2-robot-stack`.
- **Master Script**: `./scripts/dev-all.sh` (launches ROS, Backend, and Frontend in parallel).
- **Git**: Branch `main` is up-to-date on GitHub.

## 6. Next Steps

1.  **Phase 3 Completion**: Fix remaining native compilation issues for `rclnodejs` in the WSL environment (specifically message generation).
2.  **Phase 4 (Control)**: Implement "Teleoperation". Add a UI component to send `cmd_vel` (Twist messages) to the robot.
3.  **Phase 5 (Map)**: Integrate a 2D map visualization (using RVIZ-like Canvas or Leaflet/R3F).

## 7. Operational Constraints

- **Perspective**: System Architect (First-person "I have implemented").
- **No Emojis**.
- **Communication**: Professional, dense, technical.
- **Workflows**: Use `./scripts/dev-all.sh` for testing.
