# Project Memory (Mutable)

**DO NOT DELETE THIS FILE.**
This file is the agent's long-term working memory. Read it at the start of every session.

---

## 1. Project Identity

- **Name**: The Omniverse Robotics Lab (ros2-robot-stack)
- **Maintainer**: Mayank (steeltroops.ai@gmail.com)
- **Flagship**: Multi-Disciplinary Robotics Portfolio (AMR, Hand, Drone, Swarm)
- **Vision Doc**: `docs/VISION.md`
- **Roadmap**: `docs/ROADMAP.md`
- **Architecture**: `docs/ARCHITECTURE.md`

## 2. Current State

### Phase 1: Foundation (COMPLETE)
### Phase 2: 3D Visualization + Multi-Robot Fleet (COMPLETE)
### Phase 3: Autonomous Navigation (Nav2) (ACTIVE)

- **Architecture**: Monorepo with strict `apps/`, `packages/`, `robotics/`, `infra/`
- **Methodology**: Vertical Feature Slicing (RFC-001)

## Key Files (Active Context)
- `apps/frontend/components/scene/RobotViewer.tsx` (R3F Scene)
- `apps/backend/src/modules/telemetry/telemetry.gateway.ts` (Multi-Robot Bridge)
- `robotics/ros2_ws/src/simulation_manager/src/mock_robot.py` (Namespaced AMR Sim)
- `scripts/dev-all.sh` (Launcher for 3 robots)

## 3. What Exists (Working)

| Component       | Path                          | Status    |
| :-------------- | :---------------------------- | :-------- |
| Mock Robot      | `robotics/.../mock_robot.py`  | Working   |
| Telemetry       | Backend → Socket.IO → UI     | Working   |
| Teleop          | UI → Socket.IO → cmd_vel     | Working   |
| 2D Map          | OccupancyGrid → Canvas       | Working   |
| Dashboard       | Next.js + Tailwind v4         | Working   |
| Dev Script      | `scripts/dev-all.sh`          | Working   |
| Shared Types    | `packages/shared-types`       | Minimal   |
| ML Service      | `apps/ml-service`             | Skeleton  |

## 4. Agent Capabilities

- **Access**: Full Linux FS (WSL 2)
- **Runtime**: Node 20, Bun 1.3, Python 3.10, ROS 2 Humble
- **Backend**: tsx (NOT Bun) for rclnodejs compatibility
- **Package Manager**: Bun (Node), UV (Python)

## 5. Active Directives

- **Decoupling**:
  - Backend: Module-based (`modules/telemetry/`, `modules/fleet/`)
  - Frontend: Feature-based (`components/scene/`, `components/fleet/`, `components/telemetry/`)
  - ROS: Package-per-capability (`simulation_manager/`, `amr_navigation/`)
- **Development Loop** (for every feature):
  1. Define Type (`packages/shared-types`)
  2. ROS Implementation (Publish/Subscribe/Action)
  3. Backend Bridge (rclnodejs → Socket.IO)
  4. Frontend UI (React/R3F)
  5. Verify end-to-end

## 6. Known Issues & Constraints

- Bun cannot run `rclnodejs` (native module crash). Use `tsx` for backend.
- Tailwind v4 uses `@import "tailwindcss"` not `@tailwind base/components/utilities`.
- CSS linter shows false positives for `@theme`, `@plugin`, `@custom-variant` (Tailwind v4 syntax).
- Map emits at 1Hz flood logs with `[Telemetry] Map received.` — throttle in future.
- Multi-robot namespace support implemented (Phase 2).
- Mock physics allows robots to pass through each other.
- **Git Constraint**: Never `git push` without user permission or end-to-end feature completion. See `.agent/rules/git-usage.md`.

## 7. Robot Types Planned

| Robot        | ROS Package         | Phase | Status   |
| :----------- | :------------------ | :---: | :------- |
| AMR          | simulation_manager  |   1   | Working  |
| AMR (Nav2)   | amr_navigation      |   3   | Planned  |
| 6-DOF Arm    | arm_control         |   4   | Planned  |
| Drone        | —                   |  7+   | Future   |

## 8. Key Documents

| Document                          | Purpose                          |
| :-------------------------------- | :------------------------------- |
| `docs/VISION.md`                  | North star — what we are building |
| `docs/ROADMAP.md`                 | Phase-by-phase delivery plan     |
| `docs/ARCHITECTURE.md`            | System architecture              |
| `docs/guides/ROS2_PACKAGE_DESIGN.md` | ROS topic naming, URDF, launch|
| `docs/guides/3D_FRONTEND_DESIGN.md`  | R3F architecture, models      |
| `docs/guides/ML_PERCEPTION.md`    | CV pipeline, API contracts       |
| `docs/planning/AGENT_HANDOFF.md`  | Session handoff state            |
| `.agent/rules/git-usage.md`    | Git & Deployment permission rules |
| `.agent/rules/persona.md`      | Agent behavioral rules           |
| `.agent/rules/how-to-think.md`    | Robotics mental models           |
