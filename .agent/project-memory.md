# Project Memory (Mutable)

**DO NOT DELETE THIS FILE.**
This file serves as the long-term memory for active development.

## 1. Current State

- **Architecture**: Monorepo with strict `apps/` and `packages/`.
- **Methodology**: **Vertical Feature Slicing**. We build _End-to-End Features_, not layers.

## 2. Agent Capabilities

- **Access**: Full Windows FS + WSL Execution.
- **Workflow**: `standard-version` commits. `bun run build:all`.

## 3. Active Directives

- **Decoupling**:
  - **Backend Domains**: Use module-based structure (e.g., `modules/robot/`, `modules/auth/`).
  - **Frontend Features**: Group components by feature (e.g., `components/dashboard/telemetry/`).
  - **ROS Packages**: One package per major function (e.g., `robot_navigation`, `robot_vision`).
- **Development**:
  - "Can I run this feature in isolation?" -> YES.
  - "Does breaking 'Map' break 'Login'?" -> NO.

## 4. Feature Checklist (The Loop)

For every NEW feature request:

1.  **Define Type** (`shared-types`).
2.  **ROS Implementation** (Publish/Sub).
3.  **Backend Implementation** (Bridge/API).
4.  **Frontend Implementation** (UI).
5.  **Verify**.
