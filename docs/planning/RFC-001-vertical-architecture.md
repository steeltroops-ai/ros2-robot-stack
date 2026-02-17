# Architectural Start: Vertical Feature Slicing Strategy

## 1. The Core Philosophy

We do **not** build layers (all DB, then all API). We build **Vertical Features**.
A vertical feature is a self-contained slice of functionality that cuts through the Frontend, Backend, ML, and Robotics layers.

## 2. Decoupling Strategy

### Frontend (`apps/frontend`)

- **Structure**: `components/features/[feature_name]` (e.g., `components/features/telemetry`, `components/features/map`).
- **Rule**: A component should be isolated. It imports its own data hooks.
- **Robot Isolation**: Use dynamic routing `/robots/[id]/dashboard`. Components take `robotId` as a prop.

### Backend (`apps/backend`)

- **Structure**: `src/modules/[module_name]` (e.g., `src/modules/fleet`, `src/modules/auth`).
- **Rule**: A module contains its own Service, Controller, and DTOs. Modules communicate via Event Bus or Public Interface.

### ROS 2 (`robotics/ros2_ws`)

- **Structure**: Feature-based packages (e.g., `navigation_stack`, `vision_pipeline`).
- **Rule**: Use standard interfaces. Do not depend on proprietary backend logic.

## 3. Workflow

1.  **Plan**: Write `docs/planning/RFC-001-[feature].md`.
2.  **Define Contracts**: Update `packages/shared-types`.
3.  **Implement Robot Loop**: Add ROS node/topic.
4.  **Implement Backend Bridge**: Update Node.js module.
5.  **Implement Frontend UI**: Add React component.
6.  **Report**: Write `docs/reports/2026-02-17-feature-complete.md`.
