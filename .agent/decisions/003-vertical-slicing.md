# Architectural Decision: Vertical Feature Slicing

## The Problem

Monoliths (or even Monorepos) often suffer from "Horizontal Layering".

- **Horizontal**: "I will write all the database schemas. Then all the API routes. Then all the frontend components."
- **Result**: You have 3 broken layers for weeks. Nothing works until the very end.

## The Solution: Vertical Features

We build **End-to-End Features** one at a time. A feature is a vertical slice through the entire stack.

### 1. The Structure of a Vertical Slice

A single folder in your brain (not necessarily on disk, though we can group by domain) contains:

- **Database Schema** (Postgres Migration)
- **Backend Service** (API Route + Controller)
- **Shared Type** (DTO)
- **Frontend Component** (UI Widget)
- **ROS Message** (Topic Definition)

### 2. Feature-First Directory Structure (Future State)

Instead of:

```
/controllers
  /auth_controller.ts
  /robot_controller.ts
/models
  /user.ts
  /robot.ts
```

We move towards **Domain Modules** (in Backend):

```
/modules
  /auth
    /auth.controller.ts
    /auth.service.ts
    /auth.schema.ts
  /robot-fleet
    /fleet.controller.ts
    /fleet.service.ts
```

### 3. The Coupling Rule

- **Loose Coupling Between Verticals**: The "Auth" module should not know about "Robot Path Planning". They talk via **Pub/Sub (Events)** or **Public Service Interface**.
- **Tight Coupling Within Verticals**: The "Robot Map" UI component requires the "Robot Map" API endpoint. They change together. This is good.

## Execution Strategy: "The Tracer Bullet"

For every new feature (e.g., "Add Lidar Visualization"):

1.  **Define the Interface** (`packages/shared-types`): `LidarScan { ranges: number[] }`
2.  **Implement ROS Node**: Publish `sensor_msgs/LaserScan`.
3.  **Implement Backend Bridge**: Subscribe to ROS, forward to Websocket.
4.  **Implement Frontend**: Render points on HTML Canvas.
5.  **Verify**: Does it light up? Yes. **Done.**

## Mental Check

- **Can I delete this feature?** If I delete the "Mapping" folder, does the "Login" screen break?
  - **No**: Good. Decoupled.
  - **Yes**: Bad. Tight coupling.
