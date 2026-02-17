# RFC-002: Real-Time Telemetry Streaming

## 1. Problem Statement

We need a foundational pipeline to visualize the state of `N` robots in real-time on the web dashboard.
The latency must be low (< 50ms) and the architecture must support scaling to multiple robots without code changes.

## 2. Technical Architecture (Vertical Slice)

### A. Shared Contract (`packages/shared-types`)

We need a strict interface for the "Heartbeat" of every robot.

```typescript
export interface RobotState {
  id: string; // "robot_1"
  status: "IDLE" | "MOVING" | "ERROR";
  battery: number; // 0-100
  pose: {
    x: number;
    y: number;
    theta: number; // Orientation in radians
  };
  lastUpdate: number; // Timestamp
}
```

### B. Robotics Layer (`robotics/ros2_ws`)

- **Package**: `simulation_manager`
- **Node**: `mock_robot_node`
- **Behavior**: Publishes standard ROS messages.
  - `/robot_1/odom` (nav_msgs/Odometry)
  - `/robot_1/battery` (sensor_msgs/BatteryState) -> _Simulated sine wave for now._

### C. Backend Layer (`apps/backend`)

- **Module**: `modules/telemetry`
- **Technology**: `rclnodejs` + `Socket.io`
- **Logic**:
  1.  Discover active robots (via ROS 2 discovery).
  2.  Subscribe to `/[namespace]/odom`.
  3.  Throttle data to 60Hz.
  4.  Emit to WebSocket Room: `room:robot_1`.

### D. Frontend Layer (`apps/frontend`)

- **Component**: `components/dashboard/TelemetryCard.tsx`
- **Hook**: `useRobotTelemetry(robotId: string)`
- **Behavior**: Connects to the backend websocket, joins the specific robot room, and updates local state.

## 3. Implementation Plan

1.  **Phase 1 (Types)**: Define `RobotState` in shared package.
2.  **Phase 2 (ROS)**: Create `mock_robot.py` in ROS workspace to generate fake data.
3.  **Phase 3 (Backend)**: Set up `rclnodejs` and basic Socket.io gateway.
4.  **Phase 4 (Frontend)**: Build the React component to display `x, y, theta`.

## 4. Risks & Mitigations

- **Risk**: High frequency Odom (100Hz) flooding the websocket.
- **Mitigation**: Backend **MUST** throttle/downsample to 30Hz or 60Hz before sending to UI.
