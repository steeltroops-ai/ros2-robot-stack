# AI Agent Mental Models: The INTJ Robotics Architect

## 1. Think in Isolation (Dependency Hygiene)

- **Node.js**: Use `bun` for install. Use `tsx` for rclnodejs runtime.
- **Python**: NEVER install global packages.
  - **Bad**: `pip install pandas` (Pollutes system/WSL).
  - **Good**: `uv venv` → `source .venv/bin/activate` → `uv pip install pandas`.
- **ROS 2**: Use `rosdep` for system deps. Keep Python logic in nodes.
- **Frontend**: Use `bun add` for npm packages. Never `npm install`.

## 2. Think in Control Loops (The 100Hz Rule)

Every decision regarding the robot's logical core must respect the control loop.

- **Input**: Sensor Data (LiDAR, Odom, Cameras, JointState).
- **Process**: State Estimation → Path Planning → Control / IK Solve.
- **Output**: Actuator Commands (Velocity, Joint Trajectory, Gripper).
- **Constraint**: This loop runs at **100Hz (10ms)**. Any logic here must be **O(1) or O(log n)**. NO blocking HTTP calls. NO heavy serialization.

## 3. Think in State Synchronization (The Twin Problem)

The "Robot" and the "Dashboard" are two different timelines.

- **Robot Time**: Continuous, real-time, authoritative (100Hz).
- **Dashboard Time**: Discrete, delayed, observational (30-60Hz).
- **Rule**: The Dashboard computes **nothing**. It only **renders** the state it receives.
- **3D Rule**: Pose data → useRef → useFrame update. Never React useState for 60Hz data.

## 4. Think in Coordinate Systems

Three coordinate systems must be mapped correctly:

| System     | Convention | X       | Y       | Z       |
| :--------- | :--------- | :------ | :------ | :------ |
| ROS 2      | ENU        | Forward | Left    | Up      |
| Three.js   | Y-up       | Right   | Up      | Towards |
| OccupGrid  | Row-column | →       | ↑       | N/A     |

**Conversion (2D ground robots)**:
```
Three.x = ROS.x
Three.y = 0.1 (elevation)
Three.z = -ROS.y
Three.rotation.y = -ROS.theta
```

## 5. Think in Types as Safety Interlocks

- **Loose Typing** = **Physical Damage**.
- If the frontend sends a string "fast" instead of a float `1.5`, the robot interprets `0.0` or crashes.
- **Shared Types** (`packages/shared-types`) are not convenience — they are safety interlocks.
- Every WebSocket event must have a typed contract. Every ROS message must map to a TypeScript interface.

## 6. Think in Failure Modes (Defensive Architecture)

- **Network Loss**: Robot enters "Safe Stop". Dashboard shows "Reconnecting".
- **Process Crash**: Robot continues collision avoidance. It does not freeze.
- **Latency Spike**: Control loop must not stutter. Backend buffers, dashboard interpolates.
- **ML Service Down**: Robot navigates without detections. Dashboard shows "Vision Offline".

## 7. Think in Scalability (The N+1 Robot)

- Never design for "The Robot". Design for "Robot[i]".
- **Namespace Everything**: Nodes, Topics, Services, Actions.
- **Stateless Backend**: Backend reads from ROS graph, not local variables.
- **3D Scaling**: Use InstancedMesh for 10+ robots. LOD for distant robots.

## 8. Think in Layers

Before writing any code, the agent must ask:

1. Is this per-pixel / per-vertex? → GPU / Three.js shader
2. Is this control logic? → ROS 2 (C++ or Python)
3. Is this orchestration/routing? → Node.js Backend
4. Is this visualization? → React / R3F
5. Is this inference? → Python ML Service

No cross-layer leakage. If it violates this, it must justify it.

## 9. Think in 3D Performance

For Three.js / R3F:

1. Does this create geometry every frame? → VIOLATION
2. Does this setState at 60Hz? → VIOLATION (use useRef)
3. Does this re-render the React tree for pose updates? → VIOLATION
4. Does this create materials inside useFrame? → VIOLATION
5. Can this be instanced? → USE InstancedMesh

## 10. Think in URDF → glTF Pipeline

For robot visualization:

1. Does the URDF have visual meshes? → Convert to glTF
2. Are we parsing URDF in the browser at runtime? → BAD (pre-convert)
3. Is the model < 5MB? → OK for web delivery
4. Does the arm model have joint transforms? → Map to Three.js bone hierarchy

## 11. Think in Feature Completeness (Vertical)

For every feature, verify ALL layers:

1. **Type defined** in `packages/shared-types`? 
2. **ROS topic/service/action** publishing/subscribing?
3. **Backend bridge** subscribing and emitting via Socket.IO?
4. **Frontend component** rendering the data?
5. **3D scene** updated if spatial?
6. **Tests** covering the happy path?

If ANY layer is missing, the feature is NOT done.

## 12. Think in Profiling Before Optimizing

Before optimizing:

1. Identify CPU time (Node.js event loop, ROS callbacks).
2. Identify GPU time (Three.js draw calls, shader complexity).
3. Identify network bandwidth (WebSocket message size * frequency).
4. Identify memory (GPU textures, ROS message buffers).

No speculative optimization allowed. Measure first.
