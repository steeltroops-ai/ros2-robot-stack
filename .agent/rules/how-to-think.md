---
trigger: always_on
---

# AI Agent Thinking Mental Models

## 1. Think in Hardware Layers

Before writing any code, the agent must ask:

1. Is this per-pixel / per-vertex? → GPU / Three.js shader
2. Is this control logic (100Hz)? → ROS 2 C++ / Python node
3. Is this orchestration/routing? → TypeScript (Node.js Backend)
4. Is this visualization? → React / React Three Fiber
5. Is this ML inference? → Python (FastAPI + PyTorch)

No cross-layer leakage. If it violates this, it must justify it.

## 2. Think in Frame Budget

Every feature must be evaluated as:

1. How many ms does this cost?
2. Is this inside the control loop (10ms budget)?
3. Is this inside the render loop (16ms budget at 60fps)?
4. Does this increase WebSocket message size?

If the answer increases frame cost without measurable gain, reject it.

## 3. Think in Coordinate Systems

Three coordinate systems in play:

| System     | X       | Y       | Z       |
| :--------- | :------ | :------ | :------ |
| ROS 2 ENU  | Forward | Left    | Up      |
| Three.js   | Right   | Up      | Camera  |
| OccGrid    | Column  | Row     | N/A     |

Conversions MUST be explicit and documented.

## 4. Think Determinism First

Agent must ask:

1. Is this deterministic?
2. Does this break reproducibility?
3. Does this affect robot safety?

Debug mode must always produce identical results.

## 5. Think Profiling Before Optimizing

Before optimizing:

1. Identify CPU time (Node.js event loop, ROS callbacks).
2. Identify GPU time (Three.js draw calls, shader complexity).
3. Identify network bandwidth (message size * frequency).

No speculative optimization allowed.

## 6. Think in 3D Performance

Inside R3F / Three.js:

1. Creating geometry every frame? → P0 VIOLATION
2. Using useState for 60Hz data? → P0 VIOLATION (use useRef)
3. Causing React re-renders for pose updates? → P0 VIOLATION
4. Can identical robots use InstancedMesh? → USE IT

## 7. Think in Invariants

Agent must hoist invariants:

1. Precompute outside render loop.
2. Precompute outside frame callback.
3. Cache glTF models on first load.

Never recompute constants inside hot loops.

## 8. Think in Robot Safety

For any command sent to a robot:

1. Is the velocity within safe limits?
2. Is there a timeout/deadman switch?
3. What happens if the WebSocket disconnects mid-command?
4. Does the robot have a local safety controller?

Safety is non-negotiable.

## 9. Think in Minimal Surface API

Backend ↔ Frontend boundary must stay small. Agent must resist:

1. Sending raw ROS messages to frontend (transform to clean DTOs).
2. Streaming unnecessary fields (send only what the UI needs).
3. Adding bridge functions without a clear consumer.

Minimal interface is mandatory.

## 10. Think in State Topology

Ask:

1. Does this state change at 60Hz? → Ref / useFrame / SharedArrayBuffer
2. Does this state change on user interaction? → React State / Zustand
3. Does this state change on app load? → Constant / Config
4. Does this state change per robot? → Namespace it

Never mix 60Hz state with React Reconciliation.

## 11. Think in Resource Lifecycles (3D)

Setup Phase (One-time):
- Load glTF models
- Create materials and textures
- Initialize WebSocket connection

Loop Phase (Per-frame):
- Update pose refs
- Apply transforms in useFrame
- DO NOT create new objects

Creating a resource inside useFrame is a P0 violation.

## 12. Think in Vertical Completeness

For every feature, ALL layers must exist:

1. **Type** in shared-types
2. **ROS** topic/action defined
3. **Backend** bridge implemented
4. **Frontend** component rendering
5. **3D** scene updated if spatial

A feature missing ANY layer is NOT DONE.

## 13. Core Execution Directives

The agent must always:

1. Prioritize robot safety.
2. Prioritize real-time performance.
3. Never put control logic in the frontend.
4. Always profile before optimizing.
5. Document coordinate transformations.
6. Maintain deterministic debug mode.
7. Design for N robots, not 1.
8. Avoid abstractions that hide cost.
9. Avoid unnecessary streaming between layers.
10. Keep the URDF → glTF pipeline offline.

## 14. Think in Verification (Research Protocol)

For complex tasks (Simulations, ML, IK Solvers):

1. **Trigger Quest First**: Determine *expected* output.
2. **Verify Assumptions**: Check units, ranges, coordinate frames.
3. **Act as Expert**: If data looks wrong, STOP. Fix the process.
4. **No Hallucination**: Verify inputs and outputs explicitly.
