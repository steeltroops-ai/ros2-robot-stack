---
trigger: always_on
---

# AI Agent Thinking Mental Models

## 1. Think in Hardware Layers

Before writing any code, the agent must ask:

1. Is this per-pixel? → GPU
1. Is this high-precision math? → Rust
1. Is this orchestration? → TypeScript
1. Is this UI? → React

No cross-layer leakage. If it violates this, it must justify it.

## 2. Think in Frame Budget

Every feature must be evaluated as:

1. How many ms does this cost?
1. Is this inside the hot path?
1. Is this inside the integration loop?
1. Does this increase average ray steps?

If the answer increases frame cost without measurable gain, reject it.

## 3. Think in Curvature, Not Distance

For physics decisions:

1. Adaptive step must scale with curvature (M / r³).
1. Horizon region gets precision.
1. Far region gets speed.
1. Not arbitrary distance scaling.

## 4. Think Determinism First

Agent must ask:

1. Is this deterministic?
1. Does this break reproducibility?
1. Does this affect energy conservation?

Debug mode must always be stable.

## 5. Think Profiling Before Optimizing

Before optimizing:

1. Identify CPU time.
1. Identify GPU time.
1. Identify memory bandwidth usage.

No speculative optimization allowed.

## 6. Think Branch Divergence

Inside shader:

1. Will this branch cause warp divergence?
1. Can this be written branchless?
1. Can this be masked instead of conditionally broken?

GPU reasoning must be explicit.

## 7. Think in Invariants

Agent must hoist invariants:

1. Precompute outside loop.
1. Precompute outside frame.
1. Precompute outside shader if possible.

Never recompute constants inside hot loops.

## 8. Think in Numerical Stability

For integrators:

1. What is local truncation error?
1. Is method symplectic?
1. Does adaptive dt maintain stability?
1. Does f32 precision drift accumulate?

No “looks correct” acceptance.

## 9. Think in Minimal Surface API

Rust ↔ TS boundary must stay small. Agent must resist:

1. Exporting large structs.
1. Streaming big buffers.
1. Adding unnecessary bridge functions.

Minimal interface is mandatory.

## 10. Think in State Topology

Ask:

1. Does this state change at 60Hz? → Ref / Texture / SharedArrayBuffer
1. Does this state change on user interaction? → React State / Zustand
1. Does this state change on app load? → Constant / Config

Never mix 60Hz state with React Reconciliation.

## 11. Think in Resource Lifecycles (WebGPU)

Setup Phase (One-time):

- Create Buffers
- Compile Pipelines
- Create BindGroups

Loop Phase (Per-frame):

- Write Buffers (only deltas)
- Encode Passes
- Submit WorkReference

Creating a resource inside the loop is a P0 violation.

## 12. Think in Research Credibility

If claiming Kerr:

1. Is it actually Kerr?
1. Or pseudo-Newtonian with hacks?
1. Agent must label approximations explicitly.

## 13. Core Execution Directives

The agent must always:

1. Prioritize physics correctness.
1. Prioritize GPU execution for parallel workloads.
1. Never move per-pixel logic to CPU.
1. Always profile before optimizing.
1. Document approximations.
1. Maintain deterministic debug mode.
1. Design for future WebGPU migration.
1. Avoid abstractions that hide cost.
1. Avoid unnecessary streaming between layers.

## 14. Think in Verification (Research Protocol)

For complex tasks (Simulations, ML, Data Generation), you MUST adhere to the **Research Team Protocol** (`.agent/rules/research-team-protocol.md`).

1.  **Trigger Quest First**: Do not execute blindly. Determine *expected* output.
2.  **Verify Assumptions**: Check units, ranges, and patterns on small samples.
3.  **Act as Expert**: If data looks wrong, STOP. Fix the process.
4.  **No Hallucination**: Verify inputs and outputs explicitly.

