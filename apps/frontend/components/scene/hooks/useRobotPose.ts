"use client";

import { useRef, useEffect } from "react";
import { useFrame } from "@react-three/fiber";
import * as THREE from "three";

export interface RobotPoseInput {
  x: number;
  y: number;
  theta: number;
}

export interface RobotVisualState {
  linear: number;
  angular: number;
  vy: number;
  status: string;
}

/**
 * useRobotPose — Decoupled robot pose interpolation.
 * 
 * Responsibilities:
 * 1. Smoothly interpolate from current pose → target pose (frame-rate independent)
 * 2. Update the Three.js group transform
 * 3. Compute visual velocity state for wheel animations
 * 
 * Does NOT touch the camera. Does NOT handle navigation. Pure SRP.
 */
export function useRobotPose(
  robotRef: React.RefObject<THREE.Group>,
  pose: RobotPoseInput
) {
  const targetPose = useRef<RobotPoseInput>({ ...pose });
  const currentPose = useRef<RobotPoseInput>({ ...pose });

  const robotStateRef = useRef<RobotVisualState>({
    linear: 0,
    angular: 0,
    vy: 0,
    status: "idle",
  });

  // Smoothing for velocity calculation to prevent wheel flicker
  const smoothVel = useRef({ x: 0, y: 0, w: 0 });

  // When external pose updates, update the target
  useEffect(() => {
    targetPose.current = { x: pose.x, y: pose.y, theta: pose.theta };
  }, [pose.x, pose.y, pose.theta]);

  // Force-sync on mount
  useEffect(() => {
    currentPose.current = { ...pose };
    targetPose.current = { ...pose };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useFrame((_state, delta) => {
    if (!robotRef.current) return;

    const dt = Math.min(delta, 0.05); // Clamp to avoid huge jumps on tab-switch

    // Frame-rate independent smoothing
    // At 60fps: alpha ≈ 0.15, at 30fps: alpha ≈ 0.27
    const alpha = 1 - Math.pow(0.001, dt);

    // --- Position ---
    const dx = (targetPose.current.x - currentPose.current.x) * alpha;
    const dy = (targetPose.current.y - currentPose.current.y) * alpha;
    currentPose.current.x += dx;
    currentPose.current.y += dy;

    // --- Rotation (shortest path, wrapping at ±π) ---
    let dAngle = targetPose.current.theta - currentPose.current.theta;
    while (dAngle > Math.PI) dAngle -= 2 * Math.PI;
    while (dAngle < -Math.PI) dAngle += 2 * Math.PI;
    const dTheta = dAngle * alpha;
    currentPose.current.theta += dTheta;

    // --- Apply to mesh (ROS: X forward, Y left → Three.js: X right, Z backward) ---
    robotRef.current.position.set(currentPose.current.x, 0, -currentPose.current.y);
    robotRef.current.rotation.y = currentPose.current.theta;

    // --- Compute visual velocity for wheel animations ---
    const globalMove = new THREE.Vector3(dx, 0, -dy);
    const localMove = globalMove
      .clone()
      .applyQuaternion(robotRef.current.quaternion.clone().invert());

    const filter = 0.2;
    smoothVel.current.x += (localMove.x / dt - smoothVel.current.x) * filter;
    smoothVel.current.y += (-localMove.z / dt - smoothVel.current.y) * filter;
    smoothVel.current.w += (dTheta / dt - smoothVel.current.w) * filter;

    const isMoving =
      Math.abs(smoothVel.current.x) > 0.02 ||
      Math.abs(smoothVel.current.y) > 0.02 ||
      Math.abs(smoothVel.current.w) > 0.02;

    robotStateRef.current = {
      linear: smoothVel.current.x,
      angular: smoothVel.current.w,
      vy: smoothVel.current.y,
      status: isMoving ? "moving" : "idle",
    };
  });

  return {
    currentPose,
    robotStateRef,
  };
}
