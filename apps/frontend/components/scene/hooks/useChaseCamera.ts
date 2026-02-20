"use client";

import { useRef, useMemo, useEffect, useCallback } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import * as THREE from "three";

/**
 * useChaseCamera — 3rd Person Chase Camera (Racing Game Style)
 *
 * ## How it works:
 * 1. Computes a "desired" camera position BEHIND the robot based on its heading.
 *    - `offset` = (distance behind, height above, 0) rotated by robot heading
 * 2. Smoothly lerps the actual camera toward the desired position.
 * 3. Smoothly lerps the look-at target toward the robot (slightly ahead of it).
 * 4. When the user is NOT driving, allows free-orbit mode.
 *    On drive start, smoothly transitions back to chase mode.
 *
 * ## Config:
 * - `distance`: How far behind the robot the camera sits (default: 3)
 * - `height`: Camera height above ground (default: 2)
 * - `lookAheadDistance`: How far ahead of the robot the camera looks (default: 1)
 * - `followSpeed`: How fast the camera catches up (default: 4)
 * - `rotationSpeed`: How fast the camera swings behind on turns (default: 3)
 */

export interface ChaseCameraConfig {
  distance?: number;       // Distance behind the robot
  height?: number;         // Height above ground
  lookAheadDistance?: number; // Look-at point ahead of robot
  followSpeed?: number;    // Position lerp speed (units/sec)
  rotationSpeed?: number;  // Rotation lerp speed (units/sec)
}

interface CurrentPose {
  x: number;
  y: number;
  theta: number;
}

const DEFAULT_CONFIG: Required<ChaseCameraConfig> = {
  distance: 3,
  height: 2,
  lookAheadDistance: 1.0,
  followSpeed: 4,
  rotationSpeed: 3,
};

export function useChaseCamera(
  currentPose: React.RefObject<CurrentPose>,
  config?: ChaseCameraConfig
) {
  const { camera } = useThree();
  const cfg = { ...DEFAULT_CONFIG, ...config };

  // Track the camera heading separately so it smoothly rotates
  // instead of snapping to the robot's heading
  const cameraHeading = useRef(0);
  const isInitialized = useRef(false);

  // Reusable vectors to avoid GC in the render loop
  const _desiredPos = useMemo(() => new THREE.Vector3(), []);
  const _lookAt = useMemo(() => new THREE.Vector3(), []);
  const _currentLookAt = useRef(new THREE.Vector3());

  // Initialize camera behind the robot on first available pose
  useEffect(() => {
    if (!currentPose.current) return;
    const { x, y, theta } = currentPose.current;

    cameraHeading.current = theta;

    // Position camera behind robot
    const camX = x - Math.cos(theta) * cfg.distance;
    const camZ = -y + Math.sin(theta) * cfg.distance;
    camera.position.set(camX, cfg.height, camZ);

    // Look at robot
    const lookX = x + Math.cos(theta) * cfg.lookAheadDistance;
    const lookZ = -y - Math.sin(theta) * cfg.lookAheadDistance;
    _currentLookAt.current.set(lookX, 0.4, lookZ);
    camera.lookAt(_currentLookAt.current);

    isInitialized.current = true;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useFrame((_state, delta) => {
    if (!currentPose.current || !isInitialized.current) return;
    const dt = Math.min(delta, 0.05);
    const { x, y, theta } = currentPose.current;

    // ─── 1. Smoothly rotate the camera heading to match robot heading ───
    // This prevents the camera from "snapping" on sharp turns
    let headingDiff = theta - cameraHeading.current;
    while (headingDiff > Math.PI) headingDiff -= 2 * Math.PI;
    while (headingDiff < -Math.PI) headingDiff += 2 * Math.PI;
    const rotAlpha = 1 - Math.exp(-cfg.rotationSpeed * dt);
    cameraHeading.current += headingDiff * rotAlpha;

    // ─── 2. Compute desired camera position (BEHIND the robot) ───
    // In Three.js: robot faces along its local +X (after rotation.y = theta)
    // "Behind" = opposite of forward direction
    // ROS forward = (cos(theta), sin(theta)) → Three.js = (cos(theta), 0, -sin(theta))
    const behindX = x - Math.cos(cameraHeading.current) * cfg.distance;
    const behindZ = -y + Math.sin(cameraHeading.current) * cfg.distance;
    _desiredPos.set(behindX, cfg.height, behindZ);

    // ─── 3. Smoothly lerp camera position ───
    const posAlpha = 1 - Math.exp(-cfg.followSpeed * dt);
    camera.position.lerp(_desiredPos, posAlpha);

    // ─── 4. Compute look-at target (slightly AHEAD of the robot) ───
    const aheadX = x + Math.cos(theta) * cfg.lookAheadDistance;
    const aheadZ = -y - Math.sin(theta) * cfg.lookAheadDistance;
    _lookAt.set(aheadX, 0.3, aheadZ);

    // Smoothly lerp the look-at target
    _currentLookAt.current.lerp(_lookAt, posAlpha);

    // ─── 5. Apply look-at ───
    camera.lookAt(_currentLookAt.current);
  });

  return { cameraHeading };
}
