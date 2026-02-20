"use client";

import { useFrame } from "@react-three/fiber";
import { useMemo, useRef } from "react";
import * as THREE from "three";

// ═══════════════════════════════════════════════════════════════════
// BionicHand — 24-DOF Humanoid Robotic Arm (Procedural Geometry)
// ═══════════════════════════════════════════════════════════════════
// Architecture follows CyberAmr.tsx patterns:
// - All materials created via useMemo (no re-creation per frame)
// - Animation via useFrame with stateRef (no React re-renders)
// - Hierarchical group nesting for kinematic chain
// ═══════════════════════════════════════════════════════════════════

// ── Color Palette (Research-Lab Industrial) ────────────────────
const THEME = {
  shell: "#2c2f35",       // Arm panels — dark titanium grey
  innerMech: "#3a3d44",   // Exposed mechanisms — charcoal alloy
  actuator: "#6b7280",    // Joint housings — brushed steel grey
  accent: "#8a9ab0",      // Neutral steel-blue tendon highlights
  brand: "#c8943a",       // Muted amber — lab status LED
  tendon: "#7a8490",      // Cable channels — cool grey
  fingerPad: "#4a5060",   // Silicone fingertip pads — dark slate
  danger: "#b34040",      // Error state — muted red
  dark: "#1a1c21",        // Deepest chassis black
};

// ── State Interface ──────────────────────────────────────────────
export interface BionicHandState {
  shoulderYaw: number;
  shoulderPitch: number;
  shoulderRoll: number;
  elbowPitch: number;
  forearmRoll: number;
  wristPitch: number;
  wristYaw: number;
  thumb: [number, number, number];    // CMC, MCP, IP (0=open, 1=closed)
  index: [number, number, number];    // MCP, PIP, DIP
  middle: [number, number, number];
  ring: [number, number, number];
  pinky: [number, number, number];
  status: "idle" | "grasping" | "releasing" | "calibrating" | "error";
  gripForce: number;
}

interface BionicHandProps {
  stateRef?: React.MutableRefObject<BionicHandState>;
}

// ── Helper: Smooth sinusoidal easing ─────────────────────────────
function smoothStep(t: number): number {
  return t * t * (3 - 2 * t);
}

// ── Idle Animation Presets ───────────────────────────────────────
function getIdleState(time: number): BionicHandState {
  const cycle = time % 14; // 14-second loop

  const base: BionicHandState = {
    // Natural hanging arm: gentle sway, humanlike resting pose
    shoulderYaw: Math.sin(time * 0.2) * 0.05,
    shoulderPitch: 0.12 + Math.sin(time * 0.15) * 0.06,  // slightly forward at rest
    shoulderRoll: Math.sin(time * 0.18) * 0.04,
    elbowPitch: 0.30 + Math.sin(time * 0.25) * 0.06,     // ~17° natural bend
    forearmRoll: Math.sin(time * 0.22) * 0.10,
    wristPitch: 0.08 + Math.sin(time * 0.30) * 0.12,     // slight extension
    wristYaw: Math.sin(time * 0.28) * 0.08,
    // Natural resting finger curl (~15° at MCP)
    thumb: [0.15, 0.10, 0.05],
    index: [0.15, 0.10, 0.05],
    middle: [0.15, 0.10, 0.05],
    ring: [0.18, 0.12, 0.06],
    pinky: [0.20, 0.14, 0.07],
    status: "idle",
    gripForce: 0,
  };

  if (cycle < 3) {
    // Phase 1: Open hand, slight wave
    const t = cycle / 3;
    base.wristYaw = Math.sin(t * Math.PI * 4) * 0.3;
  } else if (cycle < 5.5) {
    // Phase 2: Sequential finger curl
    const t = (cycle - 3) / 2.5;
    const indexT = smoothStep(Math.min(1, Math.max(0, t * 4)));
    const middleT = smoothStep(Math.min(1, Math.max(0, (t - 0.15) * 4)));
    const ringT = smoothStep(Math.min(1, Math.max(0, (t - 0.3) * 4)));
    const pinkyT = smoothStep(Math.min(1, Math.max(0, (t - 0.45) * 4)));
    const thumbT = smoothStep(Math.min(1, Math.max(0, (t - 0.6) * 4)));

    base.index = [indexT, indexT * 0.9, indexT * 0.7];
    base.middle = [middleT, middleT * 0.9, middleT * 0.7];
    base.ring = [ringT, ringT * 0.9, ringT * 0.7];
    base.pinky = [pinkyT, pinkyT * 0.9, pinkyT * 0.7];
    base.thumb = [thumbT * 0.7, thumbT * 0.8, thumbT * 0.6];
  } else if (cycle < 7.5) {
    // Phase 3: Full fist
    base.index = [1, 0.95, 0.8];
    base.middle = [1, 0.95, 0.8];
    base.ring = [1, 0.95, 0.8];
    base.pinky = [1, 0.95, 0.8];
    base.thumb = [0.7, 0.8, 0.6];
    base.gripForce = 0.8;
    base.status = "grasping";
  } else if (cycle < 9.5) {
    // Phase 4: Unclench with spread
    const t = (cycle - 7.5) / 2;
    const release = 1 - smoothStep(t);
    base.index = [release, release * 0.9, release * 0.7];
    base.middle = [release, release * 0.9, release * 0.7];
    base.ring = [release, release * 0.9, release * 0.7];
    base.pinky = [release, release * 0.9, release * 0.7];
    base.thumb = [release * 0.7, release * 0.8, release * 0.6];
    base.forearmRoll = t * 0.5;
    base.status = "releasing";
  } else if (cycle < 11.5) {
    // Phase 5: Piano wiggle
    const t = (cycle - 9.5) / 2;
    const wave = (finger: number) => Math.sin((t * 8 + finger * 0.8)) * 0.5 + 0.5;
    base.index = [wave(0) * 0.6, wave(0) * 0.5, wave(0) * 0.3];
    base.middle = [wave(1) * 0.6, wave(1) * 0.5, wave(1) * 0.3];
    base.ring = [wave(2) * 0.6, wave(2) * 0.5, wave(2) * 0.3];
    base.pinky = [wave(3) * 0.6, wave(3) * 0.5, wave(3) * 0.3];
    base.wristPitch = Math.sin(t * 3) * 0.15;
  } else {
    // Phase 6: Precision pinch (thumb + index)
    const t = (cycle - 11.5) / 2.5;
    const pinch = smoothStep(Math.min(1, t * 2));
    const release = t > 0.5 ? smoothStep((t - 0.5) * 2) : 0;
    const amount = pinch - release;
    base.thumb = [amount * 0.9, amount * 0.7, amount * 0.3];
    base.index = [amount * 0.8, amount * 0.6, amount * 0.2];
    base.gripForce = amount * 0.4;
  }

  return base;
}

// ═══════════════════════════════════════════════════════════════════
// Sub-component: Single Finger
// ═══════════════════════════════════════════════════════════════════
function Finger({
  lengths,
  widths,
  refs,
  position,
  splayAngle = 0,
  shellMat,
  jointMat,
  padMat,
  glowMat,
}: {
  lengths: [number, number, number];
  widths: [number, number, number];
  refs: [
    React.RefObject<THREE.Group>,
    React.RefObject<THREE.Group>,
    React.RefObject<THREE.Group>,
  ];
  position: [number, number, number];
  splayAngle?: number;
  shellMat: THREE.Material;
  jointMat: THREE.Material;
  padMat: THREE.Material;
  glowMat: THREE.Material;
}) {
  return (
    <group position={position} rotation={[0, splayAngle, 0]}>
      {/* MCP Joint */}
      <mesh material={jointMat}>
        <sphereGeometry args={[widths[0] * 0.55, 12, 8]} />
      </mesh>

      {/* Proximal Phalanx */}
      <group ref={refs[0]}>
        <mesh position={[lengths[0] / 2, 0, 0]} castShadow material={shellMat}>
          <boxGeometry args={[lengths[0], widths[0], widths[0] * 0.9]} />
        </mesh>
        {/* Tendon line */}
        <mesh position={[lengths[0] / 2, widths[0] * 0.5, 0]} material={glowMat}>
          <boxGeometry args={[lengths[0] * 0.8, 0.001, 0.002]} />
        </mesh>

        {/* PIP Joint */}
        <group position={[lengths[0], 0, 0]}>
          <mesh material={jointMat}>
            <sphereGeometry args={[widths[1] * 0.5, 12, 8]} />
          </mesh>

          {/* Middle Phalanx */}
          <group ref={refs[1]}>
            <mesh position={[lengths[1] / 2, 0, 0]} castShadow material={shellMat}>
              <boxGeometry args={[lengths[1], widths[1], widths[1] * 0.85]} />
            </mesh>

            {/* DIP Joint */}
            <group position={[lengths[1], 0, 0]}>
              <mesh material={jointMat}>
                <sphereGeometry args={[widths[2] * 0.45, 12, 8]} />
              </mesh>

              {/* Distal Phalanx */}
              <group ref={refs[2]}>
                <mesh position={[lengths[2] / 2, 0, 0]} castShadow material={padMat}>
                  <boxGeometry args={[lengths[2], widths[2], widths[2] * 0.8]} />
                </mesh>
                {/* Fingertip sensor pad */}
                <mesh position={[lengths[2], 0, 0]} material={glowMat}>
                  <sphereGeometry args={[widths[2] * 0.35, 8, 8]} />
                </mesh>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}

// ═══════════════════════════════════════════════════════════════════
// Sub-component: Thumb (unique kinematics)
// ═══════════════════════════════════════════════════════════════════
function Thumb({
  refs,
  shellMat,
  jointMat,
  padMat,
  glowMat,
}: {
  refs: [
    React.RefObject<THREE.Group>,
    React.RefObject<THREE.Group>,
    React.RefObject<THREE.Group>,
  ];
  shellMat: THREE.Material;
  jointMat: THREE.Material;
  padMat: THREE.Material;
  glowMat: THREE.Material;
}) {
  return (
    <group position={[0.01, -0.008, 0.04]} rotation={[0, 0.7, -0.3]}>
      {/* CMC Joint — opposition */}
      <mesh material={jointMat}>
        <sphereGeometry args={[0.012, 12, 8]} />
      </mesh>

      <group ref={refs[0]}>
        {/* Metacarpal */}
        <mesh position={[0.018, 0, 0]} castShadow material={shellMat}>
          <boxGeometry args={[0.035, 0.022, 0.020]} />
        </mesh>

        {/* MCP Joint */}
        <group position={[0.035, 0, 0]}>
          <mesh material={jointMat}>
            <sphereGeometry args={[0.011, 12, 8]} />
          </mesh>

          <group ref={refs[1]}>
            {/* Proximal Phalanx */}
            <mesh position={[0.015, 0, 0]} castShadow material={shellMat}>
              <boxGeometry args={[0.03, 0.020, 0.018]} />
            </mesh>

            {/* IP Joint */}
            <group position={[0.03, 0, 0]}>
              <mesh material={jointMat}>
                <sphereGeometry args={[0.010, 12, 8]} />
              </mesh>

              <group ref={refs[2]}>
                {/* Distal */}
                <mesh position={[0.012, 0, 0]} castShadow material={padMat}>
                  <boxGeometry args={[0.025, 0.020, 0.018]} />
                </mesh>
                {/* Tip sensor */}
                <mesh position={[0.025, 0, 0]} material={glowMat}>
                  <sphereGeometry args={[0.008, 8, 8]} />
                </mesh>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}

// ═══════════════════════════════════════════════════════════════════
// Main Component: BionicHand
// ═══════════════════════════════════════════════════════════════════
export function BionicHand({ stateRef }: BionicHandProps) {
  // ── Joint Refs (Kinematic Chain) ───────────────────────────────
  // Arm
  const shoulderYawRef = useRef<THREE.Group>(null!);
  const shoulderPitchRef = useRef<THREE.Group>(null!);
  const shoulderRollRef = useRef<THREE.Group>(null!);
  const elbowPitchRef = useRef<THREE.Group>(null!);
  const forearmRollRef = useRef<THREE.Group>(null!);
  const wristPitchRef = useRef<THREE.Group>(null!);
  const wristYawRef = useRef<THREE.Group>(null!);

  // Fingers (3 joints each: MCP, PIP, DIP)
  const indexRefs = [
    useRef<THREE.Group>(null!),
    useRef<THREE.Group>(null!),
    useRef<THREE.Group>(null!),
  ] as [React.RefObject<THREE.Group>, React.RefObject<THREE.Group>, React.RefObject<THREE.Group>];

  const middleRefs = [
    useRef<THREE.Group>(null!),
    useRef<THREE.Group>(null!),
    useRef<THREE.Group>(null!),
  ] as [React.RefObject<THREE.Group>, React.RefObject<THREE.Group>, React.RefObject<THREE.Group>];

  const ringRefs = [
    useRef<THREE.Group>(null!),
    useRef<THREE.Group>(null!),
    useRef<THREE.Group>(null!),
  ] as [React.RefObject<THREE.Group>, React.RefObject<THREE.Group>, React.RefObject<THREE.Group>];

  const pinkyRefs = [
    useRef<THREE.Group>(null!),
    useRef<THREE.Group>(null!),
    useRef<THREE.Group>(null!),
  ] as [React.RefObject<THREE.Group>, React.RefObject<THREE.Group>, React.RefObject<THREE.Group>];

  const thumbRefs = [
    useRef<THREE.Group>(null!),
    useRef<THREE.Group>(null!),
    useRef<THREE.Group>(null!),
  ] as [React.RefObject<THREE.Group>, React.RefObject<THREE.Group>, React.RefObject<THREE.Group>];

  // Status ring
  const statusRingRef = useRef<THREE.Mesh>(null!);

  // ── Materials (memoized, never re-created) ─────────────────────
  const shellMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: THEME.shell,
        roughness: 0.7,
        metalness: 0.2,
      }),
    []
  );

  const innerMechMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: THEME.innerMech,
        roughness: 0.3,
        metalness: 0.7,
        emissive: THEME.innerMech,
        emissiveIntensity: 0.05,
      }),
    []
  );

  const actuatorMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: THEME.actuator,
        roughness: 0.25,
        metalness: 0.9,
      }),
    []
  );

  const accentMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: THEME.accent,
        roughness: 0.35,
        metalness: 0.75,
        // No emissive — lab-grade neutral steel highlight
      }),
    []
  );

  const brandGlowMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: THEME.brand,
        emissive: THEME.brand,
        emissiveIntensity: 0.6,  // Subtle amber glow — lab status LED only
        toneMapped: false,
      }),
    []
  );

  const tendonMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: THEME.tendon,
        roughness: 0.4,
        metalness: 0.6,
      }),
    []
  );

  const fingerPadMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: THEME.fingerPad,
        roughness: 0.85,
        metalness: 0.05,
      }),
    []
  );

  const statusMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: THEME.brand,
        emissive: THEME.brand,
        emissiveIntensity: 0.8,  // Calm amber pulse — not neon
        toneMapped: false,
      }),
    []
  );

  // ── Current interpolated state ─────────────────────────────────
  const currentState = useRef<BionicHandState>(getIdleState(0));

  // ── Animation Loop ─────────────────────────────────────────────
  useFrame((threeState, delta) => {
    const time = threeState.clock.getElapsedTime();

    // Get target state: external or idle animation
    const target = stateRef?.current ?? getIdleState(time);

    // Smooth interpolation toward target
    const lerp = (a: number, b: number, t: number) => a + (b - a) * t;
    const alpha = 0.08; // Smooth factor

    const cs = currentState.current;
    cs.shoulderYaw = lerp(cs.shoulderYaw, target.shoulderYaw, alpha);
    cs.shoulderPitch = lerp(cs.shoulderPitch, target.shoulderPitch, alpha);
    cs.shoulderRoll = lerp(cs.shoulderRoll, target.shoulderRoll, alpha);
    cs.elbowPitch = lerp(cs.elbowPitch, target.elbowPitch, alpha);
    cs.forearmRoll = lerp(cs.forearmRoll, target.forearmRoll, alpha);
    cs.wristPitch = lerp(cs.wristPitch, target.wristPitch, alpha);
    cs.wristYaw = lerp(cs.wristYaw, target.wristYaw, alpha);
    cs.gripForce = lerp(cs.gripForce, target.gripForce, alpha);

    for (let i = 0; i < 3; i++) {
      cs.thumb[i] = lerp(cs.thumb[i], target.thumb[i], alpha);
      cs.index[i] = lerp(cs.index[i], target.index[i], alpha);
      cs.middle[i] = lerp(cs.middle[i], target.middle[i], alpha);
      cs.ring[i] = lerp(cs.ring[i], target.ring[i], alpha);
      cs.pinky[i] = lerp(cs.pinky[i], target.pinky[i], alpha);
    }

    // ── Apply to joint hierarchy ─────────────────────────────────
    // Max flexion angle for finger joints (in radians)
    const MAX_FLEX = Math.PI / 2; // 90°

    // Arm joints
    if (shoulderYawRef.current) shoulderYawRef.current.rotation.y = cs.shoulderYaw;
    if (shoulderPitchRef.current) shoulderPitchRef.current.rotation.z = cs.shoulderPitch;
    if (shoulderRollRef.current) shoulderRollRef.current.rotation.x = cs.shoulderRoll;
    if (elbowPitchRef.current) elbowPitchRef.current.rotation.z = -cs.elbowPitch;
    if (forearmRollRef.current) forearmRollRef.current.rotation.x = cs.forearmRoll;
    if (wristPitchRef.current) wristPitchRef.current.rotation.z = cs.wristPitch;
    if (wristYawRef.current) wristYawRef.current.rotation.y = cs.wristYaw;

    // Finger application (flexion = negative Z rotation for curl)
    const applyFinger = (
      refs: [React.RefObject<THREE.Group>, React.RefObject<THREE.Group>, React.RefObject<THREE.Group>],
      values: [number, number, number]
    ) => {
      if (refs[0].current) refs[0].current.rotation.z = -values[0] * MAX_FLEX;
      if (refs[1].current) refs[1].current.rotation.z = -values[1] * MAX_FLEX;
      if (refs[2].current) refs[2].current.rotation.z = -values[2] * MAX_FLEX;
    };

    applyFinger(indexRefs, cs.index);
    applyFinger(middleRefs, cs.middle);
    applyFinger(ringRefs, cs.ring);
    applyFinger(pinkyRefs, cs.pinky);
    applyFinger(thumbRefs, cs.thumb);

    // ── Status ring gentle pulse (muted amber) ──────────────────
    statusMat.emissiveIntensity = 0.5 + Math.sin(time * 2) * 0.2;

    const statusColor = new THREE.Color(
      target.status === "grasping" ? "#b08840" :  // deeper amber on grasp
      target.status === "error"    ? THEME.danger :
      target.status === "calibrating" ? "#a07830" :
      THEME.brand
    );
    statusMat.color.lerp(statusColor, 0.1);
    statusMat.emissive.lerp(statusColor, 0.1);
    // accentMat has no emissive in lab palette — nothing to update
  });

  // ═══════════════════════════════════════════════════════════════
  // JSX — Kinematic chain (nested groups for FK)
  // ═══════════════════════════════════════════════════════════════
  return (
    <group dispose={null}>
      {/* ═══ SHOULDER INTERFACE ═══ */}
      {/* Shoulder flange — faces along +X (the arm's forward direction)
          When the arm group is rotated -90°Z in the viewer, this faces
          outward from the torso socket correctly — no clipping. */}
      {/* Outer flange ring */}
      <mesh rotation={[0, 0, Math.PI / 2]} material={actuatorMat}>
        <cylinderGeometry args={[0.048, 0.050, 0.012, 24]} />
      </mesh>
      {/* Status ring — faces +X (outward from shoulder) */}
      <mesh rotation={[0, 0, Math.PI / 2]} material={statusMat} ref={statusRingRef}>
        <ringGeometry args={[0.038, 0.046, 32]} />
      </mesh>
      {/* Mount bolts around flange */}
      {[0, 1, 2, 3, 4, 5].map((i) => (
        <mesh
          key={`bolt-${i}`}
          position={[
            0,
            Math.cos((i * Math.PI * 2) / 6) * 0.042,
            Math.sin((i * Math.PI * 2) / 6) * 0.042,
          ]}
          rotation={[0, 0, Math.PI / 2]}
          material={actuatorMat}
        >
          <cylinderGeometry args={[0.003, 0.003, 0.014, 6]} />
        </mesh>
      ))}

      {/* ═══ SHOULDER YAW (J1: Y-axis) ═══ */}
      <group ref={shoulderYawRef}>
        {/* Shoulder actuator housing */}
        <mesh position={[0, 0.015, 0]} castShadow material={actuatorMat}>
          <cylinderGeometry args={[0.045, 0.045, 0.04, 20]} />
        </mesh>
        {/* Accent ring */}
        <mesh position={[0, 0.036, 0]} rotation={[-Math.PI / 2, 0, 0]} material={accentMat}>
          <ringGeometry args={[0.038, 0.042, 24]} />
        </mesh>

        {/* ═══ SHOULDER PITCH (J2: Z-axis) ═══ */}
        <group ref={shoulderPitchRef} position={[0, 0.04, 0]}>
          {/* Pitch actuator */}
          <mesh castShadow material={actuatorMat}>
            <sphereGeometry args={[0.035, 16, 12]} />
          </mesh>

          {/* ═══ SHOULDER ROLL (J3: X-axis) ═══ */}
          <group ref={shoulderRollRef}>

            {/* ════════════════════════════════════════════ */}
            {/* ═══ UPPER ARM ═══ */}
            {/* ════════════════════════════════════════════ */}
            <group>
              {/* Main shell — armored plate */}
              <mesh position={[0.14, 0, 0]} castShadow receiveShadow material={shellMat}>
                <boxGeometry args={[0.26, 0.06, 0.055]} />
              </mesh>

              {/* Inner mechanism strip (exposed gap) */}
              <mesh position={[0.14, 0.031, 0]} material={innerMechMat}>
                <boxGeometry args={[0.22, 0.003, 0.03]} />
              </mesh>

              {/* Accent LED strip */}
              <mesh position={[0.14, 0, 0.029]} material={accentMat}>
                <boxGeometry args={[0.20, 0.004, 0.001]} />
              </mesh>
              <mesh position={[0.14, 0, -0.029]} material={accentMat}>
                <boxGeometry args={[0.20, 0.004, 0.001]} />
              </mesh>

              {/* Elbow motor bulge */}
              <mesh position={[0.24, 0, 0]} castShadow material={actuatorMat}>
                <sphereGeometry args={[0.032, 14, 10]} />
              </mesh>

              {/* Panel detail lines */}
              {[0.06, 0.12, 0.20].map((x, i) => (
                <mesh key={`panel-${i}`} position={[x, 0.031, 0]} material={tendonMat}>
                  <boxGeometry args={[0.002, 0.001, 0.04]} />
                </mesh>
              ))}

              {/* ═══ ELBOW PITCH (J4: Z-axis) ═══ */}
              <group ref={elbowPitchRef} position={[0.27, 0, 0]}>
                {/* Elbow joint housing */}
                <mesh rotation={[Math.PI / 2, 0, 0]} castShadow material={actuatorMat}>
                  <cylinderGeometry args={[0.028, 0.028, 0.04, 16]} />
                </mesh>
                {/* Hinge pin */}
                <mesh rotation={[Math.PI / 2, 0, 0]} material={tendonMat}>
                  <cylinderGeometry args={[0.008, 0.008, 0.05, 8]} />
                </mesh>
                {/* Elbow accent ring */}
                <mesh rotation={[Math.PI / 2, 0, 0]} material={accentMat} position={[0, 0.021, 0]}>
                  <ringGeometry args={[0.022, 0.026, 20]} />
                </mesh>

                {/* ═══ FOREARM ROLL (J5: X-axis) ═══ */}
                <group ref={forearmRollRef}>

                  {/* ════════════════════════════════════════════ */}
                  {/* ═══ FOREARM ═══ */}
                  {/* ════════════════════════════════════════════ */}
                  <group>
                    {/* Dual-rail forearm shells */}
                    <mesh position={[0.12, 0.012, 0]} castShadow receiveShadow material={shellMat}>
                      <boxGeometry args={[0.22, 0.03, 0.048]} />
                    </mesh>
                    <mesh position={[0.12, -0.012, 0]} castShadow receiveShadow material={shellMat}>
                      <boxGeometry args={[0.22, 0.03, 0.048]} />
                    </mesh>

                    {/* Cross-members */}
                    {[0.04, 0.10, 0.16, 0.22].map((x, i) => (
                      <mesh key={`xbar-${i}`} position={[x, 0, 0]} material={innerMechMat}>
                        <boxGeometry args={[0.008, 0.02, 0.04]} />
                      </mesh>
                    ))}

                    {/* Tendon channels (5 cables) */}
                    {[-0.016, -0.008, 0, 0.008, 0.016].map((z, i) => (
                      <mesh key={`tendon-${i}`} position={[0.12, 0.028, z]} material={tendonMat}>
                        <boxGeometry args={[0.18, 0.003, 0.003]} />
                      </mesh>
                    ))}

                    {/* Forearm accent strip */}
                    <mesh position={[0.12, 0.028, 0.025]} material={accentMat}>
                      <boxGeometry args={[0.16, 0.003, 0.001]} />
                    </mesh>

                    {/* ═══ WRIST PITCH (J6: Z-axis) ═══ */}
                    <group ref={wristPitchRef} position={[0.235, 0, 0]}>
                      {/* Wrist gimbal outer ring */}
                      <mesh rotation={[0, 0, Math.PI / 2]} material={actuatorMat}>
                        <torusGeometry args={[0.022, 0.005, 8, 20]} />
                      </mesh>

                      {/* Force/torque sensor ring */}
                      <mesh rotation={[0, 0, Math.PI / 2]} material={brandGlowMat}>
                        <torusGeometry args={[0.018, 0.002, 8, 20]} />
                      </mesh>

                      {/* ═══ WRIST YAW (J7: Y-axis) ═══ */}
                      <group ref={wristYawRef}>
                        {/* Inner gimbal ring */}
                        <mesh rotation={[Math.PI / 2, 0, 0]} material={actuatorMat}>
                          <torusGeometry args={[0.018, 0.004, 8, 20]} />
                        </mesh>

                        {/* ════════════════════════════════════════ */}
                        {/* ═══ PALM ═══ */}
                        {/* ════════════════════════════════════════ */}
                        <group position={[0.045, 0, 0]}>
                          {/* Palm base plate */}
                          <mesh castShadow receiveShadow material={shellMat}>
                            <boxGeometry args={[0.07, 0.02, 0.08]} />
                          </mesh>

                          {/* Metacarpal arch (curved top) */}
                          <mesh position={[0, 0.011, 0]} material={innerMechMat}>
                            <boxGeometry args={[0.06, 0.004, 0.07]} />
                          </mesh>

                          {/* Palm sensor grid (emissive dots) */}
                          {[-0.02, 0, 0.02].map((x, xi) =>
                            [-0.025, 0, 0.025].map((z, zi) => (
                              <mesh
                                key={`sensor-${xi}-${zi}`}
                                position={[x, -0.011, z]}
                                material={brandGlowMat}
                              >
                                <sphereGeometry args={[0.003, 6, 6]} />
                              </mesh>
                            ))
                          )}

                          {/* Tendon anchor points */}
                          {[-0.03, -0.015, 0, 0.015, 0.03].map((z, i) => (
                            <mesh key={`anchor-${i}`} position={[0.02, 0.012, z]} material={tendonMat}>
                              <cylinderGeometry args={[0.003, 0.003, 0.004, 6]} />
                            </mesh>
                          ))}

                          {/* ════════════════════════════════ */}
                          {/* ═══ FINGERS ═══ */}
                          {/* ════════════════════════════════ */}

                          {/* Index Finger */}
                          <Finger
                            refs={indexRefs}
                            lengths={[0.04, 0.025, 0.02]}
                            widths={[0.018, 0.016, 0.014]}
                            position={[0.035, 0, -0.025]}
                            splayAngle={-0.05}
                            shellMat={shellMat}
                            jointMat={actuatorMat}
                            padMat={fingerPadMat}
                            glowMat={accentMat}
                          />

                          {/* Middle Finger */}
                          <Finger
                            refs={middleRefs}
                            lengths={[0.044, 0.028, 0.02]}
                            widths={[0.018, 0.016, 0.014]}
                            position={[0.038, 0, -0.008]}
                            splayAngle={0}
                            shellMat={shellMat}
                            jointMat={actuatorMat}
                            padMat={fingerPadMat}
                            glowMat={accentMat}
                          />

                          {/* Ring Finger */}
                          <Finger
                            refs={ringRefs}
                            lengths={[0.04, 0.025, 0.018]}
                            widths={[0.017, 0.015, 0.013]}
                            position={[0.035, 0, 0.009]}
                            splayAngle={0.05}
                            shellMat={shellMat}
                            jointMat={actuatorMat}
                            padMat={fingerPadMat}
                            glowMat={accentMat}
                          />

                          {/* Pinky Finger */}
                          <Finger
                            refs={pinkyRefs}
                            lengths={[0.032, 0.02, 0.015]}
                            widths={[0.015, 0.013, 0.011]}
                            position={[0.030, 0, 0.025]}
                            splayAngle={0.12}
                            shellMat={shellMat}
                            jointMat={actuatorMat}
                            padMat={fingerPadMat}
                            glowMat={accentMat}
                          />

                          {/* Thumb */}
                          <Thumb
                            refs={thumbRefs}
                            shellMat={shellMat}
                            jointMat={actuatorMat}
                            padMat={fingerPadMat}
                            glowMat={accentMat}
                          />
                        </group>
                      </group>
                    </group>
                  </group>
                </group>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}
