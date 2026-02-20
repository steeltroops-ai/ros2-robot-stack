"use client";

import { Canvas } from "@react-three/fiber";
import { Grid, PerspectiveCamera } from "@react-three/drei";
import { useRef, useState } from "react";
import * as THREE from "three";

import { ThreeEvent } from "@react-three/fiber";
import { CyberAmr } from "./assets/CyberAmr";
import { LidarScanner } from "./LidarScanner";
import { DestinationMarker } from "./assets/DestinationMarker";
import { PathLine } from "./assets/PathLine";
import { useRobotPose } from "./hooks/useRobotPose";
import { useChaseCamera } from "./hooks/useChaseCamera";

// ─── Types ───────────────────────────────────────────────────────
interface RobotViewerProps {
  robotId: string;
  pose: { x: number; y: number; theta: number };
  onNavigationGoal?: (x: number, y: number) => void;
  pathData?: { x: number; y: number }[];
}

// ─── Robot Entity (Mesh + Pose Interpolation) ────────────────────
function RobotEntity({
  pose,
  robotId,
}: {
  pose: { x: number; y: number; theta: number };
  robotId: string;
}) {
  const robotRef = useRef<THREE.Group>(null!);

  const { currentPose, robotStateRef } = useRobotPose(robotRef, pose);

  // Chase camera: sits behind the robot, follows heading like a racing game
  useChaseCamera(currentPose, {
    distance: 3.5,
    height: 2.2,
    lookAheadDistance: 1.5,
    followSpeed: 5,
    rotationSpeed: 3,
  });

  return (
    <group ref={robotRef}>
      <CyberAmr stateRef={robotStateRef} />
      <LidarScanner robotId={robotId} />
    </group>
  );
}

// ─── Navigation Floor (Click-to-Navigate) ────────────────────────
function NavigationFloor({
  onGoalSet,
}: {
  onGoalSet: (x: number, y: number) => void;
}) {
  const handleFloorClick = (e: ThreeEvent<MouseEvent>) => {
    e.stopPropagation();
    const { x, z } = e.point;

    // Three.js Z → ROS Y (inverted)
    const rosX = x;
    const rosY = -z;

    onGoalSet(rosX, rosY);
  };

  return (
    <mesh
      rotation={[-Math.PI / 2, 0, 0]}
      position={[0, -0.01, 0]}
      receiveShadow
      onPointerDown={handleFloorClick}
    >
      <planeGeometry args={[30, 30]} />
      <meshStandardMaterial color={"#2c3e50"} roughness={0.55} metalness={0.2} />
    </mesh>
  );
}

// ─── Lighting Rig ────────────────────────────────────────────────
function SceneLighting() {
  return (
    <>
      <ambientLight intensity={0.8} color={"#c8e0ff"} />
      <directionalLight
        position={[4, 10, 4]}
        intensity={3.0}
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
        shadow-bias={-0.0005}
        color={"#ffffff"}
      />
      <pointLight position={[0, 4, 0]} intensity={4} color={"#10b981"} distance={18} decay={2} />
      <pointLight position={[6, 2, -4]} intensity={2} color={"#10b981"} distance={14} decay={2} />
      <pointLight position={[-6, 2, 4]} intensity={1.5} color={"#6090ff"} distance={14} decay={2} />
    </>
  );
}

// ─── Environment Grid ────────────────────────────────────────────
function EnvironmentGrid() {
  return (
    <Grid
      args={[30, 30]}
      position={[0, 0.001, 0]}
      cellSize={0.5}
      cellThickness={0.5}
      cellColor={"#10b981"}
      sectionSize={2.5}
      sectionThickness={1.0}
      sectionColor={"#10b981"}
      fadeDistance={20}
      fadeStrength={1.5}
      infiniteGrid={false}
    />
  );
}

// ─── Scene Composition ───────────────────────────────────────────
// Pure composition — no logic, no hooks, just wiring components together
function SceneContent({
  pose,
  robotId,
  onNavigationGoal,
  pathData,
}: {
  pose: { x: number; y: number; theta: number };
  robotId: string;
  onNavigationGoal?: (x: number, y: number) => void;
  pathData?: { x: number; y: number }[];
}) {
  const [goal, setGoal] = useState<{ x: number; y: number } | null>(null);

  const handleGoalSet = (x: number, y: number) => {
    setGoal({ x, y });
    onNavigationGoal?.(x, y);
  };

  return (
    <>
      <PerspectiveCamera makeDefault fov={50} />
      <SceneLighting />
      <NavigationFloor onGoalSet={handleGoalSet} />
      <EnvironmentGrid />

      {/* Navigation visualization */}
      {pathData && pathData.length > 0 && <PathLine path={pathData} />}
      {goal && <DestinationMarker position={[goal.x, 0.02, -goal.y]} />}

      {/* The robot entity: mesh + pose interpolation + chase camera */}
      <RobotEntity pose={pose} robotId={robotId} />
    </>
  );
}

// ─── Public Component ────────────────────────────────────────────
export default function RobotViewer({
  robotId,
  pose,
  onNavigationGoal,
  pathData,
}: RobotViewerProps) {
  return (
    <Canvas
      shadows
      gl={{
        antialias: true,
        toneMapping: THREE.ACESFilmicToneMapping,
        toneMappingExposure: 1.4,
      }}
      style={{ width: "100%", height: "100%" }}
      onCreated={({ gl, scene }) => {
        gl.setClearColor(new THREE.Color("#0d1520"));
        scene.fog = new THREE.FogExp2("#0d1520", 0.025);
      }}
    >
      <SceneContent
        pose={pose}
        robotId={robotId}
        onNavigationGoal={onNavigationGoal}
        pathData={pathData}
      />
    </Canvas>
  );
}
