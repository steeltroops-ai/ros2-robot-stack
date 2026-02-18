"use client";

import { Canvas, useFrame } from "@react-three/fiber";
import {
  OrbitControls,
  Grid,
  PerspectiveCamera,
} from "@react-three/drei";
import { useRef, useMemo, useEffect, useState } from "react";
import * as THREE from "three";

interface RobotViewerProps {
  robotId: string;
  pose: { x: number; y: number; theta: number };
}

import { CyberAmr } from "./assets/CyberAmr";
import { LidarScanner } from "./LidarScanner";
import { DestinationMarker } from "./assets/DestinationMarker";
import { PathLine } from "./assets/PathLine";
import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";

function SceneContent({ pose, robotId }: { pose: { x: number; y: number; theta: number }, robotId: string }) {
  const robotRef = useRef<THREE.Group>(null!);
  const controlsRef = useRef<any>(null!);
  const { sendNavigationGoal, pathData } = useFleetTelemetry();
  
  // Navigation State
  const [goal, setGoal] = useState<{ x: number; y: number } | null>(null);

  // Refs for smooth interpolation
  // Navigation Goal Handler
  const handleFloorClick = (e: any) => {
    // Only handle if it's the right click or some modifier? 
    // Let's just do standard click for now.
    e.stopPropagation();
    const { x, z } = e.point;
    
    // Convert Three.js (x, z) to ROS 2 (x, y)
    // Three.z = -ROS.y => ROS.y = -Three.z
    const rosX = x;
    const rosY = -z;
    
    console.log(`[Navigation] Goal Set: ${rosX.toFixed(2)}, ${rosY.toFixed(2)}`);
    setGoal({ x: rosX, y: rosY });
    sendNavigationGoal(robotId, rosX, rosY);
  };

  const targetPose = useRef(pose);
  const currentPose = useRef({ ...pose });
  const lastPose = useRef({ ...pose }); 

  // Visual state without triggering re-renders (fixes stuttering)
  const robotStateRef = useRef({
    linear: 0,
    angular: 0,
    vy: 0,
    status: "idle"
  });

  // Smoothing for velocity calculation to prevent wheel flicker
  const smoothVel = useRef({ x: 0, y: 0, w: 0 });

  useEffect(() => {
    targetPose.current = pose;
  }, [pose]);

  useFrame((state, delta) => {
    if (!robotRef.current || !controlsRef.current) return;

    // 1. Smoothly interpolate robot position
    // Alpha 0.15 @ 60fps = ~90% convergence in 0.25s
    const alpha = 0.15; 
    const dx = (targetPose.current.x - currentPose.current.x) * alpha;
    const dy = (targetPose.current.y - currentPose.current.y) * alpha;
    
    currentPose.current.x += dx;
    currentPose.current.y += dy;

    // 2. Interpolate Rotation (Standardized ROS Z CCW -> Three Y CCW)
    let diff = targetPose.current.theta - currentPose.current.theta;
    while (diff > Math.PI) diff -= 2 * Math.PI;
    while (diff < -Math.PI) diff += 2 * Math.PI;
    const dTheta = diff * alpha;
    currentPose.current.theta += dTheta;

    // 3. Update Robot Mesh
    robotRef.current.position.set(currentPose.current.x, 0, -currentPose.current.y);
    robotRef.current.rotation.y = currentPose.current.theta;

    // 4. Calculate Visual Velocity for animations
    const dt = delta || 0.016;
    const globalMove = new THREE.Vector3(dx, 0, -dy);
    const localMove = globalMove.clone().applyQuaternion(robotRef.current.quaternion.clone().invert());
    
    // Filtered velocity to avoid 'jittery' wheels
    const filter = 0.2;
    smoothVel.current.x += (localMove.x / dt - smoothVel.current.x) * filter;
    smoothVel.current.y += (-localMove.z / dt - smoothVel.current.y) * filter;
    smoothVel.current.w += (dTheta / dt - smoothVel.current.w) * filter;
    
    const isMoving = Math.abs(smoothVel.current.x) > 0.02 || Math.abs(smoothVel.current.y) > 0.02 || Math.abs(smoothVel.current.w) > 0.02;
    robotStateRef.current = {
        linear: smoothVel.current.x,
        angular: smoothVel.current.w,
        vy: smoothVel.current.y,
        status: isMoving ? "moving" : "idle"
    };

    // 5. Perfectly Synchronized Follow Camera
    // Update target of OrbitControls to match robot mesh EXACTLY
    controlsRef.current.target.set(currentPose.current.x, 0.4, -currentPose.current.y);
    
    // Slide camera position by the same global delta to maintain locked perspective
    // This prevents the 'camera lag' or 'drunk cam' feeling
    state.camera.position.x += dx;
    state.camera.position.z -= dy; 
    
    controlsRef.current.update();
  });

  // Initialize positions precisely on mount
  useEffect(() => {
    // Force sync
    currentPose.current = { ...pose };
    targetPose.current = { ...pose };
    lastPose.current = { ...pose };

    if (controlsRef.current) {
        controlsRef.current.target.set(pose.x, 0.4, -pose.y);
        controlsRef.current.update();
    }
  }, []); 
  
  return (
    <>
      <PerspectiveCamera makeDefault position={[pose.x - 2, 1.5, -pose.y]} fov={50} />
      <OrbitControls
        ref={controlsRef}
        enableDamping={true}
        dampingFactor={0.15}
        minDistance={0.5}
        maxDistance={10}
        maxPolarAngle={Math.PI / 2.1}
      />

      <ambientLight intensity={0.8} />
      <directionalLight position={[5, 8, 5]} intensity={1.5} castShadow shadow-mapSize-width={1024} shadow-mapSize-height={1024} />
      <directionalLight position={[-3, 4, -3]} intensity={0.5} />
      
      {/* Background Plane */}
      <mesh 
        rotation={[-Math.PI / 2, 0, 0]} 
        position={[0, -0.005, 0]} 
        receiveShadow 
        onPointerDown={handleFloorClick}
      >
        <planeGeometry args={[50, 50]} />
        <meshStandardMaterial color="#e4e4e7" roughness={0.8} metalness={0.2} /> 
      </mesh>

      <Grid 
        args={[50, 50]} 
        position={[0, 0, 0]} 
        cellSize={0.5} 
        cellThickness={1} 
        cellColor="#cbd5e1" 
        sectionSize={2.5} 
        sectionThickness={1.5} 
        sectionColor="#9ca3af" 
        fadeDistance={25} 
        infiniteGrid 
      />

      {/* Navigation Path visualization */}
      {pathData && pathData.length > 0 && <PathLine path={pathData} />}

      {/* Navigation Destination Marker */}
      {goal && <DestinationMarker position={[goal.x, 0.02, -goal.y]} />}

      <group ref={robotRef}>
         <CyberAmr stateRef={robotStateRef} />
         <LidarScanner robotId={robotId} />
       </group>
    </>
  );
}

export default function RobotViewer({ robotId, pose }: RobotViewerProps) {
  return (
    <Canvas
      shadows
      gl={{ antialias: true, toneMapping: THREE.ACESFilmicToneMapping, toneMappingExposure: 1.2 }}
      style={{ width: "100%", height: "100%", background: "#000000" }}
    >
      <SceneContent pose={pose} robotId={robotId} />
    </Canvas>
  );
}
