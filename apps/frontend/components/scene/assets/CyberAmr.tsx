"use client";

import { useFrame } from "@react-three/fiber";
import { useMemo, useRef } from "react";
import * as THREE from "three";

interface CyberAmrProps {
  stateRef: React.MutableRefObject<{
    linear: number;
    angular: number;
    vy: number;
    status: string;
  }>;
}

// Color Palette for Cyber Industrial Theme
const THEME = {
  chassis: "#18181b", // Zinc-900 (Matte Dark Grey)
  accent: "#22d3ee",  // Cyan-400 (Tron Glow)
  danger: "#ef4444",  // Red-500
  success: "#22c55e", // Green-500
  metal: "#52525b",   // Zinc-600
  tire: "#09090b",    // Zinc-950
};

export function CyberAmr({ stateRef }: CyberAmrProps) {
  // --- Refs for Animation ---
  const lidarRef = useRef<THREE.Group>(null!);
  const wheelFL = useRef<THREE.Group>(null!);
  const wheelFR = useRef<THREE.Group>(null!);
  const wheelBL = useRef<THREE.Group>(null!);
  const wheelBR = useRef<THREE.Group>(null!);
  const statusRingRef = useRef<THREE.Mesh>(null!);

  // --- Materials ---
  const chassisMat = useMemo(() => new THREE.MeshStandardMaterial({ 
    color: THEME.chassis, 
    roughness: 0.7, 
    metalness: 0.2 
  }), []);
  
  const techPanelMat = useMemo(() => new THREE.MeshStandardMaterial({
    color: "#27272a",
    roughness: 0.2,
    metalness: 0.8,
    emissive: "#27272a",
    emissiveIntensity: 0.1
  }), []);

  const tireMat = useMemo(() => new THREE.MeshStandardMaterial({ 
    color: THEME.tire, 
    roughness: 0.9, 
    metalness: 0.1 
  }), []);

  const rimMat = useMemo(() => new THREE.MeshStandardMaterial({ 
    color: THEME.metal, 
    roughness: 0.3, 
    metalness: 0.9 
  }), []);

  const lidarGlassMat = useMemo(() => new THREE.MeshPhysicalMaterial({
    color: "#000000",
    metalness: 1,
    roughness: 0,
    transmission: 0.2, // Some glass effect
    transparent: true,
    opacity: 0.9
  }), []);

  // Dynamic Emission Material for Status Ring
  const ringMat = useMemo(() => new THREE.MeshStandardMaterial({
    color: "#ffffff",
    toneMapped: false,
    emissive: "#ffffff",
    emissiveIntensity: 2
  }), []);

  // --- Animation Loop ---
  useFrame((state, delta) => {
    if (!stateRef.current) return;
    const { linear: vx, angular: wz, vy, status: currentStatus } = stateRef.current;
    
    // 1. Spin LiDAR
    if (lidarRef.current) {
      lidarRef.current.rotation.y += 10 * delta;
    }

    // 2. Rotate Wheels based on velocity (Mechanum Kinematics)
    const wheelRadius = 0.05;
    const off = 0.2; // track width / wheelbase offset
    
    // Mechanum wheel speeds for visual effect
    const sFL = (vx - vy - wz * off) / wheelRadius;
    const sFR = (vx + vy + wz * off) / wheelRadius;
    const sBL = (vx + vy - wz * off) / wheelRadius;
    const sBR = (vx - vy + wz * off) / wheelRadius;

    if (wheelFL.current) wheelFL.current.rotation.z -= sFL * delta;
    if (wheelBL.current) wheelBL.current.rotation.z -= sBL * delta;
    if (wheelFR.current) wheelFR.current.rotation.z -= sFR * delta;
    if (wheelBR.current) wheelBR.current.rotation.z -= sBR * delta;

    // 3. Status Pulse
    const time = state.clock.getElapsedTime();
    ringMat.emissiveIntensity = 1.5 + Math.sin(time * 3) * 0.5;
    
    let targetColor = new THREE.Color(THEME.success);
    if (currentStatus === "moving") targetColor.set(THEME.accent);
    if (currentStatus === "error") targetColor.set(THEME.danger);
    if (currentStatus === "offline") targetColor.set("#71717a");
    
    ringMat.color.lerp(targetColor, 0.1);
    ringMat.emissive.lerp(targetColor, 0.1);
  });

  return (
    <group dispose={null}>
      {/* --- Main Chassis Body --- */}
      {/* Lower Base */}
      <mesh position={[0, 0.08, 0]} castShadow receiveShadow material={chassisMat}>
        <boxGeometry args={[0.5, 0.1, 0.35]} />
      </mesh>
      
      {/* Front Headlights (Visual Orientation Cues) */}
      <mesh position={[0.251, 0.08, 0.12]} material={ringMat}>
        <boxGeometry args={[0.01, 0.04, 0.08]} />
      </mesh>
      <mesh position={[0.251, 0.08, -0.12]} material={ringMat}>
        <boxGeometry args={[0.01, 0.04, 0.08]} />
      </mesh>
      
      {/* Upper Tech Deck */}
      <mesh position={[0, 0.16, 0]} castShadow receiveShadow material={techPanelMat}>
        <boxGeometry args={[0.42, 0.06, 0.28]} />
      </mesh>

      {/* Front Bumper / Sensor Array */}
      <mesh position={[0.26, 0.06, 0]} castShadow receiveShadow material={rimMat}>
        <boxGeometry args={[0.04, 0.08, 0.32]} />
      </mesh>

      {/* --- Status Ring (Glowing Core) --- */}
      <mesh ref={statusRingRef} position={[0, 0.191, 0]} rotation={[-Math.PI / 2, 0, 0]} material={ringMat}>
        <ringGeometry args={[0.08, 0.10, 32]} />
      </mesh>

      {/* --- LiDAR Unit --- */}
      <group position={[0.12, 0.19, 0]}>
        {/* Lidar Base */}
        <mesh position={[0, 0.02, 0]} material={rimMat}>
          <cylinderGeometry args={[0.04, 0.04, 0.04, 16]} />
        </mesh>
        {/* Rotating Head */}
        <group ref={lidarRef} position={[0, 0.05, 0]}>
           <mesh material={lidarGlassMat}>
             <cylinderGeometry args={[0.038, 0.038, 0.03, 16]} />
           </mesh>
           {/* Internal Lasers (Visual Detail) */}
           <mesh position={[0, 0, 0.02]} material={rimMat}>
             <boxGeometry args={[0.02, 0.02, 0.05]} />
           </mesh>
        </group>
      </group>

      {/* --- Wheels (Mechanum Style Visuals) --- */}
      {/* Front Left */}
      <group ref={wheelFL} position={[0.15, 0.05, 0.20]}>
        <mesh rotation={[Math.PI / 2, 0, 0]} material={tireMat}>
          <cylinderGeometry args={[0.05, 0.05, 0.04, 32]} />
        </mesh>
        <mesh rotation={[Math.PI / 2, 0, 0]} material={rimMat}>
          <cylinderGeometry args={[0.02, 0.02, 0.042, 16]} />
        </mesh>
      </group>

      {/* Front Right */}
      <group ref={wheelFR} position={[0.15, 0.05, -0.20]}>
        <mesh rotation={[Math.PI / 2, 0, 0]} material={tireMat}>
          <cylinderGeometry args={[0.05, 0.05, 0.04, 32]} />
        </mesh>
        <mesh rotation={[Math.PI / 2, 0, 0]} material={rimMat}>
          <cylinderGeometry args={[0.02, 0.02, 0.042, 16]} />
        </mesh>
      </group>

      {/* Back Left */}
      <group ref={wheelBL} position={[-0.15, 0.05, 0.20]}>
        <mesh rotation={[Math.PI / 2, 0, 0]} material={tireMat}>
          <cylinderGeometry args={[0.05, 0.05, 0.04, 32]} />
        </mesh>
        <mesh rotation={[Math.PI / 2, 0, 0]} material={rimMat}>
          <cylinderGeometry args={[0.02, 0.02, 0.042, 16]} />
        </mesh>
      </group>

      {/* Back Right */}
      <group ref={wheelBR} position={[-0.15, 0.05, -0.20]}>
        <mesh rotation={[Math.PI / 2, 0, 0]} material={tireMat}>
          <cylinderGeometry args={[0.05, 0.05, 0.04, 32]} />
        </mesh>
        <mesh rotation={[Math.PI / 2, 0, 0]} material={rimMat}>
          <cylinderGeometry args={[0.02, 0.02, 0.042, 16]} />
        </mesh>
      </group>

      {/* --- Aesthetic Decals --- */}
      <mesh position={[-0.26, 0.10, 0]} rotation={[0, 0, 0]} material={rimMat}>
         <boxGeometry args={[0.02, 0.10, 0.20]} />
      </mesh>
      
      {/* Direction Arrow (Pointer to Front) */}
      <mesh position={[0.2, 0.191, 0]} rotation={[-Math.PI / 2, 0, -Math.PI / 2]}>
        <coneGeometry args={[0.06, 0.15, 3]} />
        <meshBasicMaterial color={THEME.accent} transparent opacity={0.6} />
      </mesh>

    </group>
  );
}
