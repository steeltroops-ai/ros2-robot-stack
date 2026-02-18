"use client";

import { useFrame } from "@react-three/fiber";
import { useRef } from "react";
import * as THREE from "three";

interface DestinationMarkerProps {
  position: [number, number, number] | null;
}

export function DestinationMarker({ position }: DestinationMarkerProps) {
  const meshRef = useRef<THREE.Mesh>(null!);
  const ringRef = useRef<THREE.Mesh>(null!);

  useFrame((state) => {
    if (!meshRef.current) return;
    const t = state.clock.getElapsedTime();
    
    // Animate holographic pillar
    meshRef.current.position.y = 0.5 + Math.sin(t * 2) * 0.1;
    meshRef.current.rotation.y += 0.01;
    
    // Animate pulse ring
    if (ringRef.current) {
        ringRef.current.scale.setScalar(1 + Math.sin(t * 4) * 0.2);
        (ringRef.current.material as THREE.MeshBasicMaterial).opacity = 0.5 - Math.sin(t * 4) * 0.2;
    }
  });

  if (!position) return null;

  return (
    <group position={position}>
      {/* Holographic Pillar */}
      <mesh ref={meshRef}>
        <cylinderGeometry args={[0.02, 0.02, 1, 8]} />
        <meshBasicMaterial color="#22d3ee" transparent opacity={0.6} />
      </mesh>
      
      {/* Ground Ring */}
      <mesh ref={ringRef} rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.01, 0]}>
        <ringGeometry args={[0.2, 0.25, 32]} />
        <meshBasicMaterial color="#22d3ee" transparent opacity={0.4} />
      </mesh>
      
      {/* Point Light */}
      <pointLight color="#22d3ee" intensity={0.5} distance={2} />
    </group>
  );
}
