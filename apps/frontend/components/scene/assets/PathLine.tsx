"use client";

import { useMemo } from "react";
import * as THREE from "three";

interface PathLineProps {
  path: { x: number; y: number }[];
}

export function PathLine({ path }: PathLineProps) {
  const points = useMemo(() => {
    return path.map((p) => new THREE.Vector3(p.x, 0.05, -p.y));
  }, [path]);

  if (points.length < 2) return null;

  const curve = new THREE.CatmullRomCurve3(points);
  const tubeGeometry = new THREE.TubeGeometry(curve, points.length * 2, 0.015, 8, false);

  return (
    <mesh geometry={tubeGeometry}>
      <meshBasicMaterial color="#22d3ee" transparent opacity={0.4} />
    </mesh>
  );
}
