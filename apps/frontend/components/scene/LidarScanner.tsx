"use client";

import { useFrame, useThree } from "@react-three/fiber";
import { useEffect, useRef, useState, useMemo } from "react";
import * as THREE from "three";
import { socket } from "@/utils/socket";

// Simplified Scan Msg
interface ScanMsg {
  ranges: number[];
  angle_min: number;
  angle_increment: number;
  range_max: number;
}

interface LidarScannerProps {
  robotId: string;
}

export function LidarScanner({ robotId }: LidarScannerProps) {
  const pointsRef = useRef<THREE.Points>(null!);
  const [scanData, setScanData] = useState<ScanMsg | null>(null);

  // Buffer Geometry for Points
  // Max ranges usually 360 or 720. Let's reserve 1080 to be safe.
  const MAX_POINTS = 1080;
  
  const geometry = useMemo(() => {
    const geo = new THREE.BufferGeometry();
    const positions = new Float32Array(MAX_POINTS * 3);
    const colors = new Float32Array(MAX_POINTS * 3);
    geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    return geo;
  }, []);

  const material = useMemo(() => new THREE.PointsMaterial({
    vertexColors: true,
    size: 0.08,
    sizeAttenuation: true,
    transparent: true,
    opacity: 0.8,
    blending: THREE.AdditiveBlending
  }), []);

  // Socket Listener
  useEffect(() => {
    if (!socket) return;
    
    const handler = (data: ScanMsg) => {
        setScanData(data);
    };

    socket.on("scan", handler);
    return () => {
      socket.off("scan", handler);
    };
  }, [robotId]);

  // Update Geometry Frame Loop
  useFrame(() => {
    if (!scanData || !pointsRef.current) return;
    
    const positions = pointsRef.current.geometry.attributes.position.array as Float32Array;
    const colors = pointsRef.current.geometry.attributes.color.array as Float32Array;
    const count = Math.min(scanData.ranges.length, MAX_POINTS);
    
    let validPoints = 0;

    for (let i = 0; i < count; i++) {
        const range = scanData.ranges[i];
        
        // Filter invalid ranges (too close or infinite)
        if (range < 0.1 || range > scanData.range_max || !isFinite(range)) {
            // Hide point by moving to 0,0,0 or just skipping?
            // Better to zero out or move far away.
            positions[validPoints * 3] = 0;
            positions[validPoints * 3 + 1] = 0;
            positions[validPoints * 3 + 2] = 0;
            // Make invisible
            continue;
        }

        const angle = scanData.angle_min + i * scanData.angle_increment;
        
        // Polar to Cartesian (Local Robot Frame)
        // X = Forward, Y = Left in ROS
        // Three.js: X = Right, Y = Up, Z = Backward? 
        // No. our RobotViewer converts:
        // ROS X -> Three X
        // ROS Y -> Three -Z
        // We are INSIDE the robot group. 
        // The robot group is already rotated by -theta.
        // So we are in the Robot's LOCAL frame.
        // In local frame: X is Forward.
        // In Three terms: X is Forward? 
        // Let's check SceneContent: 
        //   robotRef.current.position.set(x, 0, -y);
        //   robotRef.current.rotation.y = theta;
        // So global X is "Right" in Three?
        // Wait. RobotViewer setup:
        // grid is X/Z plane.
        // ROS: X=East, Y=North.
        // Three: X=East, -Z=North.
        // Rotation Y (Yaw) matches ROS Yaw (if sign is handled).
        
        // Inside the group (Local Frame):
        // X should be Forward.
        // So x = r * cos(a), y = r * sin(a) (ROS local x,y)
        // Convert to Three Local: (x, 0, -y)
        
        const lx = range * Math.cos(angle);
        const ly = range * Math.sin(angle);
        
        positions[validPoints * 3] = lx;
        positions[validPoints * 3 + 1] = 0.2; // Height of Lidar deck (approx)
        positions[validPoints * 3 + 2] = -ly; // Flip Y to -Z
        
        // Color Coding based on Distance
        // < 1m = Red (Danger)
        // > 1m = Green/Teal (Safe)
        const intensity = Math.max(0, 1 - range / 5.0); // Fades out with distance
        
        if (range < 0.8) {
            // Danger Red
            colors[validPoints * 3] = 1;
            colors[validPoints * 3 + 1] = 0;
            colors[validPoints * 3 + 2] = 0;
        } else {
            // Cyber Cyan/Green
            colors[validPoints * 3] = 0; 
            colors[validPoints * 3 + 1] = 1; 
            colors[validPoints * 3 + 2] = 1;
        }
        
        validPoints++;
    }
    
    // Set draw range to optimize
    pointsRef.current.geometry.setDrawRange(0, validPoints);
    pointsRef.current.geometry.attributes.position.needsUpdate = true;
    pointsRef.current.geometry.attributes.color.needsUpdate = true;
  });

  return (
    <points ref={pointsRef} geometry={geometry} material={material} />
  );
}
