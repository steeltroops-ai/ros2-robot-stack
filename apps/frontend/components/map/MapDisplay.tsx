"use client";

import { useEffect, useRef } from "react";

interface MapDisplayProps {
  mapData: any;
  robots: any[];
  onSelectRobot?: (id: string | null) => void;
  selectedId?: string | null;
}

export function MapDisplay({ mapData, robots, onSelectRobot, selectedId }: MapDisplayProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  const handleClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!mapData || !onSelectRobot) return;
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    
    const clickX = (e.clientX - rect.left) * scaleX;
    const clickY = (e.clientY - rect.top) * scaleY;

    // Check hit on robots
    const hitRadius = 10; // Pixels on canvas
    let found = null;

    robots.forEach((robot) => {
      const originX = mapData.info.origin.position.x;
      const originY = mapData.info.origin.position.y;
      const resolution = mapData.info.resolution;
      
      const rx = (robot.x - originX) / resolution;
      const ry = mapData.info.height - (robot.y - originY) / resolution;

      const dist = Math.sqrt((clickX - rx) ** 2 + (clickY - ry) ** 2);
      if (dist < hitRadius) {
        found = robot.id;
      }
    });

    onSelectRobot(found);
  };

  useEffect(() => {
    if (!mapData || !canvasRef.current) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const width = mapData.info.width;
    const height = mapData.info.height;
    const resolution = mapData.info.resolution;

    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }

    const data = mapData.data;
    const imgData = ctx.createImageData(width, height);

    for (let i = 0; i < data.length; i++) {
      const val = data[i];
      let r = 229, g = 229, b = 234; // unknown (#e5e5ea)

      if (val === 0) {
        r = 255; g = 255; b = 255; // free (#ffffff)
      } else if (val === 100) {
        r = 28; g = 28; b = 30; // occupied (#1c1c1e)
      }

      const row = Math.floor(i / width);
      const col = i % width;
      const targetIndex = (height - 1 - row) * width + col;

      imgData.data[targetIndex * 4 + 0] = r;
      imgData.data[targetIndex * 4 + 1] = g;
      imgData.data[targetIndex * 4 + 2] = b;
      imgData.data[targetIndex * 4 + 3] = 255;
    }

    ctx.putImageData(imgData, 0, 0);

    // Draw robots
    robots.forEach((robot) => {
      const originX = mapData.info.origin.position.x;
      const originY = mapData.info.origin.position.y;

      const px = (robot.x - originX) / resolution;
      const py = (robot.y - originY) / resolution;

      const canvasY = height - py;
      const canvasX = px;

      // Robot dot — primary black
      const isSelected = selectedId === robot.id;
      
      if (isSelected) {
        ctx.fillStyle = "rgba(34, 211, 238, 0.3)";
        ctx.beginPath();
        ctx.arc(canvasX, canvasY, 12, 0, 2 * Math.PI);
        ctx.fill();
        
        ctx.strokeStyle = "#22d3ee";
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      ctx.fillStyle = isSelected ? "#22d3ee" : "#1c1c1e";
      ctx.beginPath();
      ctx.arc(canvasX, canvasY, 4, 0, 2 * Math.PI);
      ctx.fill();

      // Heading line
      const headLen = 8;
      const dx = Math.cos(robot.theta) * headLen;
      const dy = Math.sin(robot.theta) * headLen;

      ctx.strokeStyle = isSelected ? "#22d3ee" : "#1c1c1e";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(canvasX, canvasY);
      ctx.lineTo(canvasX + dx, canvasY - dy);
      ctx.stroke();
    });
  }, [mapData, robots, selectedId]);

  if (!mapData) {
    return (
      <div className="w-full h-full flex flex-col items-center justify-center gap-3">
         <div className="w-8 h-8 rounded-full border-2 border-zinc-200 border-t-zinc-400 animate-spin" />
         <span className="text-[10px] font-mono font-bold text-zinc-400 tracking-widest uppercase animate-pulse">Scanning Grid...</span>
      </div>
    );
  }

  return (
    <canvas
      ref={canvasRef}
      onClick={handleClick}
      style={{
        width: "100%",
        height: "100%",
        objectFit: "contain",
        imageRendering: "pixelated",
        cursor: "crosshair"
      }}
    />
  );
}
