"use client";

import { useEffect, useRef } from "react";

import { MapData, RobotState } from "@/hooks/useFleetTelemetry";

interface MapDisplayProps {
  mapData: MapData | null;
  robots: RobotState[];
  onSelectRobot?: (id: string | null) => void;
  selectedId?: string | null;
}

export function MapDisplay({ mapData, robots, onSelectRobot, selectedId }: MapDisplayProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  // Cache the expensive map ImageData so we don't recompute pixels every robot tick
  const cachedMapImage = useRef<ImageData | null>(null);
  const cachedMapDataRef = useRef<MapData | null>(null);

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
      const originX = mapData.info.origin?.position.x || 0;
      const originY = mapData.info.origin?.position.y || 0;
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

  // ── Phase 1: Expensive map pixel rendering — only when mapData changes ──
  useEffect(() => {
    if (!mapData || !canvasRef.current) return;
    // Skip if the same mapData ref
    if (mapData === cachedMapDataRef.current && cachedMapImage.current) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const width = mapData.info.width;
    const height = mapData.info.height;

    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }

    const data = mapData.data;
    const imgData = ctx.createImageData(width, height);

    for (let i = 0; i < data.length; i++) {
      const val = data[i];
      let r = 5, g = 5, b = 5, a = 0; // unknown

      if (val === 0) {
        r = 255; g = 255; b = 255; a = 15;
      } else if (val === 100) {
        r = 16; g = 185; b = 129; a = 255;
      } else if (val > 0) {
        r = 16; g = 185; b = 129; a = Math.max(80, Math.floor(val * 2.55));
      }

      const row = Math.floor(i / width);
      const col = i % width;
      const targetIndex = (height - 1 - row) * width + col;

      imgData.data[targetIndex * 4 + 0] = r;
      imgData.data[targetIndex * 4 + 1] = g;
      imgData.data[targetIndex * 4 + 2] = b;
      imgData.data[targetIndex * 4 + 3] = val === -1 ? 0 : a;
    }

    // Cache the computed image data
    cachedMapImage.current = imgData;
    cachedMapDataRef.current = mapData;

    ctx.putImageData(imgData, 0, 0);
  }, [mapData]);

  // ── Phase 2: Cheap robot overlay — runs on robot position / selection changes ──
  useEffect(() => {
    if (!mapData || !canvasRef.current || !cachedMapImage.current) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const width = mapData.info.width;
    const height = mapData.info.height;
    const resolution = mapData.info.resolution;

    // Restore the cached map base (clears previous robot dots)
    ctx.putImageData(cachedMapImage.current, 0, 0);

    // Draw robots (cheap — just a few circles)
    robots.forEach((robot) => {
      const originX = mapData.info.origin?.position.x || 0;
      const originY = mapData.info.origin?.position.y || 0;

      const px = (robot.x - originX) / resolution;
      const py = (robot.y - originY) / resolution;

      const canvasY = height - py;
      const canvasX = px;

      const isSelected = selectedId === robot.id;
      
      if (isSelected) {
        ctx.fillStyle = "rgba(16, 185, 129, 0.3)";
        ctx.beginPath();
        ctx.arc(canvasX, canvasY, 12, 0, 2 * Math.PI);
        ctx.fill();
        
        ctx.strokeStyle = "#10b981";
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      ctx.fillStyle = isSelected ? "#10b981" : "#050505";
      ctx.strokeStyle = "#10b981";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(canvasX, canvasY, 4, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();

      // Heading line
      const headLen = 8;
      const dx = Math.cos(robot.theta) * headLen;
      const dy = Math.sin(robot.theta) * headLen;

      ctx.strokeStyle = "#10b981";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(canvasX, canvasY);
      ctx.lineTo(canvasX + dx, canvasY - dy);
      ctx.stroke();
    });
  }, [mapData, robots, selectedId]);

  if (!mapData) {
    return (
      <div className="w-full h-full flex flex-col items-center justify-center gap-3 bg-[var(--color-surface-0)] border border-[var(--color-border-0)]/50 backdrop-blur-sm">
         <div className="w-8 h-8 rounded-full border-2 border-[var(--color-primary)]/20 border-t-[var(--color-primary)] animate-spin" />
         <span className="text-[10px] font-mono font-bold text-[var(--color-primary)] tracking-widest uppercase animate-pulse">Scanning Grid...</span>
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
