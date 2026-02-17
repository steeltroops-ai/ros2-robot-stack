"use client";

import { useEffect, useState, useRef } from "react";
import { io, Socket } from "socket.io-client";

// Shared type (Should be in shared-types in production)
export interface RobotState {
  id: string;
  x: number;
  y: number;
  theta: number;
  battery: number;
  status: "ONLINE" | "OFFLINE";
  lastSeen: number;
}

const SOCKET_URL = "http://localhost:4000";

export function useFleetTelemetry() {
  const [robots, setRobots] = useState<Map<string, RobotState>>(new Map());
  const socketRef = useRef<Socket | null>(null);
  const [isConnected, setIsConnected] = useState(false);

  useEffect(() => {
    // 1. Initialize Socket
    const socket = io(SOCKET_URL, {
      transports: ["websocket"],
      reconnectionAttempts: 5,
    });
    socketRef.current = socket;

    // 2. Connection Handlers
    socket.on("connect", () => {
      console.log("[Frontend] Connected to Telemetry Gateway");
      setIsConnected(true);
    });

    socket.on("disconnect", () => {
      console.log("[Frontend] Disconnected");
      setIsConnected(false);
    });

    // 3. Telemetry Listener
    socket.on("telemetry", (data: any) => {
      setRobots((prev) => {
        const next = new Map(prev);
        next.set(data.id, {
          ...data,
          status: "ONLINE",
          lastSeen: Date.now(),
        });
        return next;
      });
    });

    return () => {
      socket.disconnect();
    };
  }, []);

  return {
    robots: Array.from(robots.values()),
    isConnected,
  };
}
