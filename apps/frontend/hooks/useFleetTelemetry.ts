import { useEffect, useState, useCallback, useRef } from "react";
import { getSocket } from "@/utils/socket";

export interface RobotState {
  id: string;
  x: number;
  y: number;
  theta: number;
  battery: number;
  status: "ONLINE" | "OFFLINE";
  lastSeen: number;
}

interface TelemetryData {
  id: string;
  x: number;
  y: number;
  theta: number;
  battery: number;
}

export interface MapData {
  info: {
    resolution: number;
    width: number;
    height: number;
    origin?: {
      position: {
        x: number;
        y: number;
      };
    };
  };
  data: number[];
}

export function useFleetTelemetry() {
  const [robots, setRobots] = useState<Map<string, RobotState>>(new Map());
  const [isConnected, setIsConnected] = useState(false);
  const [mapData, setMapData] = useState<MapData | null>(null);
  const [pathData, setPathData] = useState<{ x: number; y: number }[]>([]);
  
  // Use ref to avoid stale closure issues
  const socketRef = useRef(typeof window !== "undefined" ? getSocket() : null);

  useEffect(() => {
    const socket = socketRef.current;
    if (!socket) return;
    
    function onConnect() {
      console.log("[Frontend] Connected to Telemetry Gateway");
      setIsConnected(true);
      socket!.emit("subscribe", "fleet");
      socket!.emit("subscribe", "diagnostics");
    }

    function onDisconnect() {
      console.log("[Frontend] Disconnected");
      setIsConnected(false);
    }

    function onConnectError(err: Error) {
      // Silently handle connection errors - the socket will retry automatically
      // Only log once to avoid console spam
      console.warn("[Frontend] Backend not available, retrying...", err.message);
    }

    function onTelemetry(data: TelemetryData) {
      setRobots((prev) => {
        const next = new Map(prev);
        const existing = next.get(data.id) || { ...data, status: "ONLINE", lastSeen: 0 };
        next.set(data.id, {
          ...existing,
          ...data,
          status: "ONLINE",
          lastSeen: Date.now(),
        } as RobotState);
        return next;
      });
    }

    function onMap(data: MapData) {
      setMapData(data);
    }

    function onPlan(data: { x: number; y: number }[]) {
      setPathData(data);
    }

    socket.on("connect", onConnect);
    socket.on("disconnect", onDisconnect);
    socket.on("connect_error", onConnectError);
    socket.on("telemetry", onTelemetry);
    socket.on("map", onMap);
    socket.on("plan", onPlan);

    // Explicitly connect now (autoConnect is false)
    if (!socket.connected) {
      socket.connect();
    } else {
      // Already connected (e.g., from another hook instance)
      onConnect();
    }

    return () => {
      socket.off("connect", onConnect);
      socket.off("disconnect", onDisconnect);
      socket.off("connect_error", onConnectError);
      socket.off("telemetry", onTelemetry);
      socket.off("map", onMap);
      socket.off("plan", onPlan);
    };
  }, []);

  // Memoized control sender — stable reference prevents re-subscription loops
  const sendControl = useCallback((robotId: string, linear: number, angular: number) => {
    socketRef.current?.emit("control", { robotId, linear, angular });
  }, []);

  // Memoized subscribe/unsubscribe to prevent infinite re-render loops in useEffect deps
  const subscribeToRobot = useCallback((robotId: string) => {
    socketRef.current?.emit("subscribe", `robot:${robotId}`);
    setPathData([]); // Clear path on new sub
  }, []);

  const unsubscribeFromRobot = useCallback((robotId: string) => {
    socketRef.current?.emit("unsubscribe", `robot:${robotId}`);
  }, []);

  const sendNavigationGoal = useCallback((robotId: string, x: number, y: number, theta: number = 0) => {
    socketRef.current?.emit("navigate", { robotId, x, y, theta });
  }, []);

  return {
    robots: Array.from(robots.values()),
    isConnected,
    sendControl,
    sendNavigationGoal,
    mapData,
    pathData,
    subscribeToRobot,
    unsubscribeFromRobot,
  };
}
