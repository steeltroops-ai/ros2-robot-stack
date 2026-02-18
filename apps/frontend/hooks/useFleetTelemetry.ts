import { useEffect, useState } from "react";
import { socket } from "@/utils/socket";

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
  const [isConnected, setIsConnected] = useState(socket.connected);
  const [mapData, setMapData] = useState<MapData | null>(null);
  const [pathData, setPathData] = useState<{ x: number; y: number }[]>([]);

  useEffect(() => {
    function onConnect() {
      console.log("[Frontend] Connected to Telemetry Gateway");
      setIsConnected(true);
      socket.emit("subscribe", "fleet");
      socket.emit("subscribe", "diagnostics");
    }

    function onDisconnect() {
      console.log("[Frontend] Disconnected");
      setIsConnected(false);
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
    socket.on("telemetry", onTelemetry);
    socket.on("map", onMap);
    socket.on("plan", onPlan);
    
    // Check initial state
    if (socket.connected) {
        onConnect();
    }

    return () => {
      socket.off("connect", onConnect);
      socket.off("disconnect", onDisconnect);
      socket.off("telemetry", onTelemetry);
      socket.off("map", onMap);
      socket.off("plan", onPlan);
    };
  }, []);

  const sendControl = (robotId: string, linear: number, angular: number) => {
    socket.emit("control", { robotId, linear, angular });
  };

  const subscribeToRobot = (robotId: string) => {
    socket.emit("subscribe", `robot:${robotId}`);
    setPathData([]); // Clear path on new sub
  };

  const unsubscribeFromRobot = (robotId: string) => {
    socket.emit("unsubscribe", `robot:${robotId}`);
  };


  const sendNavigationGoal = (robotId: string, x: number, y: number, theta: number = 0) => {
    socket.emit("navigate", { robotId, x, y, theta });
  };

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
