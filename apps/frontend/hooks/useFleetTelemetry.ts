import { useCallback, useEffect, useState } from "react";
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

// ─── Global State & Throttling ──────────────────────────────────────────────
let globalRobotsMap = new Map<string, RobotState>();
let globalIsConnected = false;
let globalMapData: MapData | null = null;
let globalPathData: { x: number; y: number }[] = [];

const listeners = new Set<() => void>();

let notifyPending = false;
function notifyListeners() {
  if (notifyPending) return;
  notifyPending = true;
  // 100ms throttle (10Hz max React render rate) to prevent UI thread starvation
  setTimeout(() => {
    listeners.forEach((listener) => listener());
    notifyPending = false;
  }, 100);
}

// ─── Global Socket Init ──────────────────────────────────────────────────────
let isInitialized = false;

function initGlobalTelemetry() {
  if (typeof window === "undefined" || isInitialized) return;
  isInitialized = true;

  const socket = getSocket();

  function trigger() {
    notifyListeners();
  }

  socket.on("connect", () => {
    console.log("[Frontend] Connected to Telemetry Gateway");
    globalIsConnected = true;
    socket.emit("subscribe", "fleet");
    socket.emit("subscribe", "diagnostics");
    trigger();
  });

  socket.on("disconnect", () => {
    console.log("[Frontend] Disconnected");
    globalIsConnected = false;
    trigger();
  });

  socket.on("connect_error", (err: Error) => {
    console.warn("[Frontend] Backend not available, retrying...", err.message);
  });

  socket.on("telemetry", (data: TelemetryData) => {
    const existing = globalRobotsMap.get(data.id) || {
      ...data,
      status: "ONLINE",
      lastSeen: 0,
    };
    
    // Mutate the map, but trigger a throttled React render
    globalRobotsMap.set(data.id, {
      ...existing,
      ...data,
      status: "ONLINE",
      lastSeen: Date.now(),
    } as RobotState);
    
    trigger();
  });

  socket.on("map", (data: MapData) => {
    globalMapData = data;
    trigger();
  });

  socket.on("plan", (data: { x: number; y: number }[]) => {
    globalPathData = data;
    trigger();
  });

  if (!socket.connected) {
    socket.connect();
  } else {
    globalIsConnected = true;
    socket.emit("subscribe", "fleet");
    socket.emit("subscribe", "diagnostics");
    trigger();
  }
}

// ─── Hook ───────────────────────────────────────────────────────────────────
export function useFleetTelemetry() {
  // Initialize socket listeners only once globally
  useEffect(() => {
    initGlobalTelemetry();
  }, []);

  // Force re-renders bounded by the 100ms throttle block above
  const [, forceRender] = useState({});
  useEffect(() => {
    const listener = () => forceRender({});
    listeners.add(listener);
    // Return early snapshot render exactly once on mount
    listener();
    return () => {
      listeners.delete(listener);
    };
  }, []);

  // Memoized methods interacting with socket directly
  const sendControl = useCallback(
    (robotId: string, linear: number, angular: number) => {
      getSocket().emit("control", { robotId, linear, angular });
    },
    []
  );

  const subscribeToRobot = useCallback((robotId: string) => {
    getSocket().emit("subscribe", `robot:${robotId}`);
    globalPathData = []; // Clear current path line
    notifyListeners();
  }, []);

  const unsubscribeFromRobot = useCallback((robotId: string) => {
    getSocket().emit("unsubscribe", `robot:${robotId}`);
  }, []);

  const sendNavigationGoal = useCallback(
    (robotId: string, x: number, y: number, theta: number = 0) => {
      getSocket().emit("navigate", { robotId, x, y, theta });
    },
    []
  );

  // Return a fresh array copy each render so React detects diffs,
  // but note that this only happens max 10 times a sec!
  return {
    robots: Array.from(globalRobotsMap.values()),
    isConnected: globalIsConnected,
    mapData: globalMapData,
    pathData: globalPathData,
    sendControl,
    sendNavigationGoal,
    subscribeToRobot,
    unsubscribeFromRobot,
  };
}
