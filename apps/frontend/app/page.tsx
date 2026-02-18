"use client";

import DashboardLayout from "@/components/layout/DashboardLayout";
import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";
import { TeleopPanel } from "@/components/telemetry/TeleopPanel";
import { MapDisplay } from "@/components/map/MapDisplay";
import {
  Bot,
  Battery,
  Gauge,
  Wifi,
  WifiOff,
  ChevronRight,
  Navigation,
  MapPin,
  Map,
  CircleStop,
} from "lucide-react";
import { useState, useEffect } from "react";
import { useSearchParams, useRouter, usePathname } from "next/navigation";
import dynamic from "next/dynamic";

const RobotViewer = dynamic(
  () => import("@/components/scene/RobotViewer"),
  { 
    ssr: false,
    loading: () => (
      <div className="w-full h-full flex flex-col items-center justify-center bg-black gap-3">
         <div className="w-8 h-8 rounded-full border-2 border-zinc-800 border-t-zinc-400 animate-spin" />
         <span className="text-[10px] font-mono font-bold text-zinc-600 tracking-widest uppercase animate-pulse">Initializing Visualizer</span>
      </div>
    )
  }
);

function StatusBadge({ status }: { status: string }) {
  const isOnline = status === "online" || status === "moving";
  return (
    <div 
      className="flex items-center gap-1.5 px-2 py-0.5 rounded-full"
      style={{ 
        background: isOnline ? "rgba(52, 199, 89, 0.1)" : "rgba(142, 142, 147, 0.1)",
        border: `1px solid ${isOnline ? "rgba(52, 199, 89, 0.2)" : "rgba(142, 142, 147, 0.2)"}`
      }}
    >
      <div className={`w-1 h-1 rounded-full ${isOnline ? "bg-green-500 animate-pulse" : "bg-zinc-400"}`} />
      <span style={{ fontSize: "0.625rem", fontWeight: 700, color: isOnline ? "#1a7f37" : "#8e8e93", textTransform: "uppercase" }}>
        {status}
      </span>
    </div>
  );
}

function BatteryIndicator({ level }: { level: number }) {
  const color =
    level > 50
      ? "var(--color-success)"
      : level > 20
        ? "var(--color-warning)"
        : "var(--color-error)";
  return (
    <div className="flex items-center gap-2">
      <div
        style={{
          width: 48,
          height: 6,
          borderRadius: 3,
          background: "var(--color-surface-3)",
          overflow: "hidden",
        }}
      >
        <div
          style={{
            width: `${level}%`,
            height: "100%",
            borderRadius: 3,
            background: color,
            transition: "width 0.3s ease",
          }}
        />
      </div>
      <span
        className="data-mono"
        style={{
          fontSize: "0.75rem",
          color: "var(--color-text-2)",
        }}
      >
        {level.toFixed(0)}%
      </span>
    </div>
  );
}

export default function Home() {
  const searchParams = useSearchParams();
  const router = useRouter();
  const pathname = usePathname();
  const { robots, isConnected, sendControl, mapData, subscribeToRobot, unsubscribeFromRobot } = useFleetTelemetry();
  const [mounted, setMounted] = useState(false);
  useEffect(() => {
    setMounted(true);
  }, []);

  const selectedRobot = searchParams.get("robot");

  const setSelectedRobot = (id: string | null) => {
    const params = new URLSearchParams(searchParams.toString());
    if (id) {
      params.set("robot", id);
    } else {
      params.delete("robot");
    }
    router.push(`${pathname}?${params.toString()}`, { scroll: false });
  };

  // Smart Subscription Management
  useEffect(() => {
    if (selectedRobot) {
      subscribeToRobot(selectedRobot);
    }
    return () => {
      if (selectedRobot) unsubscribeFromRobot(selectedRobot);
    };
  }, [selectedRobot, subscribeToRobot, unsubscribeFromRobot]);

  const selectedData = robots.find((r) => r.id === selectedRobot);
  const avgBattery =
    robots.length > 0
      ? Math.round(robots.reduce((a, r) => a + r.battery, 0) / robots.length)
      : 0;

  if (!mounted) return null;

  return (
    <DashboardLayout
      noPadding
      rightPanel={
        selectedRobot ? (
          <div className="flex flex-col h-full bg-zinc-950 text-white">
            {/* 3D Cockpit Header */}
            <div className="p-4 border-b border-white/5 flex items-center justify-between">
               <div className="flex items-center gap-3">
                  <div className="w-2 h-2 rounded-full bg-cyan-400 animate-pulse" />
                  <span className="text-xs font-bold tracking-widest uppercase text-cyan-400">
                    Cockpit: {selectedRobot.replace("_", "-")}
                  </span>
               </div>
               <button 
                  onClick={() => setSelectedRobot(null)}
                  className="p-1.5 hover:bg-white/10 rounded-full transition-colors"
               >
                  <CircleStop size={16} />
               </button>
            </div>

            {/* 3D Viewport - The Only One */}
            <div className="flex-1 min-h-[40%] relative bg-black">
              <RobotViewer
                robotId={selectedRobot}
                pose={{
                  x: selectedData?.x ?? 0,
                  y: selectedData?.y ?? 0,
                  theta: selectedData?.theta ?? 0,
                }}
              />
            </div>

            {/* Tactical Controls */}
            <div className="p-6 flex flex-col gap-6 overflow-y-auto">
                <TeleopPanel
                  onControl={(l, a) => {
                    console.log(`[Dashboard] Sending Teleop to ${selectedRobot}: v=${l}, w=${a}`);
                    sendControl(selectedRobot, l, a);
                  }}
                />

               <div className="flex flex-col gap-3">
                  <span className="text-[10px] font-black uppercase tracking-widest text-zinc-500">Live Telemetry</span>
                  <div className="grid grid-cols-2 gap-2">
                     {[
                        { label: "X-COORD", value: selectedData?.x.toFixed(2) },
                        { label: "Y-COORD", value: selectedData?.y.toFixed(2) },
                        { label: "HEADING", value: `${((selectedData?.theta ?? 0) * 180 / Math.PI).toFixed(0)}°` },
                        { label: "BATTERY", value: `${selectedData?.battery?.toFixed(0)}%` },
                     ].map((m, i) => (
                        <div key={i} className="bg-white/5 p-3 rounded-xl border border-white/5">
                           <div className="text-[9px] font-bold text-zinc-500 mb-1">{m.label}</div>
                           <div className="text-sm font-mono font-bold text-white">{m.value}</div>
                        </div>
                     ))}
                  </div>
               </div>
            </div>
          </div>
        ) : null
      }
    >
      <div
        className="flex flex-col gap-6 p-6"
        style={{ height: "100%", minHeight: 0 }}
      >
        {/* Metric Cards - RESTORED */}
        <div
          className="grid flex-shrink-0"
          style={{
            gridTemplateColumns: "repeat(4, 1fr)",
            gap: "0.75rem",
          }}
        >
          <div className="metric-card">
            <div className="metric-label" style={{ marginBottom: 4 }}>
              Active Units
            </div>
            <div className="flex items-center justify-between">
              <span className="metric-value" style={{ fontSize: "1.25rem" }}>{robots.length}</span>
              <div className="px-2 py-0.5 rounded bg-black/5 text-[0.625rem] font-bold">LIVE</div>
            </div>
          </div>

          <div className="metric-card">
            <div className="metric-label" style={{ marginBottom: 4 }}>
              Avg Battery
            </div>
            <div className="flex items-center justify-between">
              <span className="metric-value" style={{ fontSize: "1.25rem" }}>{avgBattery}%</span>
              <div className={`w-2 h-2 rounded-full ${avgBattery > 20 ? "bg-green-500" : "bg-red-500"}`} />
            </div>
          </div>

          <div className="metric-card">
            <div className="metric-label" style={{ marginBottom: 4 }}>
              System Latency
            </div>
            <div className="flex items-center justify-between">
              <span className="metric-value" style={{ fontSize: "1.25rem" }}>
                {isConnected ? "32" : "--"}
                <span className="ml-1 text-[0.75rem] font-medium opacity-40">ms</span>
              </span>
            </div>
          </div>

          <div className="metric-card">
            <div className="metric-label" style={{ marginBottom: 4 }}>
              Nav Tasks
            </div>
            <div className="flex items-center justify-between">
              <span className="metric-value" style={{ fontSize: "1.25rem" }}>0</span>
              <div className="text-[0.625rem] font-bold opacity-30">IDLE</div>
            </div>
          </div>
        </div>

        {/* Fleet Units Selection List */}
        <div className="flex-1 min-h-0 flex flex-col">
          <div className="flex items-center justify-between px-2 mb-4">
            <div className="flex flex-col">
              <h3 style={{ fontSize: "1rem", fontWeight: 800, color: "var(--color-text-0)", letterSpacing: "-0.02em" }}>
                Fleet Inventory
              </h3>
              <span style={{ fontSize: "0.6875rem", color: "var(--color-text-3)" }}>Operational status per unit</span>
            </div>
          </div>
          
          <div className="flex-1 overflow-y-auto min-h-0 px-1">
            {robots.length === 0 ? (
              <div className="panel flex flex-col items-center justify-center p-12">
                  <p className="text-sm font-medium text-zinc-400">Scanning for telemetry signals...</p>
              </div>
            ) : (
              <div className="flex flex-col gap-2">
                {robots.map((robot) => {
                  const isActive = selectedRobot === robot.id;
                  return (
                    <div
                      key={robot.id}
                      className={`group relative transition-all cursor-pointer ${
                        isActive 
                          ? "bg-black text-white shadow-xl ring-1 ring-black border-transparent" 
                          : "bg-white hover:bg-zinc-50 border-zinc-200 shadow-sm"
                      }`}
                      style={{
                        borderRadius: "0.75rem",
                        border: isActive ? "none" : "1px solid #e5e7eb",
                        display: "flex",
                        alignItems: "center",
                        padding: "0.75rem 1.25rem",
                        gap: "1.5rem",
                      }}
                      onClick={() => setSelectedRobot(isActive ? null : robot.id)}
                    >
                      {/* Identity Section */}
                      <div className="flex flex-col gap-0.5 min-w-[120px]">
                        <span style={{ fontSize: "0.625rem", fontWeight: 800, color: isActive ? "rgba(255,255,255,0.4)" : "var(--color-text-4)", textTransform: "uppercase", letterSpacing: "0.05em" }}>
                          Unit ID
                        </span>
                        <span style={{ fontSize: "0.875rem", fontWeight: 800, color: isActive ? "white" : "var(--color-text-0)", letterSpacing: "-0.01em" }}>
                          {robot.id.replace("_", "-").toUpperCase()}
                        </span>
                      </div>

                      {/* Status Section */}
                      <div className="flex flex-col gap-0.5 min-w-[100px]">
                        <span style={{ fontSize: "0.625rem", fontWeight: 800, color: isActive ? "rgba(255,255,255,0.4)" : "var(--color-text-4)", textTransform: "uppercase", letterSpacing: "0.05em" }}>
                          Status
                        </span>
                        <StatusBadge status="online" />
                      </div>

                      {/* Battery Section - Compact Horizontal */}
                      <div className="flex-1 flex items-center gap-4">
                        <div className="flex flex-col gap-0.5 flex-1">
                          <span style={{ fontSize: "0.625rem", fontWeight: 800, color: isActive ? "rgba(255,255,255,0.4)" : "var(--color-text-4)", textTransform: "uppercase", letterSpacing: "0.05em" }}>
                            Power
                          </span>
                          <div className="w-full">
                            <BatteryIndicator level={robot.battery} />
                          </div>
                        </div>
                      </div>

                      {/* Position Section */}
                      <div className="flex flex-col gap-0.5 min-w-[140px] text-right">
                        <span style={{ fontSize: "0.625rem", fontWeight: 800, color: isActive ? "rgba(255,255,255,0.4)" : "var(--color-text-4)", textTransform: "uppercase", letterSpacing: "0.05em" }}>
                          Coordinates (M)
                        </span>
                        <span className="font-mono" style={{ fontSize: "0.75rem", color: isActive ? "rgba(255,255,255,0.8)" : "var(--color-text-2)", fontWeight: 600 }}>
                          {robot.x.toFixed(2)}, {robot.y.toFixed(2)}
                        </span>
                      </div>

                      {/* Arrow / Active Indicator */}
                      <div className="w-6 flex justify-end">
                        <ChevronRight 
                          size={16} 
                          className={`transition-all duration-300 ${isActive ? "text-white translate-x-0" : "text-zinc-200 -translate-x-2 opacity-0 group-hover:opacity-100 group-hover:translate-x-0"}`} 
                        />
                      </div>
                    </div>
                  );
                })}
              </div>
            )}
          </div>
        </div>
      </div>
    </DashboardLayout>
  );
}
