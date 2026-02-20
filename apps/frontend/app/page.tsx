"use client";

import DashboardLayout from "@/components/layout/DashboardLayout";
import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";
import { useDashboardPanel } from "@/components/layout/panel-content/DashboardPanelContent";
import {
  Bot,
  Battery,
  Gauge,
  Wifi,
  WifiOff,
  ChevronRight,
  Navigation,
  CircleStop,
} from "lucide-react";
import { useState, useEffect, useCallback } from "react";
import { useSearchParams, useRouter, usePathname } from "next/navigation";
import { Suspense } from "react";

// ─── Status badge ─────────────────────────────────────────────────────────────
function StatusBadge({ status }: { status: string }) {
  const isOnline = status === "online" || status === "moving";
  return (
    <div
      className="flex items-center gap-1.5 px-2 py-0.5 rounded"
      style={{
        background: isOnline ? "rgba(16, 185, 129, 0.15)" : "rgba(239, 68, 68, 0.15)",
        border: `1px solid ${isOnline ? "rgba(16, 185, 129, 0.5)" : "rgba(239, 68, 68, 0.5)"}`,
      }}
    >
      <div
        className={`w-1 h-1 rounded-full ${isOnline ? "bg-[var(--color-success)] animate-pulse" : "bg-[var(--color-error)]"}`}
      />
      <span
        style={{
          fontSize: "0.625rem",
          fontWeight: 700,
          fontFamily: "var(--font-mono), monospace",
          color: isOnline ? "var(--color-success)" : "var(--color-error)",
          textTransform: "uppercase",
          letterSpacing: "0.1em",
        }}
      >
        {status}
      </span>
    </div>
  );
}

// ─── Battery bar ──────────────────────────────────────────────────────────────
function BatteryIndicator({ level }: { level: number }) {
  const color =
    level > 50 ? "var(--color-success)" : level > 20 ? "var(--color-warning)" : "var(--color-error)";
  return (
    <div className="flex items-center gap-2">
      <div style={{ width: 48, height: 6, borderRadius: 3, background: "var(--color-surface-3)", overflow: "hidden" }}>
        <div style={{ width: `${level}%`, height: "100%", borderRadius: 3, background: color, transition: "width 0.3s ease" }} />
      </div>
      <span className="data-mono" style={{ fontSize: "0.75rem", color: "var(--color-text-2)" }}>
        {level.toFixed(0)}%
      </span>
    </div>
  );
}

// ─── Main dashboard ───────────────────────────────────────────────────────────
function FleetDashboard() {
  const searchParams = useSearchParams();
  const router = useRouter();
  const pathname = usePathname();
  const { robots, isConnected, sendControl, subscribeToRobot, unsubscribeFromRobot } =
    useFleetTelemetry();
  const selectedRobot = searchParams.get("robot");

  const setSelectedRobot = useCallback(
    (id: string | null) => {
      const params = new URLSearchParams(searchParams.toString());
      if (id) params.set("robot", id);
      else params.delete("robot");
      router.push(`${pathname}?${params.toString()}`, { scroll: false });
    },
    [searchParams, router, pathname]
  );

  // Smart subscription
  useEffect(() => {
    if (selectedRobot) subscribeToRobot(selectedRobot);
    return () => { if (selectedRobot) unsubscribeFromRobot(selectedRobot); };
  }, [selectedRobot, subscribeToRobot, unsubscribeFromRobot]);

  const selectedData = robots.find((r) => r.id === selectedRobot);
  const avgBattery =
    robots.length > 0
      ? Math.round(robots.reduce((a, r) => a + r.battery, 0) / robots.length)
      : 0;

  // ── Register right panel content ──────────────────────────────────────────
  useDashboardPanel({
    selectedRobotId: selectedRobot,
    selectedData: selectedData
      ? { x: selectedData.x, y: selectedData.y, theta: selectedData.theta, battery: selectedData.battery }
      : undefined,
    robotCount: robots.length,
    avgBattery,
    isConnected,
    sendControl,
    onDeselect: () => setSelectedRobot(null),
  });


  return (
    <div className="flex flex-col gap-6 p-4 sm:p-6 w-full h-full overflow-y-auto animate-in fade-in duration-200">
      {/* Metric Cards */}
      <div className="grid grid-cols-1 sm:grid-cols-2 xl:grid-cols-4 flex-shrink-0" style={{ gap: "0.75rem" }}>
        <div className="metric-card">
          <div className="metric-label" style={{ marginBottom: 4 }}>Connected Robots</div>
          <div className="flex items-center justify-between">
            <span className="metric-value" style={{ fontSize: "1.25rem" }}>{robots.length}</span>
            <div className="px-2 py-0.5 rounded bg-black/5 text-[0.625rem] font-bold">LIVE</div>
          </div>
        </div>
        <div className="metric-card">
          <div className="metric-label" style={{ marginBottom: 4 }}>Average Power</div>
          <div className="flex items-center justify-between">
            <span className="metric-value" style={{ fontSize: "1.25rem" }}>{avgBattery}%</span>
            <div className={`w-2 h-2 rounded-full ${avgBattery > 20 ? "bg-green-500" : "bg-red-500"}`} />
          </div>
        </div>
        <div className="metric-card">
          <div className="metric-label" style={{ marginBottom: 4 }}>Network Speed</div>
          <div className="flex items-center justify-between">
            <span className="metric-value" style={{ fontSize: "1.25rem" }}>
              {isConnected ? "32" : "--"}
              <span className="ml-1 text-[0.75rem] font-medium opacity-40">ms</span>
            </span>
          </div>
        </div>
        <div className="metric-card">
          <div className="metric-label" style={{ marginBottom: 4 }}>Active Missions</div>
          <div className="flex items-center justify-between">
            <span className="metric-value" style={{ fontSize: "1.25rem" }}>0</span>
            <div className="text-[0.625rem] font-bold opacity-30">IDLE</div>
          </div>
        </div>
      </div>

      {/* Robot Roster */}
      <div className="flex-1 min-h-0 flex flex-col">
        <div className="flex items-center justify-between px-2 mb-4">
          <div className="flex flex-col">
            <h3 style={{ fontSize: "1rem", fontWeight: 800, color: "var(--color-text-0)", letterSpacing: "-0.02em" }}>
              Robot Roster
            </h3>
            <span style={{ fontSize: "0.6875rem", color: "var(--color-text-3)" }}>
              View and manage all connected robots
            </span>
          </div>
        </div>

        <div className="flex-1 overflow-y-auto min-h-0 px-1">
          {robots.length === 0 ? (
            <div className="panel flex flex-col items-center justify-center p-12">
              <p className="text-sm font-medium text-zinc-400">Waiting for robots to connect...</p>
            </div>
          ) : (
            <div className="flex flex-col gap-2">
              {robots.map((robot) => {
                const isActive = selectedRobot === robot.id;
                return (
                  <div
                    key={robot.id}
                    className={`group relative transition-all cursor-pointer`}
                    style={{
                      borderRadius: "0.25rem",
                      border: isActive
                        ? "1px solid var(--color-primary)"
                        : "1px solid var(--color-border-0)",
                      display: "flex",
                      alignItems: "center",
                      padding: "0.75rem 1.25rem",
                      gap: "1.5rem",
                      backdropFilter: "blur(10px)",
                      background: isActive
                        ? "rgba(16, 185, 129, 0.10)"
                        : "transparent",
                    }}
                    onClick={() => setSelectedRobot(isActive ? null : robot.id)}
                  >
                    {/* Unit ID */}
                    <div className="flex flex-col gap-0.5 min-w-[120px]">
                      <span style={{ fontSize: "0.625rem", fontWeight: 800, color: isActive ? "var(--color-primary)" : "var(--color-text-4)", textTransform: "uppercase", letterSpacing: "0.05em" }}>
                        Unit ID
                      </span>
                      <span style={{ fontSize: "0.875rem", fontWeight: 800, color: isActive ? "white" : "var(--color-text-0)", letterSpacing: "-0.01em", fontFamily: "var(--font-mono), monospace" }}>
                        {robot.id.replace("_", "-").toUpperCase()}
                      </span>
                    </div>
                    {/* Status */}
                    <div className="flex flex-col gap-0.5 min-w-[100px]">
                      <span style={{ fontSize: "0.625rem", fontWeight: 800, color: isActive ? "rgba(255,255,255,0.4)" : "var(--color-text-4)", textTransform: "uppercase", letterSpacing: "0.05em" }}>
                        Status
                      </span>
                      <StatusBadge status="online" />
                    </div>
                    {/* Battery */}
                    <div className="flex-1 flex items-center gap-4">
                      <div className="flex flex-col gap-0.5 flex-1">
                        <span style={{ fontSize: "0.625rem", fontWeight: 800, color: isActive ? "var(--color-primary)" : "var(--color-text-4)", textTransform: "uppercase", letterSpacing: "0.05em" }}>
                          Power
                        </span>
                        <BatteryIndicator level={robot.battery} />
                      </div>
                    </div>
                    {/* Coordinates */}
                    <div className="flex flex-col gap-0.5 min-w-[140px] text-right">
                      <span style={{ fontSize: "0.625rem", fontWeight: 800, color: isActive ? "var(--color-primary)" : "var(--color-text-4)", textTransform: "uppercase", letterSpacing: "0.05em" }}>
                        Coordinates
                      </span>
                      <span className="font-mono" style={{ fontSize: "0.75rem", color: isActive ? "rgba(255,255,255,0.8)" : "var(--color-text-2)", fontWeight: 600 }}>
                        {robot.x.toFixed(2)}, {robot.y.toFixed(2)}
                      </span>
                    </div>
                    {/* Arrow */}
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
  );
}

// ─── Page export ──────────────────────────────────────────────────────────────
export default function Home() {
  return (
    <Suspense
      fallback={
        <div className="w-full h-full flex items-center justify-center" style={{ background: "rgba(255,255,255,0.01)" }}>
          <div className="flex flex-col items-center gap-4">
            <div className="w-8 h-8 rounded-full border-2 border-zinc-800 border-t-[var(--color-primary)] animate-spin" />
            <span className="text-[10px] font-mono font-bold uppercase tracking-[0.2em] text-[var(--color-primary)] opacity-80">
              Synchronizing
            </span>
          </div>
        </div>
      }
    >
      <FleetDashboard />
    </Suspense>
  );
}
