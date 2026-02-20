"use client";

import { useEffect } from "react";
import { useRightPanel } from "@/components/layout/RightPanelContext";
import { TeleopPanel } from "@/components/telemetry/TeleopPanel";
import { MapDisplay } from "@/components/map/MapDisplay";
import type { MapData, RobotState } from "@/hooks/useFleetTelemetry";

// ─── DataTile (shared micro-component) ───────────────────────────────────────
function DataTile({ label, value }: { label: string; value: string }) {
  return (
    <div
      className="p-3 rounded-lg border"
      style={{
        background: "rgba(255,255,255,0.04)",
        borderColor: "var(--color-border-0)",
      }}
    >
      <div
        className="uppercase tracking-widest mb-1"
        style={{ fontSize: "0.625rem", fontWeight: 700, color: "var(--color-text-3)", fontFamily: "var(--font-mono), monospace" }}
      >
        {label}
      </div>
      <div style={{ fontSize: "0.8125rem", fontWeight: 700, color: "var(--color-text-0)", fontFamily: "var(--font-mono), monospace" }}>
        {value}
      </div>
    </div>
  );
}

// ─── Section label ────────────────────────────────────────────────────────────
function SectionLabel({ children }: { children: string }) {
  return (
    <div
      className="uppercase tracking-widest pb-2"
      style={{
        fontSize: "0.625rem",
        fontWeight: 700,
        color: "var(--color-text-3)",
        borderBottom: "1px solid var(--color-border-0)",
        fontFamily: "var(--font-mono), monospace",
      }}
    >
      {children}
    </div>
  );
}

// ─── Robot page panel body ────────────────────────────────────────────────────
function RobotPanelBody({
  robotId,
  robot,
  mapData,
  sendControl,
}: {
  robotId: string;
  robot: { x: number; y: number; theta: number; battery: number } | undefined;
  mapData: MapData | null;
  sendControl: (id: string, lin: number, ang: number) => void;
}) {
  return (
    <div className="flex flex-col gap-5 h-full overflow-y-auto px-5 py-4">
      {/* Robot ID breadcrumb */}
      <div
        className="uppercase tracking-widest"
        style={{
          fontSize: "0.625rem",
          fontWeight: 700,
          color: "var(--color-text-3)",
          fontFamily: "var(--font-mono), monospace",
        }}
      >
        Unit /{" "}
        <span style={{ color: "var(--color-primary)" }}>
          {robotId.replace("_", "-").toUpperCase()}
        </span>
      </div>

      {/* Teleop */}
      <TeleopPanel onControl={(l, a) => sendControl(robotId, l, a)} />

      {/* Mini map */}
      <div className="flex flex-col gap-3">
        <SectionLabel>Area Map</SectionLabel>
        <div
          className="rounded-xl overflow-hidden border"
          style={{
            height: 160,
            borderColor: "var(--color-border-0)",
            background: "rgba(0,0,0,0.3)",
          }}
        >
          <MapDisplay
            mapData={mapData}
            robots={robot
              ? [{ id: robotId, x: robot.x, y: robot.y, theta: robot.theta, battery: robot.battery, status: "ONLINE" as const, lastSeen: Date.now() } satisfies RobotState]
              : []
            }
          />
        </div>
      </div>

      {/* System vitals */}
      <div className="flex flex-col gap-3">
        <SectionLabel>System Vitals</SectionLabel>
        <div className="grid grid-cols-2 gap-2">
          <DataTile label="Position X" value={robot?.x.toFixed(2) ?? "0.00"} />
          <DataTile label="Position Y" value={robot?.y.toFixed(2) ?? "0.00"} />
          <DataTile label="Heading"    value={`${(((robot?.theta ?? 0) * 180) / Math.PI).toFixed(0)}°`} />
          <DataTile label="Battery"    value={`${robot?.battery?.toFixed(0) ?? 0}%`} />
          <DataTile label="Status"     value={robot ? "LIVE" : "OFFLINE"} />
          <DataTile label="Network"    value={robot ? "32 ms" : "N/A"} />
        </div>
      </div>
    </div>
  );
}

// ─── Hook ─────────────────────────────────────────────────────────────────────
/** Register the robot page content in the shared right panel. */
export function useRobotPanel({
  robotId,
  robot,
  mapData,
  sendControl,
}: {
  robotId: string;
  robot: { x: number; y: number; theta: number; battery: number } | undefined;
  mapData: MapData | null;
  sendControl: (id: string, lin: number, ang: number) => void;
}) {
  const { setPanel } = useRightPanel();

  useEffect(() => {
    setPanel({
      title: "Control Station",
      badge: robot ? "LIVE" : "OFFLINE",
      badgeVariant: robot ? "green" : "grey",
      content: (
        <RobotPanelBody
          robotId={robotId}
          robot={robot}
          mapData={mapData}
          sendControl={sendControl}
        />
      ),
    });
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [robotId, robot?.x, robot?.y, robot?.theta, robot?.battery, mapData]);
}
