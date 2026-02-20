"use client";

import { useEffect } from "react";
import { useRightPanel } from "@/components/layout/RightPanelContext";
import { TeleopPanel } from "@/components/telemetry/TeleopPanel";
import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";
import dynamic from "next/dynamic";
import { CircleStop } from "lucide-react";

const RobotViewer = dynamic(
  () => import("@/components/scene/RobotViewer"),
  {
    ssr: false,
    loading: () => (
      <div className="w-full h-full flex items-center justify-center">
        <div className="w-6 h-6 rounded-full border-2 border-[var(--color-primary)]/20 border-t-[var(--color-primary)] animate-spin" />
      </div>
    ),
  }
);

// ─── Data tile used in the vitals grid ───────────────────────────────────────
function DataTile({ label, value }: { label: string; value: string | undefined }) {
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
        {value ?? "—"}
      </div>
    </div>
  );
}

// ─── Content rendered inside the right panel for the dashboard ───────────────
// Reads live telemetry internally so the parent doesn't need to re-set panel content
// on every position tick.
function DashboardPanelBody({
  selectedRobotId,
  onDeselect,
}: {
  selectedRobotId: string;
  onDeselect: () => void;
}) {
  const { robots, sendControl } = useFleetTelemetry();
  const selectedData = robots.find((r) => r.id === selectedRobotId);

  return (
    <div className="flex flex-col h-full overflow-hidden">
      {/* Deselect button row */}
      <div
        className="flex items-center justify-between px-5 py-3 flex-shrink-0"
        style={{ borderBottom: "1px solid var(--color-border-0)" }}
      >
        <span
          style={{
            fontSize: "0.75rem",
            fontWeight: 700,
            color: "var(--color-text-2)",
            fontFamily: "var(--font-mono), monospace",
            letterSpacing: "0.06em",
          }}
        >
          {selectedRobotId.replace(/robot_/i, "Node-").toUpperCase()}
        </span>
        <button
          onClick={onDeselect}
          className="flex items-center gap-1.5 px-3 py-1 rounded-full transition-all hover:bg-white/10"
          style={{
            fontSize: "0.6875rem",
            fontWeight: 600,
            color: "var(--color-text-3)",
            border: "1px solid var(--color-border-0)",
          }}
        >
          <CircleStop size={12} />
          Close
        </button>
      </div>

      {/* 3-D Viewport */}
      <div
        className="flex-shrink-0 relative"
        style={{ height: 200, background: "rgba(0,0,0,0.4)", borderBottom: "1px solid var(--color-border-0)" }}
      >
        <RobotViewer
          robotId={selectedRobotId}
          pose={{
            x: selectedData?.x ?? 0,
            y: selectedData?.y ?? 0,
            theta: selectedData?.theta ?? 0,
          }}
        />
      </div>

      {/* Scrollable controls + vitals */}
      <div className="flex-1 overflow-y-auto px-5 py-4 flex flex-col gap-5">
        {/* Teleop */}
        <TeleopPanel
          onControl={(l, a) => sendControl(selectedRobotId, l, a)}
        />

        {/* Live Vitals */}
        <div className="flex flex-col gap-3">
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
            Live Vitals
          </div>
          <div className="grid grid-cols-2 gap-2">
            <DataTile label="Position X" value={selectedData?.x.toFixed(2)} />
            <DataTile label="Position Y" value={selectedData?.y.toFixed(2)} />
            <DataTile
              label="Heading"
              value={`${(((selectedData?.theta ?? 0) * 180) / Math.PI).toFixed(0)}°`}
            />
            <DataTile label="Battery" value={`${selectedData?.battery?.toFixed(0) ?? 0}%`} />
          </div>
        </div>
      </div>
    </div>
  );
}

// ─── No-robot-selected state ─────────────────────────────────────────────────
// Reads live telemetry internally to avoid parent re-setting panel on every tick
function DashboardIdleBody() {
  const { robots, isConnected } = useFleetTelemetry();
  const robotCount = robots.length;
  const avgBattery = robotCount > 0
    ? Math.round(robots.reduce((a, r) => a + r.battery, 0) / robotCount)
    : 0;

  return (
    <div className="flex flex-col gap-5 px-5 py-4">
      <p style={{ fontSize: "0.75rem", color: "var(--color-text-3)", lineHeight: 1.6 }}>
        Select a node from the agent roster to control it and view live telemetry here.
      </p>

      <div className="flex flex-col gap-2">
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
          Agent Summary
        </div>
        <div className="grid grid-cols-2 gap-2">
          <DataTile label="Online Nodes"     value={String(robotCount)} />
          <DataTile label="Avg Battery"      value={`${avgBattery}%`} />
          <DataTile label="Network"          value={isConnected ? "32 ms" : "Offline"} />
          <DataTile label="Active Missions"  value="0" />
        </div>
      </div>
    </div>
  );
}

// ─── Hook — call this from the dashboard page ─────────────────────────────────
/**
 * Registers dashboard content in the shared right panel.
 * ONLY re-sets panel JSX when selection changes — NOT on every telemetry tick.
 * The panel body components read telemetry internally.
 */
export function useDashboardPanel({
  selectedRobotId,
  onDeselect,
}: {
  selectedRobotId: string | null;
  selectedData?: { x: number; y: number; theta: number; battery: number } | undefined;
  robotCount?: number;
  avgBattery?: number;
  isConnected?: boolean;
  sendControl?: (id: string, lin: number, ang: number) => void;
  onDeselect: () => void;
}) {
  const { setPanel } = useRightPanel();

  useEffect(() => {
    if (selectedRobotId) {
      setPanel({
        title: "Control Station",
        badge: "LIVE",
        badgeVariant: "green",
        content: (
          <DashboardPanelBody
            selectedRobotId={selectedRobotId}
            onDeselect={onDeselect}
          />
        ),
      });
    } else {
      setPanel({
        title: "Agent Network",
        badge: "LIVE",
        badgeVariant: "green",
        content: <DashboardIdleBody />,
      });
    }
  // Only re-create panel content when the selection itself changes
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedRobotId]);
}
