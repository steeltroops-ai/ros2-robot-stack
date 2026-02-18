"use client";

import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";
import { TeleopPanel } from "@/components/telemetry/TeleopPanel";
import { Compass, MapPin, Activity } from "lucide-react";
import { MapDisplay } from "@/components/map/MapDisplay";

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
          width: "100%",
          height: 6,
          borderRadius: 3,
          background: "var(--color-surface-2)",
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
        style={{ fontSize: "0.8125rem", color: "var(--color-text-1)", minWidth: "3ch" }}
      >
        {level.toFixed(0)}%
      </span>
    </div>
  );
}

function TelemetryRow({
  label,
  value,
  icon,
}: {
  label: string;
  value: string;
  icon: React.ReactNode;
}) {
  return (
    <div
      className="flex items-center justify-between"
      style={{
        padding: "0.75rem 0",
        borderBottom: "1px solid var(--color-border-0)",
      }}
    >
      <div className="flex items-center gap-2.5">
        <span style={{ color: "var(--color-text-4)" }}>{icon}</span>
        <span
          style={{
            fontSize: "0.8125rem",
            color: "var(--color-text-3)",
          }}
        >
          {label}
        </span>
      </div>
      <span
        className="data-mono"
        style={{ fontSize: "0.8125rem", color: "var(--color-text-1)" }}
      >
        {value}
      </span>
    </div>
  );
}

export default function RobotControlPanel({ robotId }: { robotId: string }) {
  const { robots, sendControl, mapData } = useFleetTelemetry();
  const robot = robots.find((r) => r.id === robotId);

  return (
    <div className="flex flex-col h-full gap-4 overflow-y-auto" style={{ padding: "0 1.5rem 1.5rem" }}>
      {/* Robot Status Header */}


      {/* Teleop Control */}
      <div>
        <div
          style={{
            fontSize: "0.6875rem",
            fontWeight: 500,
            textTransform: "uppercase",
            letterSpacing: "0.06em",
            color: "var(--color-text-4)",
            marginBottom: "0.75rem",
          }}
        >
          Manual Override
        </div>
        <TeleopPanel onControl={(l, a) => sendControl(robotId, l, a)} />
      </div>

      {/* Live Map */}
      <div>
        <div
          style={{
            fontSize: "0.6875rem",
            fontWeight: 500,
            textTransform: "uppercase",
            letterSpacing: "0.06em",
            color: "var(--color-text-4)",
            marginBottom: "0.75rem",
          }}
        >
          Slam Map
        </div>
        <div className="rounded-lg overflow-hidden border border-[var(--color-border-0)]" style={{ height: 160, background: "var(--color-surface-2)" }}>
           <MapDisplay
              mapData={mapData}
              robots={robot ? [robot] : []}
            />
        </div>
      </div>

      {/* Live Telemetry */}
      <div>
        <div
          style={{
            fontSize: "0.6875rem",
            fontWeight: 500,
            textTransform: "uppercase",
            letterSpacing: "0.06em",
            color: "var(--color-text-4)",
            marginBottom: "0.5rem",
          }}
        >
          Vitals
        </div>
        
        {/* Battery */}
        <div className="mb-4">
             {robot ? (
                  <BatteryIndicator level={robot.battery} />
                ) : (
                  <span style={{ fontSize: "0.8125rem", color: "var(--color-text-4)" }}>No signal</span>
            )}
        </div>

        {/* Data Points */}
        <TelemetryRow
          label="Position X"
          value={`${robot?.x.toFixed(3) || "0.000"} m`}
          icon={<MapPin size={14} />}
        />
        <TelemetryRow
          label="Position Y"
          value={`${robot?.y.toFixed(3) || "0.000"} m`}
          icon={<MapPin size={14} />}
        />
        <TelemetryRow
          label="Heading"
          value={`${robot ? (robot.theta * (180 / Math.PI)).toFixed(1) : "0.0"}°`}
          icon={<Compass size={14} />}
        />
        <TelemetryRow
          label="Comms"
          value={robot ? "Active" : "Offline"}
          icon={<Activity size={14} />}
        />
      </div>
    </div>
  );
}
