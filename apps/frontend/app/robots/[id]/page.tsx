"use client";

import DashboardLayout from "@/components/layout/DashboardLayout";
import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";
import RobotControlPanel from "@/components/telemetry/RobotControlPanel";
import dynamic from "next/dynamic";
import { use } from "react";

const RobotViewer = dynamic(
  () => import("@/components/scene/RobotViewer"),
  { ssr: false },
);



export default function RobotPage({
  params,
}: {
  params: Promise<{ id: string }>;
}) {
  const { id } = use(params);
  const { robots } = useFleetTelemetry();
  const robot = robots.find((r) => r.id === id);

  return (
    <DashboardLayout 
      noPadding 
      rightPanel={
        <div className="flex flex-col h-full">
          {/* Panel Header - Integrated with Shell Header */}
          <div 
            className="flex items-center justify-between px-6"
            style={{
              height: 64, // Exact match to Global Header
              flexShrink: 0
            }}
          >
            <span style={{ fontSize: "1rem", fontWeight: 600, color: "var(--color-text-0)", letterSpacing: "-0.01em" }}>
              Control Station
            </span>
            <div className="flex items-center gap-2 px-2 py-1 rounded-full" style={{ background: "rgba(34, 197, 94, 0.1)" }}>
               <div className="w-1.5 h-1.5 rounded-full bg-green-500 animate-pulse" />
               <span style={{ fontSize: "0.6875rem", fontWeight: 600, color: "var(--color-success)" }}>LIVE</span>
            </div>
          </div>
          <div className="flex-1 overflow-hidden">
            <RobotControlPanel robotId={id} />
          </div>
        </div>
      }
    >
      <div className="w-full h-full relative">
        <RobotViewer
          robotId={id}
          pose={
            robot
              ? { x: robot.x, y: robot.y, theta: robot.theta }
              : { x: 0, y: 0, theta: 0 }
          }
        />
      </div>
    </DashboardLayout>
  );
}
