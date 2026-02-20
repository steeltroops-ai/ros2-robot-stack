"use client";

import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";
import { useRobotPanel } from "@/components/layout/panel-content/RobotPanelContent";
import dynamic from "next/dynamic";
import { use, useEffect, useCallback } from "react";

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
  const { robots, sendNavigationGoal, pathData, subscribeToRobot, unsubscribeFromRobot } =
    useFleetTelemetry();
  const robot = robots.find((r) => r.id === id);

  // Subscribe when page mounts
  useEffect(() => {
    subscribeToRobot(id);
    return () => unsubscribeFromRobot(id);
  }, [id, subscribeToRobot, unsubscribeFromRobot]);

  const handleNavigationGoal = useCallback(
    (x: number, y: number) => sendNavigationGoal(id, x, y),
    [id, sendNavigationGoal]
  );

  // ── Inject content into the shared right panel (only re-sets on robotId change) ──
  useRobotPanel({ robotId: id });

  return (
    <div className="w-full h-full relative rounded-2xl overflow-hidden border border-white/10 shadow-[0_0_40px_rgba(0,0,0,0.3)] bg-black/40">
      <RobotViewer
        robotId={id}
        pose={robot ? { x: robot.x, y: robot.y, theta: robot.theta } : { x: 0, y: 0, theta: 0 }}
        onNavigationGoal={handleNavigationGoal}
        pathData={pathData}
      />
    </div>
  );
}

