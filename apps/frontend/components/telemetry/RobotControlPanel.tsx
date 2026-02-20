"use client";

import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";
import { TeleopPanel } from "@/components/telemetry/TeleopPanel";
import { Compass, MapPin, Activity } from "lucide-react";
import { MapDisplay } from "@/components/map/MapDisplay";

// Telemetry layout modernized via grid

export default function RobotControlPanel({ robotId }: { robotId: string }) {
  const { robots, sendControl, mapData } = useFleetTelemetry();
  const robot = robots.find((r) => r.id === robotId);

  return (
    <div className="flex flex-col h-full gap-4 overflow-y-auto" style={{ padding: "0 1.5rem 1.5rem" }}>
      {/* Robot Status Header */}


      {/* Teleop Control */}
      <div>
        {/* TeleopPanel provides its own beautiful glassmorphic header now */}
        <TeleopPanel onControl={(l, a) => sendControl(robotId, l, a)} />
      </div>

      {/* Live Map */}
      <div className="flex flex-col gap-3 mt-2">
        <div className="flex pb-2 border-b border-white/10">
          <span className="text-[11px] font-inter font-bold uppercase tracking-widest text-zinc-400">Area Map</span>
        </div>
        <div className="rounded-xl overflow-hidden border border-white/10 shadow-[0_0_20px_rgba(0,0,0,0.3)] bg-black/40 backdrop-blur-md" style={{ height: 160 }}>
           <MapDisplay
              mapData={mapData}
              robots={robot ? [robot] : []}
            />
        </div>
      </div>

      {/* Live Telemetry */}
      <div className="flex flex-col gap-4 mt-2">
        <div className="flex pb-2 border-b border-white/10">
          <span className="text-[11px] font-inter font-bold uppercase tracking-widest text-zinc-400">System Vitals</span>
        </div>
        
        {/* Data Grid */}
        <div className="grid grid-cols-2 gap-3">
            {[
              { label: "Position X", value: robot?.x.toFixed(2) ?? "0.00" },
              { label: "Position Y", value: robot?.y.toFixed(2) ?? "0.00" },
              { label: "Heading", value: `${((robot?.theta ?? 0) * 180 / Math.PI).toFixed(0)}°` },
              { label: "Battery", value: `${robot?.battery?.toFixed(0) ?? 0}%` },
              { label: "Status", value: robot ? "LIVE" : "OFFLINE" },
              { label: "Network", value: robot ? "32ms" : "N/A" }
           ].map((m, i) => (
              <div key={i} className="bg-white/5 p-3 rounded-lg border border-white/10 shadow-[inset_0_0_15px_rgba(255,255,255,0.02)] backdrop-blur-md">
                 <div className="text-[9px] font-inter font-medium tracking-widest text-zinc-400 mb-1 uppercase">{m.label}</div>
                 <div className="text-[13px] font-inter font-bold text-white shadow-sm">{m.value}</div>
              </div>
           ))}
        </div>
      </div>
    </div>
  );
}
