"use client";

import DashboardLayout from "@/components/layout/DashboardLayout";
import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";
import { 
  Activity, 
  Cpu, 
  Database, 
  Wifi, 
  AlertTriangle, 
  CheckCircle2,
  BarChart3,
  Clock,
  Terminal
} from "lucide-react";

export default function DiagnosticsPage() {
  const { robots, isConnected } = useFleetTelemetry();

  const mockLogs = [
    { time: "02:50:11", node: "/robot_1/nav2", level: "INFO", msg: "Reached waypoint [3]" },
    { time: "02:50:15", node: "/robot_2/slam", level: "WARN", msg: "Feature tracking degraded" },
    { time: "02:51:02", node: "/system/bridge", level: "INFO", msg: "HEARTBEAT active" },
    { time: "02:52:45", node: "/robot_1/odom", level: "ERROR", msg: "Sensor drift exceeded tolerance" },
  ];

  return (
    <DashboardLayout>
      <div className="flex flex-col h-full gap-6">
        {/* Header Section */}
        <div className="flex flex-col gap-1">
          <h1 className="text-2xl font-black tracking-tight text-foreground">
            System Diagnostics
          </h1>
          <p className="text-sm text-muted-foreground font-medium">
            Deep-level telemetry probes and node health monitoring
          </p>
        </div>

        {/* Global Stats Grid */}
        <div className="grid grid-cols-4 gap-4">
           {[
             { label: "DDS Latency", value: "14ms", icon: Wifi, color: "text-green-500" },
             { label: "CPU Load", value: "24.5%", icon: Cpu, color: "text-blue-500" },
             { label: "Shared Memory", value: "1.2 GB", icon: Database, color: "text-purple-500" },
             { label: "Node Count", value: "48 Active", icon: Activity, color: "text-orange-500" },
           ].map((stat, i) => (
             <div key={i} className="panel p-5 bg-white border border-border flex items-center gap-4">
                <div className={`p-2.5 rounded-xl bg-zinc-50 border border-border ${stat.color}`}>
                   <stat.icon size={20} />
                </div>
                <div className="flex flex-col gap-0.5">
                   <span className="text-[10px] uppercase font-black tracking-widest text-zinc-400">{stat.label}</span>
                   <span className="text-lg font-black text-foreground">{stat.value}</span>
                </div>
             </div>
           ))}
        </div>

        {/* Detailed Diagnostics Section */}
        <div className="grid grid-cols-3 gap-6 flex-1 min-h-0">
           {/* Node Health */}
           <div className="col-span-1 flex flex-col gap-3">
              <div className="flex items-center gap-2 px-1">
                 <CheckCircle2 size={14} className="text-primary" />
                 <span className="text-[10px] uppercase font-black tracking-widest">Active Node Cluster</span>
              </div>
              <div className="panel flex-1 bg-white border border-border overflow-y-auto p-4 space-y-4">
                 {robots.map((robot) => (
                   <div key={robot.id} className="p-4 rounded-xl border border-zinc-100 bg-zinc-50/50 flex flex-col gap-3">
                      <div className="flex items-center justify-between">
                         <span className="text-xs font-black">{robot.id.toUpperCase()}</span>
                         <div className="px-2 py-0.5 rounded-full bg-green-100 text-[9px] font-black text-green-700">HEARTBEAT OK</div>
                      </div>
                      <div className="grid grid-cols-2 gap-2">
                         <div className="flex flex-col gap-0.5">
                            <span className="text-[9px] text-zinc-400 uppercase font-bold">SLAM</span>
                            <div className="h-1 w-full bg-zinc-200 rounded-full overflow-hidden">
                               <div className="h-full bg-blue-500 w-[80%]" />
                            </div>
                         </div>
                         <div className="flex flex-col gap-0.5">
                            <span className="text-[9px] text-zinc-400 uppercase font-bold">CONTROL</span>
                            <div className="h-1 w-full bg-zinc-200 rounded-full overflow-hidden">
                               <div className="h-full bg-green-500 w-[95%]" />
                            </div>
                         </div>
                      </div>
                   </div>
                 ))}
              </div>
           </div>

           {/* Console / Logs */}
           <div className="col-span-2 flex flex-col gap-3">
              <div className="flex items-center gap-2 px-1">
                 <Terminal size={14} className="text-primary" />
                 <span className="text-[10px] uppercase font-black tracking-widest">Global Telemetry Log</span>
              </div>
              <div className="panel flex-1 bg-zinc-900 border border-zinc-800 flex flex-col overflow-hidden">
                 <div className="flex items-center gap-2 px-4 py-2 border-b border-zinc-800 bg-zinc-900/50">
                    <div className="w-2.5 h-2.5 rounded-full bg-red-400/20 flex items-center justify-center">
                       <div className="w-1 h-1 rounded-full bg-red-400" />
                    </div>
                    <span className="text-[10px] font-mono text-zinc-500 font-bold uppercase tracking-tight">System Out - /dev/null</span>
                 </div>
                 <div className="flex-1 overflow-y-auto p-4 font-mono text-xs space-y-2">
                    {mockLogs.map((log, i) => (
                      <div key={i} className="flex gap-4 border-l-2 border-zinc-800 pl-4 py-1 hover:bg-white/5 transition-colors">
                         <span className="text-zinc-500 shrink-0">[{log.time}]</span>
                         <span className={`font-black shrink-0 w-12 ${
                           log.level === 'ERROR' ? 'text-red-400' : 
                           log.level === 'WARN' ? 'text-yellow-400' : 'text-zinc-400'
                         }`}>{log.level}</span>
                         <span className="text-zinc-400 shrink-0 font-bold">[{log.node}]</span>
                         <span className="text-zinc-300 truncate">{log.msg}</span>
                      </div>
                    ))}
                    <div className="animate-pulse text-green-400 mt-4">_ Awaiting next broadcast...</div>
                 </div>
              </div>
           </div>
        </div>
      </div>
    </DashboardLayout>
  );
}
