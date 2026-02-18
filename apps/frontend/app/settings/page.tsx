"use client";

import DashboardLayout from "@/components/layout/DashboardLayout";
import { 
  Settings, 
  Shield, 
  Smartphone, 
  Globe, 
  Bell, 
  HardDrive,
  User,
  Zap,
  ChevronRight
} from "lucide-react";

export default function SettingsPage() {
  const sections = [
    {
      title: "System Parameters",
      icon: Globe,
      items: [
        { label: "DDS Domain ID", value: "0", type: "input" },
        { label: "Telemetry Socket Port", value: "4000", type: "input" },
        { label: "Discovery Server", value: "192.168.1.100", type: "input" },
      ]
    },
    {
      title: "Navigation Security",
      icon: Shield,
      items: [
        { label: "Max Velocity Limit", value: "0.5 m/s", type: "toggle" },
        { label: "Obstacle Inflation Radius", value: "0.55m", type: "input" },
        { label: "Collision Deadman Switch", value: "Active", type: "status" },
      ]
    }
  ];

  return (
    <DashboardLayout>
      <div className="flex flex-col h-full gap-8 max-w-4xl">
        {/* Header Section */}
        <div className="flex flex-col gap-1">
          <h1 className="text-2xl font-black tracking-tight text-foreground">
            Control Settings
          </h1>
          <p className="text-sm text-muted-foreground font-medium">
            Configure system-wide parameters and safety protocols
          </p>
        </div>

        <div className="grid grid-cols-12 gap-8">
           {/* Sidebar Navigation */}
           <div className="col-span-3">
              <div className="flex flex-col gap-1">
                {[
                  { label: "General", icon: Settings, active: true },
                  { label: "Robot Security", icon: Shield, active: false },
                  { label: "Devices", icon: Smartphone, active: false },
                  { label: "Notifications", icon: Bell, active: false },
                  { label: "Storage", icon: HardDrive, active: false },
                  { label: "Operator Profile", icon: User, active: false },
                ].map((item, i) => (
                  <button 
                    key={i}
                    className={`flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-200 ${
                      item.active ? "bg-black text-white shadow-lg shadow-black/10" : "text-zinc-500 hover:bg-zinc-50 hover:text-black"
                    }`}
                  >
                    <item.icon size={18} />
                    <span className="text-sm font-bold">{item.label}</span>
                  </button>
                ))}
              </div>
           </div>

           {/* Content Area */}
           <div className="col-span-9 flex flex-col gap-8">
              {sections.map((section, i) => (
                <div key={i} className="flex flex-col gap-4">
                   <div className="flex items-center gap-2 px-1">
                      <section.icon size={14} className="text-primary" />
                      <span className="text-[10px] uppercase font-black tracking-widest">{section.title}</span>
                   </div>
                   <div className="panel bg-white border border-border divide-y divide-zinc-100 shadow-sm">
                      {section.items.map((item, j) => (
                        <div key={j} className="p-5 flex items-center justify-between group hover:bg-zinc-50/50 transition-colors">
                           <div className="flex flex-col gap-0.5">
                              <span className="text-sm font-bold text-foreground">{item.label}</span>
                              <span className="text-[10px] text-zinc-400 font-bold uppercase tracking-wider">Protocol v1.0.4</span>
                           </div>
                           <div className="flex items-center gap-4">
                              <span className="font-mono text-sm font-bold text-zinc-500 bg-zinc-50 px-3 py-1.5 rounded-lg border border-zinc-100">
                                {item.value}
                              </span>
                              <button className="p-2 text-zinc-300 group-hover:text-black transition-colors">
                                 <ChevronRight size={18} />
                              </button>
                           </div>
                        </div>
                      ))}
                   </div>
                </div>
              ))}

              {/* Danger Zone */}
              <div className="mt-4 p-6 rounded-2xl bg-red-50/50 border border-red-100 flex items-center justify-between">
                 <div className="flex items-center gap-4">
                    <div className="p-3 rounded-xl bg-red-500 text-white shadow-lg shadow-red-500/20">
                       <Zap size={20} />
                    </div>
                    <div className="flex flex-col gap-0.5">
                       <span className="text-sm font-black text-red-900">Emergency Global Stop</span>
                       <span className="text-xs font-medium text-red-700/70">Kill all active namespaced nodes and purge discovery cache</span>
                    </div>
                 </div>
                 <button className="px-6 py-2.5 rounded-full bg-red-600 text-white text-xs font-black hover:bg-red-700 transition-colors shadow-lg shadow-red-600/20">
                    ACTIVATE KILLSWITCH
                 </button>
              </div>
           </div>
        </div>
      </div>
    </DashboardLayout>
  );
}
