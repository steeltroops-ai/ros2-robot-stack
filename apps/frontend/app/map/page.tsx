"use client";

import { MapDisplay } from "@/components/map/MapDisplay";
import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";
import { Map as MapIcon, Layers, Maximize2, RotateCcw } from "lucide-react";

import { useState } from "react";

export default function MapPage() {
  const { mapData, robots } = useFleetTelemetry();
  const [selectedId, setSelectedId] = useState<string | null>(null);

  return (
    <div className="w-full h-full p-2 sm:p-4 md:p-6 lg:p-8 animate-in fade-in duration-200 overflow-y-auto">
<div className="flex flex-col h-full gap-6">
        {/* Header Section */}
        <div className="flex items-center justify-between">
          <div className="flex flex-col gap-1">
            <h1 className="text-2xl font-black tracking-tight text-foreground">
              Lab Network Map
            </h1>
            <p className="text-sm text-muted-foreground font-medium">
              Live spatial view of all active agents and the test environment
            </p>
          </div>
          
          <div className="flex items-center gap-2">
            <button className="flex items-center gap-2 px-4 py-2 rounded-full border border-border bg-white text-xs font-bold hover:bg-zinc-50 transition-colors shadow-sm">
              <RotateCcw size={14} />
              Reset View
            </button>
            <button className="flex items-center gap-2 px-4 py-2 rounded-full bg-primary text-primary-foreground text-xs font-bold hover:opacity-90 transition-opacity shadow-sm">
              <Maximize2 size={14} />
              Full Screen
            </button>
          </div>
        </div>

        {/* Map Container */}
        <div className="flex-1 relative panel bg-white border border-border overflow-hidden min-h-[500px]">
          {/* Map Controls Overlay */}
          <div className="absolute top-4 right-4 z-10 flex flex-col gap-2">
             <div className="flex flex-col rounded-xl border border-border bg-white/80 backdrop-blur shadow-sm overflow-hidden">
                <button className="p-3 hover:bg-zinc-100 transition-colors border-b border-border">
                  <Layers size={18} className="text-zinc-600" />
                </button>
                <button className="p-3 hover:bg-zinc-100 transition-colors">
                  <MapIcon size={18} className="text-zinc-600" />
                </button>
             </div>
          </div>

          <div className="w-full h-full flex items-center justify-center p-8 bg-[#f8f8fb]">
             <div className="w-full h-full max-w-4xl shadow-2xl rounded-sm overflow-hidden border border-black/5 bg-white relative">
                <MapDisplay 
                  mapData={mapData} 
                  robots={robots} 
                  onSelectRobot={setSelectedId}
                  selectedId={selectedId}
                />
                
                {selectedId && (
                  <div className="absolute top-4 left-4 p-4 rounded-xl border border-border bg-white/90 backdrop-blur shadow-xl animate-in fade-in slide-in-from-left-4">
                    <div className="flex flex-col gap-1">
                      <span className="text-[10px] font-black uppercase tracking-widest text-primary">Selected Node</span>
                      <span className="text-xl font-black text-zinc-900">{selectedId.replace(/robot_/i, "Node-").toUpperCase()}</span>
                      <div className="mt-2 flex items-center gap-4">
                         <div className="flex flex-col">
                            <span className="text-[8px] font-bold text-zinc-400">STATUS</span>
                            <span className="text-[10px] font-black text-green-600">ONLINE</span>
                         </div>
                         <div className="flex flex-col text-right">
                            <span className="text-[8px] font-bold text-zinc-400">BATTERY</span>
                            <span className="text-[10px] font-black text-zinc-900">
                              {robots.find(r => r.id === selectedId)?.battery.toFixed(0)}%
                            </span>
                         </div>
                      </div>
                    </div>
                  </div>
                )}
             </div>
          </div>
          
          {/* Map Footer / Metadata */}
          <div className="absolute bottom-4 left-4 z-10">
            <div className="px-3 py-1.5 rounded-full border border-border bg-white/80 backdrop-blur shadow-sm flex items-center gap-3">
              <div className="flex items-center gap-1.5">
                <div className="w-2 h-2 rounded-full bg-green-500 animate-pulse" />
                <span className="text-[10px] font-black uppercase tracking-widest text-zinc-900">Live Feed</span>
              </div>
              <div className="w-px h-3 bg-zinc-200" />
              <span className="text-[10px] font-mono font-bold text-zinc-500">
                {mapData?.info?.resolution || "0.00"} M/PX
              </span>
            </div>
          </div>
        </div>
        
        {/* Footnote */}
        <div className="flex items-center gap-2 px-1 opacity-40">
           <MapIcon size={12} />
           <span className="text-[10px] uppercase font-black tracking-widest text-zinc-500">MAP SYSTEM OPERATIONAL</span>
        </div>
      </div>
    </div>
  );
}
