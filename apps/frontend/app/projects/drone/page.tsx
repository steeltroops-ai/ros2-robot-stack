"use client";

import { Plane, Map, Wifi } from "lucide-react";

export default function DroneProjectPage() {
  return (
    <div className="w-full h-full p-2 sm:p-4 md:p-6 lg:p-8 animate-in fade-in slide-in-from-bottom-2 duration-500 overflow-y-auto">
<div className="flex flex-col items-center justify-center h-full p-12 text-center space-y-8">
        <div className="p-6 rounded-full bg-zinc-100 mb-4">
          <Plane size={64} className="text-zinc-900" />
        </div>
        <h1 className="text-4xl font-black tracking-tighter text-zinc-900 uppercase">
          Project 3: The Explorer
        </h1>
        <p className="max-w-2xl text-lg text-zinc-500 font-medium leading-relaxed">
          An indoor quadcopter for autonomous volumetric mapping (NeRF).
          <br />
          Built with PX4, ROS 2, and Visual Odometry.
        </p>
        
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6 mt-8 w-full max-w-2xl">
          <div className="p-6 border border-zinc-200 rounded-2xl bg-white shadow-sm flex flex-col items-center gap-3">
            <Map size={24} className="text-zinc-400" />
            <span className="font-bold text-sm tracking-widest uppercase">Mapping</span>
            <span className="text-xs text-zinc-400 font-mono">3D NeRF / Voxel</span>
          </div>
          <div className="p-6 border border-zinc-200 rounded-2xl bg-white shadow-sm flex flex-col items-center gap-3">
             <Wifi size={24} className="text-zinc-400" />
             <span className="font-bold text-sm tracking-widest uppercase">Protocol</span>
             <span className="text-xs text-zinc-400 font-mono">MAVLink / DDS</span>
          </div>
          <div className="p-6 border border-zinc-200 rounded-2xl bg-white shadow-sm flex flex-col items-center gap-3">
             <Plane size={24} className="text-zinc-400" />
             <span className="font-bold text-sm tracking-widest uppercase">Autonomy</span>
             <span className="text-xs text-zinc-400 font-mono">PX4 Flight Core</span>
          </div>
        </div>

        <div className="mt-12 px-6 py-2 rounded-full bg-zinc-100 text-zinc-500 font-bold text-xs uppercase tracking-widest border border-zinc-200">
           Status: Phase 3 Planned
        </div>
      </div>
    </div>
  );
}
