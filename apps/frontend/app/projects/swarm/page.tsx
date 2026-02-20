"use client";

import { Grid, Share2, Users } from "lucide-react";

export default function SwarmProjectPage() {
  return (
    <div className="w-full h-full p-2 sm:p-4 md:p-6 lg:p-8 animate-in fade-in slide-in-from-bottom-2 duration-500 overflow-y-auto">
<div className="flex flex-col items-center justify-center h-full p-12 text-center space-y-8">
        <div className="p-6 rounded-full bg-zinc-100 mb-4">
          <Grid size={64} className="text-zinc-900" />
        </div>
        <h1 className="text-4xl font-black tracking-tighter text-zinc-900 uppercase">
          Project 4: The Hive
        </h1>
        <p className="max-w-2xl text-lg text-zinc-500 font-medium leading-relaxed">
          A distributed swarm of 20+ micro-agents exhibiting emergent behavior.
          <br />
          Powered by Rust Native Backend and WebAssembly simulation.
        </p>
        
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6 mt-8 w-full max-w-2xl">
          <div className="p-6 border border-zinc-200 rounded-2xl bg-white shadow-sm flex flex-col items-center gap-3">
            <Share2 size={24} className="text-zinc-400" />
            <span className="font-bold text-sm tracking-widest uppercase">Coordination</span>
            <span className="text-xs text-zinc-400 font-mono">Boids / Auction</span>
          </div>
          <div className="p-6 border border-zinc-200 rounded-2xl bg-white shadow-sm flex flex-col items-center gap-3">
             <Users size={24} className="text-zinc-400" />
             <span className="font-bold text-sm tracking-widest uppercase">Scalability</span>
             <span className="text-xs text-zinc-400 font-mono">100+ Agents</span>
          </div>
          <div className="p-6 border border-zinc-200 rounded-2xl bg-white shadow-sm flex flex-col items-center gap-3">
             <Grid size={24} className="text-zinc-400" />
             <span className="font-bold text-sm tracking-widest uppercase">Backend</span>
             <span className="text-xs text-zinc-400 font-mono">Rust / ECS</span>
          </div>
        </div>

        <div className="mt-12 px-6 py-2 rounded-full bg-zinc-100 text-zinc-500 font-bold text-xs uppercase tracking-widest border border-zinc-200">
           Status: Phase 4 Planned
        </div>
      </div>
    </div>
  );
}
