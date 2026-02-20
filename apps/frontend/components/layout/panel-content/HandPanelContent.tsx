"use client";

import { useEffect, useState } from "react";
import { useRightPanel } from "@/components/layout/RightPanelContext";

const JOINT_HIERARCHY = [
  { label: "SHOULDER", dof: "3 DOF (Yaw / Pitch / Roll)", color: "var(--color-primary)" },
  { label: "ELBOW",    dof: "1 DOF (Pitch)",               color: "var(--color-primary)" },
  { label: "FOREARM",  dof: "1 DOF (Pronation)",           color: "var(--color-primary)" },
  { label: "WRIST",    dof: "2 DOF (Pitch / Yaw)",         color: "var(--color-primary)" },
  { label: "THUMB",    dof: "3 DOF (CMC / MCP / IP)",      color: "#f59e0b" },
  { label: "INDEX",    dof: "3 DOF (MCP / PIP / DIP)",     color: "#f59e0b" },
  { label: "MIDDLE",   dof: "3 DOF (MCP / PIP / DIP)",     color: "#f59e0b" },
  { label: "RING",     dof: "3 DOF (MCP / PIP / DIP)",     color: "#f59e0b" },
  { label: "PINKY",    dof: "3 DOF (MCP / PIP / DIP)",     color: "#f59e0b" },
];

function ControlsTab() {
  return (
    <div className="flex flex-col gap-5 font-mono text-xs text-[var(--color-text-2)] bg-transparent">
      
      {/* Kinematic Override */}
      <div>
        <div className="flex items-center justify-between uppercase tracking-widest text-[var(--color-primary)] font-semibold mb-2 text-[0.625rem] border-b border-[var(--color-primary)]/20 pb-1">
          <span>Kinematic Override</span>
          <span className="text-[var(--color-primary)]/70">Lat: &lt;2ms</span>
        </div>
        <div className="flex flex-col gap-2.5">
          <div className="flex flex-col gap-1.5">
            <label className="text-[var(--color-text-3)] font-semibold tracking-wider text-[0.625rem] uppercase">Absolute Position XYZ (m)</label>
            <div className="flex gap-2">
              <input type="text" placeholder="X: 0.0" className="bg-white/5 border border-white/10 rounded px-2 py-1.5 flex-1 w-0 text-[var(--color-text-0)] placeholder-[var(--color-text-4)] outline-none focus:border-[var(--color-primary)] transition-colors text-[0.6875rem]" />
              <input type="text" placeholder="Y: 0.0" className="bg-white/5 border border-white/10 rounded px-2 py-1.5 flex-1 w-0 text-[var(--color-text-0)] placeholder-[var(--color-text-4)] outline-none focus:border-[var(--color-primary)] transition-colors text-[0.6875rem]" />
              <input type="text" placeholder="Z: 0.0" className="bg-white/5 border border-white/10 rounded px-2 py-1.5 flex-1 w-0 text-[var(--color-text-0)] placeholder-[var(--color-text-4)] outline-none focus:border-[var(--color-primary)] transition-colors text-[0.6875rem]" />
            </div>
          </div>
          <button 
            className="transition-colors rounded py-1.5 px-3 font-semibold uppercase tracking-wider text-[0.6875rem] hover:bg-[rgba(16,185,129,0.15)]"
            style={{ backgroundColor: "rgba(16, 185, 129, 0.08)", color: "var(--color-primary)", border: "1px solid rgba(16, 185, 129, 0.2)" }}
          >
            Apply Force Transform
          </button>
        </div>
      </div>

      {/* Execution Arrays */}
      <div>
        <div className="uppercase tracking-widest text-[var(--color-primary)] font-semibold mb-2 text-[0.625rem] border-b border-[var(--color-primary)]/20 pb-1 mt-1">
          Execution Arrays
        </div>
        <div className="grid grid-cols-2 gap-2">
          {["Wave", "Seq Curl", "Fist Clench", "Spread", "Piano", "Pinch"].map((gesture, i) => (
            <button key={i} className="bg-white/5 hover:bg-white/10 border border-white/10 transition-colors rounded py-1.5 px-2 flex items-center justify-center text-[var(--color-text-2)] hover:text-[var(--color-text-0)] font-medium text-[0.6875rem]">
              {gesture}
            </button>
          ))}
        </div>
      </div>
      
      {/* Hardware Calibrations */}
      <div>
        <div className="uppercase tracking-widest text-[var(--color-primary)] font-semibold mb-2 text-[0.625rem] border-b border-[var(--color-primary)]/20 pb-1 mt-1">
          Hardware Calibrations
        </div>
        <div className="flex flex-col gap-2">
            <div className="flex items-center justify-between p-2 rounded bg-white/5 border border-white/10">
              <span className="text-[var(--color-text-2)] font-medium text-[0.6875rem]">Torque Limiters</span>
              <button className="bg-white/5 hover:bg-white/10 border border-white/10 transition-colors px-3 py-1 rounded-sm uppercase tracking-widest text-[0.625rem] font-semibold text-[var(--color-text-3)] hover:text-[var(--color-text-0)]">Bypass</button>
            </div>
            <div className="flex items-center justify-between p-2 rounded bg-white/5 border border-white/10">
              <span className="text-[var(--color-text-2)] font-medium text-[0.6875rem]">Absolute Encoders</span>
              <button className="bg-white/5 hover:bg-white/10 border border-white/10 transition-colors px-3 py-1 rounded-sm uppercase tracking-widest text-[0.625rem] font-semibold text-[var(--color-text-3)] hover:text-[var(--color-text-0)]">Reset</button>
            </div>
        </div>
      </div>
    </div>
  );
}

function SpecsTab() {
  return (
    <div className="flex flex-col gap-6 font-mono text-xs text-[var(--color-text-2)]">
      
      {/* Removed Redundant "Specifications" Heading */}
      <div className="grid grid-cols-1 gap-y-4 pt-1">
        <div className="flex flex-col gap-0.5">
          <span className="text-[var(--color-primary)] uppercase tracking-widest text-[0.625rem] font-semibold">Degrees of Freedom</span>
          <span className="text-[var(--color-text-0)] font-semibold text-[0.75rem]">24 DOF Total Array</span>
          <span className="text-[var(--color-text-3)] text-[0.6875rem] leading-relaxed">7 arm joints + 17 hand joints controlled simultaneously. Allows full dexterous manipulation.</span>
        </div>
        <div className="flex flex-col gap-0.5">
          <span className="text-[var(--color-primary)] uppercase tracking-widest text-[0.625rem] font-semibold">Actuators</span>
          <span className="text-[var(--color-text-0)] font-semibold text-[0.75rem]">Brushless Tendon-driven</span>
          <span className="text-[var(--color-text-3)] text-[0.6875rem] leading-relaxed">High-torque density brushless DC motors with a complex tendon routing mechanism.</span>
        </div>
        <div className="flex flex-col gap-0.5">
          <span className="text-[var(--color-primary)] uppercase tracking-widest text-[0.625rem] font-semibold">Force Sensing</span>
          <span className="text-[var(--color-text-0)] font-semibold text-[0.75rem]">9-point Tactile Array</span>
          <span className="text-[var(--color-text-3)] text-[0.6875rem] leading-relaxed">Pressure arrays distributed across the palm and fingertips for precise grasp control.</span>
        </div>
        <div className="flex flex-col gap-0.5">
          <span className="text-[var(--color-primary)] uppercase tracking-widest text-[0.625rem] font-semibold">Onboard Compute & Policy</span>
          <span className="text-[var(--color-text-0)] font-semibold text-[0.75rem]">Jetson Orin NX (PPO Policy)</span>
          <span className="text-[var(--color-text-3)] text-[0.6875rem] leading-relaxed">Reinforcement Learning (PPO) via Isaac Gym Sim2Real inference, delivering &lt;10 ms control latency.</span>
        </div>
      </div>
      
      <div>
        <div className="uppercase tracking-widest text-[var(--color-primary)] font-semibold mb-3 text-[0.625rem] border-b border-[var(--color-primary)]/20 pb-1">Joint Hierarchy</div>
        <div className="flex flex-col gap-2">
          {JOINT_HIERARCHY.map((j, i) => (
            <div key={i} className="flex justify-between items-baseline gap-2">
               <span style={{ color: j.color }} className="font-semibold text-[0.6875rem] shrink-0">
                {j.label}
              </span>
              <span className="text-[var(--color-text-3)] text-[0.6875rem] truncate">{j.dof}</span>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}

function IntegratedHandInterface() {
  const [activeTab, setActiveTab] = useState<"controls" | "specs">("controls");

  return (
    <div className="flex flex-col h-full w-full bg-transparent">
      {/* Centered Tab Header with sharp edges and center divider */}
      <div 
        className="flex-shrink-0 flex items-center justify-center border-b border-white/10 bg-transparent"
        style={{ height: 48 }}
      >
        <div className="flex items-center justify-center w-full max-w-[240px]">
          <button 
            onClick={() => setActiveTab('controls')}
            className="flex-1 text-center py-2.5 text-[0.6875rem] font-bold tracking-widest uppercase transition-colors relative"
            style={{ 
              color: activeTab === 'controls' ? 'var(--color-primary)' : 'var(--color-text-3)',
            }}
          >
            Controls
            {activeTab === 'controls' && (
              <div 
                className="absolute bottom-0 left-0 right-0 h-[2px]" 
                style={{ backgroundColor: "var(--color-primary)", boxShadow: "0 0 8px rgba(16,185,129,0.4)" }} 
              />
            )}
          </button>
          
          <div className="w-[1px] h-5 bg-white/20 mx-3"></div>
          
          <button 
            onClick={() => setActiveTab('specs')}
            className="flex-1 text-center py-2.5 text-[0.6875rem] font-bold tracking-widest uppercase transition-colors relative"
            style={{ 
              color: activeTab === 'specs' ? 'var(--color-primary)' : 'var(--color-text-3)',
            }}
          >
            Specs
            {activeTab === 'specs' && (
              <div 
                className="absolute bottom-0 left-0 right-0 h-[2px]" 
                style={{ backgroundColor: "var(--color-primary)", boxShadow: "0 0 8px rgba(16,185,129,0.4)" }} 
              />
            )}
          </button>
        </div>
      </div>
      
      {/* Detail Content Wrapper */}
      <div className="flex-1 overflow-y-auto px-5 py-5 bg-transparent">
        {activeTab === 'controls' ? <ControlsTab /> : <SpecsTab />}
      </div>
    </div>
  );
}

// ─── Hook ─────────────────────────────────────────────────────────────────────
export function useHandPanel() {
  const { setPanel } = useRightPanel();

  useEffect(() => {
    setPanel({
      title: "",
      titleKey: "hand-panel-override-v4",
      badge: " ",
      badgeVariant: "grey",
      content: <IntegratedHandInterface />,
    });
    // Runs once on mount — sets the panel content for the Hand page
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
}
