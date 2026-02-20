"use client";

import { useEffect } from "react";
import { useRightPanel } from "@/components/layout/RightPanelContext";
import {
  Brain,
  Cpu,
  Sparkles,
  Activity,
  Cog,
  Gauge,
  type LucideIcon,
} from "lucide-react";

// ─── Spec data ────────────────────────────────────────────────────────────────
const SPEC_CARDS: {
  icon: LucideIcon;
  label: string;
  value: string;
  detail: string;
}[] = [
  { icon: Sparkles, label: "Degrees of Freedom", value: "24 DOF",     detail: "7 arm + 17 hand joints" },
  { icon: Brain,    label: "RL Policy",           value: "PPO",         detail: "Isaac Gym Sim2Real" },
  { icon: Gauge,    label: "Control Latency",     value: "<10 ms",      detail: "Real-time CUDA inference" },
  { icon: Cog,      label: "Actuators",            value: "Brushless",   detail: "Tendon-driven mechanism" },
  { icon: Activity, label: "Force Sensing",        value: "9-point",     detail: "Palm + fingertip array" },
  { icon: Cpu,      label: "Onboard Compute",      value: "Jetson Orin", detail: "NVIDIA Orin NX 16 GB" },
];

const JOINT_HIERARCHY = [
  { label: "SHOULDER", dof: "3 DOF — Yaw / Pitch / Roll", color: "var(--color-primary)" },
  { label: "ELBOW",    dof: "1 DOF — Pitch",               color: "var(--color-primary)" },
  { label: "FOREARM",  dof: "1 DOF — Pronation",           color: "var(--color-primary)" },
  { label: "WRIST",    dof: "2 DOF — Pitch / Yaw",         color: "var(--color-primary)" },
  { label: "THUMB",    dof: "3 DOF — CMC / MCP / IP",      color: "#f59e0b" },
  { label: "INDEX",    dof: "3 DOF — MCP / PIP / DIP",     color: "#f59e0b" },
  { label: "MIDDLE",   dof: "3 DOF — MCP / PIP / DIP",     color: "#f59e0b" },
  { label: "RING",     dof: "3 DOF — MCP / PIP / DIP",     color: "#f59e0b" },
  { label: "PINKY",    dof: "3 DOF — MCP / PIP / DIP",     color: "#f59e0b" },
];

const DEMO_PHASES = [
  "Wave Gesture",
  "Sequential Curl",
  "Full Fist Clench",
  "Release & Spread",
  "Piano Wiggle",
  "Precision Pinch",
];

// ─── Section label ────────────────────────────────────────────────────────────
function SectionLabel({ children }: { children: string }) {
  return (
    <div
      className="uppercase tracking-widest pb-2"
      style={{
        fontSize: "0.625rem",
        fontWeight: 700,
        color: "var(--color-text-3)",
        borderBottom: "1px solid var(--color-border-0)",
        fontFamily: "var(--font-mono), monospace",
      }}
    >
      {children}
    </div>
  );
}

// ─── Panel body ───────────────────────────────────────────────────────────────
function HandPanelBody() {
  return (
    <div className="flex flex-col gap-5 px-5 py-4 h-full overflow-y-auto">

      {/* Spec cards */}
      <div className="flex flex-col gap-3">
        <SectionLabel>Specifications</SectionLabel>
        <div className="grid grid-cols-1 gap-2">
          {SPEC_CARDS.map((card, i) => {
            const Icon = card.icon;
            return (
              <div
                key={i}
                className="flex items-center gap-3 p-3 rounded-xl border transition-all hover:border-white/20"
                style={{
                  background: "rgba(255,255,255,0.03)",
                  borderColor: "var(--color-border-0)",
                }}
              >
                <div
                  className="flex items-center justify-center rounded-lg flex-shrink-0"
                  style={{
                    width: 32,
                    height: 32,
                    background: "rgba(16, 185, 129, 0.1)",
                    color: "var(--color-primary)",
                  }}
                >
                  <Icon size={16} />
                </div>
                <div className="flex-1 min-w-0">
                  <div
                    className="uppercase tracking-widest truncate"
                    style={{ fontSize: "0.5625rem", fontWeight: 700, color: "var(--color-text-3)", fontFamily: "var(--font-mono), monospace" }}
                  >
                    {card.label}
                  </div>
                  <div style={{ fontSize: "0.875rem", fontWeight: 800, color: "var(--color-text-0)", letterSpacing: "-0.02em", fontFamily: "var(--font-mono), monospace" }}>
                    {card.value}
                  </div>
                </div>
                <div
                  className="text-right flex-shrink-0"
                  style={{ fontSize: "0.6875rem", color: "var(--color-text-3)", maxWidth: 100, textAlign: "right" }}
                >
                  {card.detail}
                </div>
              </div>
            );
          })}
        </div>
      </div>

      {/* Joint hierarchy */}
      <div className="flex flex-col gap-3">
        <SectionLabel>Joint Hierarchy</SectionLabel>
        <div
          className="flex flex-col gap-1 font-mono"
          style={{ fontSize: "0.6875rem", lineHeight: 1.7 }}
        >
          {JOINT_HIERARCHY.map((j, i) => (
            <div key={i} className="flex items-center gap-2">
              <span style={{ color: j.color, fontWeight: 700, minWidth: 72 }}>
                {j.label}
              </span>
              <span style={{ color: "var(--color-text-2)" }}>{j.dof}</span>
            </div>
          ))}
        </div>
      </div>

      {/* Demo phases */}
      <div className="flex flex-col gap-3">
        <SectionLabel>Idle Demo Phases</SectionLabel>
        <div className="flex flex-col gap-1.5">
          {DEMO_PHASES.map((phase, i) => (
            <div key={i} className="flex items-center gap-2.5">
              <div
                className="flex items-center justify-center rounded text-center flex-shrink-0"
                style={{
                  width: 22,
                  height: 22,
                  background: "rgba(16, 185, 129, 0.1)",
                  color: "var(--color-primary)",
                  fontSize: "0.625rem",
                  fontWeight: 800,
                  fontFamily: "var(--font-mono), monospace",
                }}
              >
                {i + 1}
              </div>
              <span style={{ fontSize: "0.75rem", color: "var(--color-text-2)" }}>
                {phase}
              </span>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}

// ─── Hook ─────────────────────────────────────────────────────────────────────
/** Register bionic hand content in the shared right panel. */
export function useHandPanel() {
  const { setPanel } = useRightPanel();

  useEffect(() => {
    setPanel({
      title: "Specifications",
      badge: "SIM",
      badgeVariant: "blue",
      content: <HandPanelBody />,
    });
    // runs once on mount
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
}
