"use client";

import dynamic from "next/dynamic";
import { useHandPanel } from "@/components/layout/panel-content/HandPanelContent";

const HandViewer = dynamic(
  () => import("@/components/scene/HandViewer"),
  { ssr: false }
);

export default function HandProjectPage() {
  // ── Inject spec sheet content into the shared right panel ─────────────────
  useHandPanel();

  return (
    <div className="w-full h-full relative rounded-2xl overflow-hidden border border-white/10 shadow-[0_0_40px_rgba(0,0,0,0.3)] bg-black/40">
      <HandViewer />

      {/* Floating title overlay */}
      <div
        className="absolute top-0 left-0 right-0 pointer-events-none"
        style={{
          padding: "1.5rem 2rem",
          background: "linear-gradient(to bottom, rgba(8,9,13,0.75) 0%, transparent 100%)",
        }}
      >
        <div className="flex items-center gap-3 mb-1">
          <div
            className="px-2 py-0.5 rounded-full text-[9px] font-mono font-bold tracking-widest uppercase"
            style={{
              background: "rgba(16, 185, 129, 0.15)",
              color: "#10b981",
              border: "1px solid rgba(16, 185, 129, 0.3)",
            }}
          >
            Project 02
          </div>
          <div
            className="px-2 py-0.5 rounded-full text-[9px] font-mono font-bold tracking-widest uppercase animate-pulse"
            style={{
              background: "rgba(245, 158, 11, 0.12)",
              color: "#f59e0b",
              border: "1px solid rgba(245, 158, 11, 0.3)",
            }}
          >
            Live Preview
          </div>
        </div>
        <h1
          className="text-2xl font-black tracking-tight"
          style={{
            color: "#ffffff",
            fontFamily: "var(--font-mono), monospace",
            letterSpacing: "-0.02em",
          }}
        >
          THE VIRAL HAND
        </h1>
        <p className="text-sm mt-1" style={{ color: "rgba(255,255,255,0.5)", maxWidth: "32rem" }}>
          Neuro-adaptive bionic manipulator · 24 DOF · RL-powered dexterity
        </p>
      </div>

      {/* Bottom info bar */}
      <div
        className="absolute bottom-0 left-0 right-0 pointer-events-none flex items-center justify-between"
        style={{
          padding: "1rem 2rem",
          background: "linear-gradient(to top, rgba(8,9,13,0.75) 0%, transparent 100%)",
        }}
      >
        <span
          className="text-[10px] font-mono tracking-widest uppercase"
          style={{ color: "rgba(255,255,255,0.3)" }}
        >
          Procedural Geometry · React Three Fiber · No GLTF
        </span>
        <span
          className="text-[10px] font-mono tracking-widest uppercase"
          style={{ color: "rgba(16, 185, 129, 0.5)" }}
        >
          Orbit to inspect
        </span>
      </div>
    </div>
  );
}
