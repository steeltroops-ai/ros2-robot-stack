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
