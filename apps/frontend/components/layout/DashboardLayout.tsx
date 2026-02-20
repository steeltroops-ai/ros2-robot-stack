"use client";

import Sidebar from "./Sidebar";
import Header from "./Header";
import { RightPanel, PANEL_W } from "./RightPanel";
import { RightPanelProvider } from "./RightPanelContext";
import { useState, useEffect } from "react";

export default function DashboardLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  const [isCollapsed, setIsCollapsed] = useState(false);

  // ── Content-readiness gate ────────────────────────────────────────────────
  // Sidebar, Header and RightPanel all render on the first paint (no mounted
  // guards blocking them). We hold off rendering the page *content* (3D
  // viewers, dashboards, etc.) until requestAnimationFrame fires — i.e. the
  // browser has actually committed one paint with the chrome visible.
  // This guarantees the visual order: chrome first → content second.
  const [contentReady, setContentReady] = useState(false);
  useEffect(() => {
    const raf = requestAnimationFrame(() => setContentReady(true));
    return () => cancelAnimationFrame(raf);
  }, []);

  return (
    <RightPanelProvider>
      <div
        className="flex h-screen w-full overflow-hidden panel"
        style={{ borderRadius: 0, border: "none", boxShadow: "none" }}
      >
        {/* ── Left Sidebar — renders immediately, no mounted guard ── */}
        <aside
          className="flex-shrink-0 transition-all duration-300 ease-in-out"
          style={{
            width: isCollapsed ? 64 : 240,
            position: "relative",
            zIndex: 40,
          }}
        >
          <Sidebar
            isCollapsed={isCollapsed}
            onToggle={() => setIsCollapsed(!isCollapsed)}
          />
        </aside>

        {/* ── Center column ── */}
        <div className="flex flex-col flex-1 h-full min-w-0">
          {/* Header — renders immediately, no mounted guard */}
          <div style={{ position: "relative", zIndex: 30, flexShrink: 0 }}>
            <Header noPadding={false} />
          </div>

          {/*
            Main stage
            paddingRight reserves the 360 px the fixed right panel occupies.
            Children are held behind the RAF gate so 3D viewers never beat the
            chrome to the screen.
          */}
          <div
            className="flex flex-1 overflow-hidden"
            style={{
              padding: "0.75rem",
              paddingRight: `calc(${PANEL_W}px + 0.75rem)`,
              position: "relative",
              zIndex: 1,
            }}
          >
            <main className="flex-1 overflow-hidden min-w-0">
              <div className="w-full h-full overflow-hidden">
                {contentReady ? (
                  children
                ) : (
                  // ── Chrome-paint skeleton ─────────────────────────────────
                  // Shown for one frame while the browser paints the chrome.
                  // Uses the same glass surface so the transition is invisible.
                  <div
                    className="w-full h-full rounded-2xl"
                    style={{
                      background: "rgba(255,255,255,0.02)",
                      border: "1px solid var(--color-border-0)",
                    }}
                  />
                )}
              </div>
            </main>
          </div>
        </div>

        {/*
          ── Right Panel shell ─────────────────────────────────────────────
          Sibling of the flex column so it's truly fixed-positioned.
          Renders in the same React tree as sidebar/header — same paint frame.
        */}
        <RightPanel />
      </div>
    </RightPanelProvider>
  );
}
