"use client";

import Sidebar from "./Sidebar";
import Header from "./Header";
import { RightPanel, PANEL_W } from "./RightPanel";
import { RightPanelProvider } from "./RightPanelContext";
import { useState } from "react";

export default function DashboardLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  const [isCollapsed, setIsCollapsed] = useState(false);

  return (
    <RightPanelProvider>
      <div
        className="flex h-screen w-full overflow-hidden panel"
        style={{ borderRadius: 0, border: "none", boxShadow: "none" }}
      >
        {/* ── Left Sidebar ── */}
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
          {/* Header */}
          <div style={{ position: "relative", zIndex: 30, flexShrink: 0 }}>
            <Header noPadding={false} />
          </div>

          {/* Main stage — children render immediately, no gate delay */}
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
                {children}
              </div>
            </main>
          </div>
        </div>

        {/* ── Right Panel ── */}
        <RightPanel />
      </div>
    </RightPanelProvider>
  );
}

