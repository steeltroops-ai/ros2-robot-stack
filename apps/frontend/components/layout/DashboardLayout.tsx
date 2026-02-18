"use client";

import Sidebar from "./Sidebar";
import Header from "./Header";
import { useState } from "react";
import { motion, AnimatePresence } from "framer-motion";

export default function DashboardLayout({
  children,
  noPadding = false,
  rightPanel,
}: {
  children: React.ReactNode;
  noPadding?: boolean;
  rightPanel?: React.ReactNode;
}) {
  const [isCollapsed, setIsCollapsed] = useState(false);

  return (
    <div
      className="flex h-screen w-full overflow-hidden"
      style={{ background: "var(--color-surface-1)" }}
    >
      {/* Left Sidebar */}
      <aside
        className="flex-shrink-0 transition-all duration-300 ease-in-out"
        style={{ 
          width: isCollapsed ? 64 : 240,
          position: "relative",
          zIndex: 40 // Highest layer
        }}
      >
        <Sidebar isCollapsed={isCollapsed} onToggle={() => setIsCollapsed(!isCollapsed)} />
      </aside>

      {/* Center Area (Header + Content) */}
      <div className="flex flex-col flex-1 h-full min-w-0">
        <div style={{ position: "relative", zIndex: 30 }}>
          <Header noPadding={noPadding} />
        </div>
        
        <div className="flex flex-1 overflow-hidden" style={{ padding: "0.75rem", position: "relative", zIndex: 1 }}>
          {/* The Stage (Main Content) */}
          <main 
            className="flex-1 overflow-hidden transition-all duration-500 ease-in-out"
            style={{ 
              paddingRight: rightPanel ? "0.75rem" : "0",
            }}
          >
            <div 
              className="w-full h-full overflow-hidden"
              style={{ 
                background: "var(--color-surface-0)", 
                borderRadius: "1.25rem",
                border: "1px solid var(--color-border-0)",
                padding: noPadding ? 0 : "1.5rem 2rem",
              }}
            >
              {children}
            </div>
          </main>

          {/* Right Panel Slot - Animated Slider */}
          <AnimatePresence>
            {rightPanel && (
              <motion.aside 
                initial={{ x: 380, opacity: 0 }}
                animate={{ x: 0, opacity: 1 }}
                exit={{ x: 380, opacity: 0 }}
                transition={{ type: "spring", damping: 25, stiffness: 200 }}
                className="flex-shrink-0 h-full overflow-hidden flex flex-col"
                style={{ 
                  width: 360,
                  background: "var(--color-surface-0)",
                  borderRadius: "1.25rem",
                  border: "1px solid var(--color-border-0)",
                  position: "relative",
                  zIndex: 20
                }}
              >
                 {rightPanel}
              </motion.aside>
            )}
          </AnimatePresence>
        </div>
      </div>
    </div>
  );
}
