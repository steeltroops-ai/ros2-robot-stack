"use client";

import Link from "next/link";
import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";
import { Wifi, WifiOff, Bell, User, Bot } from "lucide-react";
import { usePathname } from "next/navigation";

export default function Header({ noPadding = false }: { noPadding?: boolean }) {
  const { isConnected, robots } = useFleetTelemetry();
  const pathname = usePathname();

  // Simple title mapper
  const getPageTitle = () => {
    if (pathname === "/") return "Control center";
    if (pathname.startsWith("/robots/")) {
      const id = pathname.split("/").pop();
      return `Node: ${id?.replace(/robot_/i, "Node-").toUpperCase()}`;
    }
    if (pathname === "/map") return "Global live map";
    if (pathname === "/diagnostics") return "System diagnostics";
    if (pathname === "/settings") return "Control settings";
    return "Dashboard";
  };

  const getPageSubtitle = () => {
    if (pathname === "/") return "Personal robotics lab management";
    if (pathname.startsWith("/robots/")) return "Individual node telemetry and active controls";
    return "R&D workspace management system";
  };

  return (
    <header
      className="flex items-center justify-between flex-shrink-0 relative"
      style={{
        height: 64,
        padding: "0 2rem",
        background: "var(--color-surface-1)",
        // borderBottom removed to merge seamlessly with sidebar
      }}
    >
      {/* Seamless Inner Corner Fillet (Unibody Curve) */}
      <div 
        className="absolute pointer-events-none"
        style={{
          left: 0,
          top: "100%",
          width: "1rem",
          height: "1rem",
          background: "radial-gradient(circle at 100% 100%, transparent 1rem, var(--color-surface-1) 1rem)",
          zIndex: 10
        }}
      />
      {/* Title Section */}
      <div className="flex flex-col justify-center">
        <h2
          style={{
            fontSize: "1.125rem",
            fontWeight: 800,
            letterSpacing: "-0.02em",
            fontFamily: "var(--font-inter), system-ui, sans-serif",
            color: "var(--color-text-0)",
            lineHeight: 1.1,
          }}
        >
          {getPageTitle()}
        </h2>
        <span
          style={{
            fontSize: "0.75rem",
            fontFamily: "var(--font-inter), system-ui, sans-serif",
            color: "var(--color-text-3)",
            marginTop: "2px",
          }}
        >
          {getPageSubtitle()}
        </span>
      </div>

      {/* Actions & Status */}
      <div className="flex items-center gap-4">
        {/* Status Pills */}
        <div className="flex items-center gap-2 mr-2">
          {/* Connection Pill */}
          <div 
            className="flex items-center gap-2 px-3 py-1.5 rounded"
            style={{ 
              background: isConnected ? "rgba(16, 185, 129, 0.15)" : "rgba(239, 68, 68, 0.15)",
              border: `1px solid ${isConnected ? "rgba(16, 185, 129, 0.5)" : "rgba(239, 68, 68, 0.5)"}`,
              boxShadow: isConnected ? "0 0 10px rgba(16, 185, 129, 0.2)" : "0 0 10px rgba(239, 68, 68, 0.2)"
            }}
          >
            <div 
              className={`w-1.5 h-1.5 rounded-full ${isConnected ? "bg-[#10b981] animate-pulse" : "bg-[#ef4444]"}`} 
              style={{ boxShadow: isConnected ? "0 0 8px #10b981" : "0 0 8px #ef4444" }}
            />
            <span style={{ 
              fontSize: "0.6875rem", 
              fontWeight: 600, 
              fontFamily: "var(--font-inter), system-ui, sans-serif",
              color: isConnected ? "#10b981" : "#ef4444",
            }}>
              {isConnected ? "Connected" : "Disconnected"}
            </span>
          </div>

          {/* Fleet Count Pill */}
          <Link 
            href="/"
            className="flex items-center gap-2 px-3 py-1.5 rounded transition-all hover:bg-white/5"
            style={{ 
              background: "rgba(255, 255, 255, 0.05)",
              border: "1px solid var(--color-border-0)",
              backdropFilter: "blur(4px)"
            }}
          >
            <span style={{ fontSize: "0.6875rem", fontWeight: 600, fontFamily: "var(--font-inter), system-ui, sans-serif", color: "var(--color-text-0)" }}>
              {robots.length} active nodes
            </span>
          </Link>
        </div>

        {/* User Profile */}
        <div className="flex items-center gap-3 pl-4 border-l border-[var(--color-border-0)]">
           <button 
            className="w-9 h-9 flex items-center justify-center transition-all hover:bg-white/10 hover:text-white rounded text-zinc-400"
          >
            <Bell size={18} />
          </button>
          
          <div 
            className="w-9 h-9 rounded flex items-center justify-center shadow-sm"
            style={{ 
              background: "var(--color-primary)", 
              color: "var(--color-primary-foreground)",
              boxShadow: "0 0 10px rgba(16, 185, 129, 0.4)"
            }}
          >
            <span style={{ fontSize: "0.75rem", fontWeight: 700, fontFamily: "var(--font-inter), system-ui, sans-serif" }}>ME</span>
          </div>
        </div>
      </div>
    </header>
  );
}
