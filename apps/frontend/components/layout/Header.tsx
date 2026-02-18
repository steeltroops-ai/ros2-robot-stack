"use client";

import Link from "next/link";
import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";
import { Wifi, WifiOff, Bell, User, Bot } from "lucide-react";
import { usePathname } from "next/navigation";

import { useState, useEffect } from "react";

export default function Header({ noPadding = false }: { noPadding?: boolean }) {
  const { isConnected, robots } = useFleetTelemetry();
  const pathname = usePathname();
  const [mounted, setMounted] = useState(false);

  useEffect(() => setMounted(true), []);

  if (!mounted) return null;

  // Simple title mapper
  const getPageTitle = () => {
    if (pathname === "/") return "Fleet Overview";
    if (pathname.startsWith("/robots/")) {
      const id = pathname.split("/").pop();
      return `Robot: ${id?.replace("_", "-").toUpperCase()}`;
    }
    if (pathname === "/map") return "Global Live Map";
    if (pathname === "/diagnostics") return "System Diagnostics";
    if (pathname === "/settings") return "Control Settings";
    return "Dashboard";
  };

  const getPageSubtitle = () => {
    if (pathname === "/") return "Real-time fleet monitoring and control";
    if (pathname.startsWith("/robots/")) return "Individual unit telemetry and teleoperation";
    return "Omniverse robotics management system";
  };

  return (
    <header
      className="flex items-center justify-between flex-shrink-0"
      style={{
        height: 64,
        padding: "0 2rem",
        background: "var(--color-surface-1)",
      }}
    >
      {/* Title Section */}
      <div className="flex flex-col justify-center">
        <h2
          style={{
            fontSize: "1rem",
            fontWeight: 700,
            letterSpacing: "-0.02em",
            color: "var(--color-text-0)",
            lineHeight: 1.1,
          }}
        >
          {getPageTitle()}
        </h2>
        <span
          style={{
            fontSize: "0.75rem",
            color: "var(--color-text-2)",
            opacity: 0.7,
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
            className="flex items-center gap-2 px-3 py-1.5 rounded-full"
            style={{ 
              background: isConnected ? "rgba(52, 199, 89, 0.1)" : "rgba(255, 59, 48, 0.1)",
              border: `1px solid ${isConnected ? "rgba(52, 199, 89, 0.2)" : "rgba(255, 59, 48, 0.2)"}`
            }}
          >
            <div 
              className={`w-1.5 h-1.5 rounded-full ${isConnected ? "bg-green-500 animate-pulse" : "bg-red-500"}`} 
            />
            <span style={{ 
              fontSize: "0.6875rem", 
              fontWeight: 700, 
              color: isConnected ? "#1a7f37" : "#d73a49",
              letterSpacing: "0.02em"
            }}>
              {isConnected ? "LIVE" : "OFFLINE"}
            </span>
          </div>

          {/* Fleet Count Pill */}
          <Link 
            href="/"
            className="flex items-center gap-2 px-3 py-1.5 rounded-full transition-all hover:bg-black/5"
            style={{ 
              background: "var(--color-surface-0)",
              border: "1px solid var(--color-border-0)"
            }}
          >
            <span style={{ fontSize: "0.6875rem", fontWeight: 700, color: "var(--color-text-0)" }}>
              {robots.length} UNITS
            </span>
          </Link>
        </div>

        {/* User Profile */}
        <div className="flex items-center gap-3 pl-4 border-l border-[var(--color-border-0)]">
           <button 
            className="w-9 h-9 flex items-center justify-center transition-all hover:bg-black/5 rounded-full text-zinc-500"
          >
            <Bell size={18} />
          </button>
          
          <div 
            className="w-9 h-9 rounded-full flex items-center justify-center shadow-sm"
            style={{ 
              background: "var(--color-primary)", 
              color: "var(--color-primary-foreground)" 
            }}
          >
            <span style={{ fontSize: "0.75rem", fontWeight: 700 }}>AD</span>
          </div>
        </div>
      </div>
    </header>
  );
}
