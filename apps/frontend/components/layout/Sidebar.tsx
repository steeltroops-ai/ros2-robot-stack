"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import {
  LayoutDashboard,
  Bot,
  Map,
  Activity,
  Settings,
  ChevronRight,
  Hand,
  Plane,
  Grid,
} from "lucide-react";
import Image from "next/image";
import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";

const MAIN_NAV = [
  {
    id: "overview",
    label: "Warehouse Ops",
    href: "/",
    icon: <LayoutDashboard size={16} />,
  },
  {
    id: "map",
    label: "Live Map",
    href: "/map",
    icon: <Map size={16} />,
  },
  {
    id: "diagnostics",
    label: "Diagnostics",
    href: "/diagnostics",
    icon: <Activity size={16} />,
  },
];

const LAB_NAV = [
  {
    id: "hand",
    label: "P2: Viral Hand",
    href: "/projects/hand",
    icon: <Hand size={16} />,
  },
  {
    id: "drone",
    label: "P3: Explorer Drone",
    href: "/projects/drone",
    icon: <Plane size={16} />,
  },
  {
    id: "swarm",
    label: "P4: Hive Swarm",
    href: "/projects/swarm",
    icon: <Grid size={16} />,
  },
];

export default function Sidebar({ 
  isCollapsed, 
  onToggle 
}: { 
  isCollapsed: boolean; 
  onToggle: () => void; 
}) {
  const pathname = usePathname();
  const { robots } = useFleetTelemetry();

  return (
    <div
      className="w-full h-full flex flex-col group"
      style={{
        background: "var(--color-surface-1)",
      }}
    >
      {/* Logo Section */}
      <div
        style={{
          height: 64,
          display: "flex",
          alignItems: "center",
          justifyContent: isCollapsed ? "center" : "space-between",
          padding: isCollapsed ? "0" : "0 1.25rem",
          borderBottom: "1px solid var(--color-border-0)",
        }}
      >
        <div className="flex items-center gap-3">
          <div 
            className="flex-shrink-0 transition-transform duration-300"
            style={{ width: 28, height: 28 }}
          >
            <Image 
              src="/logo.webp" 
              alt="Omniverse Logo" 
              width={30} 
              height={30}
              style={{ height: 'auto' }}
              className="object-contain"
            />
          </div>
          {!isCollapsed && (
            <div className="flex flex-col">
               <span style={{ fontSize: "1.0625rem", fontWeight: 700, letterSpacing: "0.05em", color: "var(--color-text-0)", fontFamily: "var(--font-mono), monospace" }}>
                 OMNIVERSE
               </span>
               <span style={{ fontSize: "0.625rem", fontWeight: 600, color: "var(--color-primary)", textTransform: "uppercase", letterSpacing: "0.1em", fontFamily: "var(--font-mono), monospace" }}>
                 [sys.lab]
               </span>
            </div>
          )}
        </div>

        {/* Toggle Button */}
        {!isCollapsed && (
          <button
            onClick={onToggle}
            className="p-1.5 rounded-md hover:bg-white/5 text-zinc-500 hover:text-white transition-colors"
          >
            <ChevronRight 
              size={16} 
              style={{ 
                transform: isCollapsed ? "rotate(0deg)" : "rotate(180deg)",
                transition: "transform 0.3s ease" 
              }} 
            />
          </button>
        )}
        {isCollapsed && (
          <button
            onClick={onToggle}
            className="absolute inset-0 w-full h-full opacity-0 z-10"
          />
        )}
      </div>

      <nav className="flex-1 overflow-y-auto" style={{ padding: "0.75rem" }}>
        
        {/* Main Production Navigation (Warehouse) */}
        <div style={{ paddingBottom: "1.25rem", borderBottom: "1px solid rgba(0,0,0,0.06)", marginBottom: "1.25rem" }} className="dark:border-white/5">
          <div
            style={{
              padding: "0 0.75rem",
              marginBottom: "0.75rem",
              fontSize: "0.625rem",
              fontWeight: 600,
              textTransform: "uppercase",
              letterSpacing: "0.05em",
              color: "var(--color-text-3)",
              fontFamily: "var(--font-inter), system-ui, sans-serif",
            }}
          >
            {!isCollapsed && "OPERATIONS"}
          </div>
          <div className="flex flex-col gap-0.5">
            {MAIN_NAV.map((item) => {
              const isActive = pathname === item.href;
              return (
                <Link
                  key={item.id}
                  href={item.href}
                  className="flex items-center gap-2.5 transition-colors group/link"
                  style={{
                    padding: "0.5rem 0.75rem",
                    borderRadius: "0.25rem",
                    fontSize: "0.75rem",
                    fontWeight: isActive ? 600 : 500,
                    textTransform: "uppercase",
                    letterSpacing: "0.05em",
                    fontFamily: "var(--font-mono), monospace",
                    color: isActive ? "var(--color-primary)" : "var(--color-text-2)",
                    background: isActive ? "rgba(16, 185, 129, 0.1)" : "transparent",
                    borderLeft: isActive ? "3px solid var(--color-primary)" : "3px solid transparent",
                    transition: "all 0.2s ease",
                  }}
                >
                  <span style={{ opacity: isActive ? 1 : 0.8 }}>{item.icon}</span>
                  {!isCollapsed && item.label}
                </Link>
              );
            })}
          </div>
        </div>

        {/* Research Lab Navigation (New Projects) */}
        <div style={{ paddingBottom: "1.25rem", borderBottom: "1px solid rgba(0,0,0,0.06)", marginBottom: "1.25rem" }} className="dark:border-white/5">
          <div
            style={{
              padding: "0 0.75rem",
              marginBottom: "0.75rem",
              fontSize: "0.625rem",
              fontWeight: 600,
              textTransform: "uppercase",
              letterSpacing: "0.05em",
              color: "var(--color-text-3)",
              fontFamily: "var(--font-inter), system-ui, sans-serif",
            }}
          >
            {!isCollapsed && "RESEARCH LABS"}
          </div>
          <div className="flex flex-col gap-0.5">
            {LAB_NAV.map((item) => {
              const isActive = pathname === item.href;
              return (
                <Link
                  key={item.id}
                  href={item.href}
                  className="flex items-center gap-2.5 transition-colors group/link"
                  style={{
                    padding: "0.5rem 0.75rem",
                    borderRadius: "0.25rem",
                    fontSize: "0.75rem",
                    fontWeight: isActive ? 600 : 500,
                    textTransform: "uppercase",
                    letterSpacing: "0.05em",
                    fontFamily: "var(--font-mono), monospace",
                    color: isActive ? "var(--color-primary)" : "var(--color-text-2)",
                    background: isActive ? "rgba(16, 185, 129, 0.1)" : "transparent",
                    borderLeft: isActive ? "3px solid var(--color-primary)" : "3px solid transparent",
                    transition: "all 0.2s ease",
                  }}
                >
                  <span style={{ opacity: isActive ? 1 : 0.8 }}>{item.icon}</span>
                  {!isCollapsed && item.label}
                </Link>
              );
            })}
          </div>
        </div>

        {/* Fleet Units Section (Restored) */}
        <div>
          <div
            className="flex items-center justify-between"
            style={{
              padding: "0 0.75rem",
              marginBottom: "0.5rem",
            }}
          >
            <span
              style={{
                fontSize: "0.625rem",
                fontWeight: 600,
                textTransform: "uppercase",
                letterSpacing: "0.05em",
                color: "var(--color-text-3)",
                fontFamily: "var(--font-inter), system-ui, sans-serif",
              }}
            >
              {isCollapsed ? "FLT" : "FLEET UNITS"}
            </span>
            {!isCollapsed && (
              <span
                className="data-mono"
                style={{
                  fontSize: "0.625rem",
                  color: "var(--color-success)",
                  fontWeight: 500,
                }}
              >
                {robots.length} online
              </span>
            )}
          </div>
          <div className="flex flex-col gap-0.5">
            {robots.map((robot) => {
              const href = `/robots/${robot.id}`;
              const isActive = pathname === href;
              return (
                <Link
                  key={robot.id}
                  href={href}
                  className="flex items-center gap-2.5 transition-colors group/link"
                  style={{
                    padding: "0.5rem 0.75rem",
                    borderRadius: "0.25rem",
                    fontSize: "0.75rem",
                    fontWeight: isActive ? 600 : 500,
                    textTransform: "uppercase",
                    letterSpacing: "0.05em",
                    fontFamily: "var(--font-mono), monospace",
                    color: isActive ? "var(--color-primary)" : "var(--color-text-2)",
                    background: isActive ? "rgba(16, 185, 129, 0.1)" : "transparent",
                    borderLeft: isActive ? "3px solid var(--color-primary)" : "3px solid transparent",
                    transition: "all 0.2s ease",
                  }}
                >
                  <span style={{ opacity: isActive ? 1 : 0.8 }}>
                    <Activity size={16} />
                  </span>
                  {!isCollapsed && (
                    <div className="flex-1 min-w-0 flex items-center justify-between">
                      <span className="truncate">{robot.id.replace("_", "-").toUpperCase()}</span>
                      {isActive && <ChevronRight size={14} style={{ opacity: 0.5 }} />}
                    </div>
                  )}
                </Link>
              );
            })}
          </div>
        </div>
      </nav>

      {/* Operator Profile & Settings */}
      <div
        className="flex items-center gap-3"
        style={{
          padding: "1rem 1.25rem",
          borderTop: "1px solid var(--color-border-0)",
          background: "rgba(0,0,0,0.2)",
        }}
      >
         <Link href="/settings" className="flex items-center gap-3 w-full">
            <div
              className="flex items-center justify-center transition-colors"
              style={{
                width: 32,
                height: 32,
                borderRadius: "0.25rem",
                background: "var(--color-primary)",
                fontSize: "0.6875rem",
                fontWeight: 600,
                color: "var(--color-primary-foreground)",
                fontFamily: "var(--font-mono), monospace",
                flexShrink: 0,
                boxShadow: "0 0 10px rgba(16, 185, 129, 0.4)",
              }}
            >
              OP
            </div>
            {!isCollapsed && (
              <div className="flex-1 min-w-0">
                <div style={{ fontSize: "0.75rem", fontWeight: 600, color: "var(--color-text-1)", fontFamily: "var(--font-mono), monospace", letterSpacing: "0.05em" }}>
                  OPERATOR
                </div>
                <div style={{ fontSize: "0.6rem", color: "var(--color-primary)", fontFamily: "var(--font-mono), monospace", letterSpacing: "0.1em" }}>
                  SYS.ADMIN
                </div>
              </div>
            )}
           {!isCollapsed && <Settings size={14} className="text-zinc-400" />}
         </Link>
      </div>
    </div>
  );
}
