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
    label: "Control center",
    href: "/",
    icon: <LayoutDashboard size={20} />,
  },
  {
    id: "map",
    label: "Live map",
    href: "/map",
    icon: <Map size={20} />,
  },
  {
    id: "diagnostics",
    label: "Diagnostics",
    href: "/diagnostics",
    icon: <Activity size={20} />,
  },
];

const LAB_NAV = [
  {
    id: "hand",
    label: "Bionic hand",
    href: "/projects/hand",
    icon: <Hand size={20} />,
  },
  {
    id: "drone",
    label: "Quadrotor",
    href: "/projects/drone",
    icon: <Plane size={20} />,
  },
  {
    id: "swarm",
    label: "Drone swarm",
    href: "/projects/swarm",
    icon: <Grid size={20} />,
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
        className="relative"
        style={{
          height: 64,
          display: "flex",
          alignItems: "center",
          borderBottom: "1px solid var(--color-border-0)",
        }}
      >
        <div 
          className="flex-shrink-0 flex items-center justify-center transition-transform duration-300"
          style={{ width: 64, height: 64 }}
        >
          <Image 
            src="/logo.webp" 
            alt="Omniverse Logo" 
            width={34} 
            height={34}
            style={{ height: 'auto', width: 32 }}
            className="object-contain"
          />
        </div>

        {!isCollapsed && (
          <div className="flex flex-1 items-center justify-between pr-4 overflow-hidden">
            <div className="flex flex-col truncate">
               <span className="truncate" style={{ fontSize: "1.0625rem", fontWeight: 700, letterSpacing: "0.05em", color: "var(--color-text-0)", fontFamily: "var(--font-mono), monospace" }}>
                 Omniverse
               </span>
               <span className="truncate" style={{ fontSize: "0.625rem", fontWeight: 600, color: "var(--color-primary)", letterSpacing: "0.1em", fontFamily: "var(--font-mono), monospace" }}>
                 Personal R&D lab
               </span>
            </div>
            <button
              onClick={onToggle}
              className="p-1.5 rounded-md hover:bg-white/5 text-zinc-500 hover:text-white transition-colors flex-shrink-0"
            >
              <ChevronRight 
                size={16} 
                style={{ 
                  transform: isCollapsed ? "rotate(0deg)" : "rotate(180deg)",
                  transition: "transform 0.3s ease" 
                }} 
              />
            </button>
          </div>
        )}

        {isCollapsed && (
          <button
            onClick={onToggle}
            className="absolute inset-0 w-full h-full opacity-0 z-10"
          />
        )}
      </div>

      <nav className="flex-1 overflow-y-auto" style={{ padding: "0.75rem 0" }}>
        
        {/* Main Production Navigation (Warehouse) */}
        <div style={{ paddingBottom: "1.25rem", borderBottom: "1px solid rgba(0,0,0,0.06)", marginBottom: "1.25rem" }} className="dark:border-white/5 px-3">
          <div
            style={{
              padding: "0 0.25rem",
              marginBottom: "0.75rem",
              fontSize: "0.625rem",
              fontWeight: 600,
              textTransform: "uppercase",
              letterSpacing: "0.05em",
              color: "var(--color-text-3)",
              fontFamily: "var(--font-inter), system-ui, sans-serif",
            }}
          >
            {!isCollapsed && "Core utilities"}
          </div>
          <div className="flex flex-col gap-0.5">
            {MAIN_NAV.map((item) => {
              const isActive = pathname === item.href;
              return (
                <Link
                  key={item.id}
                  href={item.href}
                  className="flex items-center transition-colors duration-200 group/link relative"
                  style={{
                    height: "2.5rem",
                    width: "100%",
                    borderRadius: "0.5rem",
                    fontSize: "0.75rem",
                    fontWeight: isActive ? 600 : 500,
                    textTransform: "uppercase",
                    letterSpacing: "0.05em",
                    fontFamily: "var(--font-mono), monospace",
                    color: isActive ? "#ffffff" : "var(--color-text-2)",
                    background: isActive ? "rgba(255, 255, 255, 0.08)" : "transparent",
                    boxShadow: isActive ? "inset 0 1px 0 rgba(255, 255, 255, 0.05), 0 2px 4px rgba(0,0,0,0.1)" : "none",
                  }}
                >
                  {isActive && !isCollapsed && (
                    <div className="absolute left-0 top-1/2 -translate-y-1/2 h-5 w-[3px] bg-white rounded-r-md shadow-[0_0_8px_rgba(255,255,255,0.4)]" />
                  )}
                  <div style={{ width: 40, display: 'flex', justifyContent: 'center', alignItems: 'center', opacity: isActive ? 1 : 0.7, flexShrink: 0 }}>
                    {item.icon}
                  </div>
                  {!isCollapsed && <span className="flex-1 truncate pr-4">{item.label}</span>}
                </Link>
              );
            })}
          </div>
        </div>

        {/* Research Lab Navigation (New Projects) */}
        <div style={{ paddingBottom: "1.25rem", borderBottom: "1px solid rgba(0,0,0,0.06)", marginBottom: "1.25rem" }} className="dark:border-white/5 px-3">
          <div
            style={{
              padding: "0 0.25rem",
              marginBottom: "0.75rem",
              fontSize: "0.625rem",
              fontWeight: 600,
              textTransform: "uppercase",
              letterSpacing: "0.05em",
              color: "var(--color-text-3)",
              fontFamily: "var(--font-inter), system-ui, sans-serif",
            }}
          >
            {!isCollapsed && "Research projects"}
          </div>
          <div className="flex flex-col gap-0.5">
            {LAB_NAV.map((item) => {
              const isActive = pathname === item.href;
              return (
                <Link
                  key={item.id}
                  href={item.href}
                  className="flex items-center transition-colors duration-200 group/link relative"
                  style={{
                    height: "2.5rem",
                    width: "100%",
                    borderRadius: "0.5rem",
                    fontSize: "0.75rem",
                    fontWeight: isActive ? 600 : 500,
                    textTransform: "uppercase",
                    letterSpacing: "0.05em",
                    fontFamily: "var(--font-mono), monospace",
                    color: isActive ? "#ffffff" : "var(--color-text-2)",
                    background: isActive ? "rgba(255, 255, 255, 0.08)" : "transparent",
                    boxShadow: isActive ? "inset 0 1px 0 rgba(255, 255, 255, 0.05), 0 2px 4px rgba(0,0,0,0.1)" : "none",
                  }}
                >
                  {isActive && !isCollapsed && (
                    <div className="absolute left-0 top-1/2 -translate-y-1/2 h-5 w-[3px] bg-white rounded-r-md shadow-[0_0_8px_rgba(255,255,255,0.4)]" />
                  )}
                  <div style={{ width: 40, display: 'flex', justifyContent: 'center', alignItems: 'center', opacity: isActive ? 1 : 0.7, flexShrink: 0 }}>
                    {item.icon}
                  </div>
                  {!isCollapsed && <span className="flex-1 truncate pr-4">{item.label}</span>}
                </Link>
              );
            })}
          </div>
        </div>

        {/* Fleet Units Section (Restored) */}
        <div className="px-3">
          <div
            className="flex items-center justify-between"
            style={{
              padding: "0 0.25rem",
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
                textAlign: isCollapsed ? "center" : "left",
                width: isCollapsed ? "100%" : "auto",
              }}
            >
              {isCollapsed ? "Nodes" : "Active nodes"}
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
                  className="flex items-center transition-colors duration-200 group/link relative"
                  style={{
                    height: "2.5rem",
                    width: "100%",
                    borderRadius: "0.5rem",
                    fontSize: "0.75rem",
                    fontWeight: isActive ? 600 : 500,
                    textTransform: "uppercase",
                    letterSpacing: "0.05em",
                    fontFamily: "var(--font-mono), monospace",
                    color: isActive ? "#ffffff" : "var(--color-text-2)",
                    background: isActive ? "rgba(255, 255, 255, 0.08)" : "transparent",
                    boxShadow: isActive ? "inset 0 1px 0 rgba(255, 255, 255, 0.05), 0 2px 4px rgba(0,0,0,0.1)" : "none",
                  }}
                >
                  {isActive && !isCollapsed && (
                    <div className="absolute left-0 top-1/2 -translate-y-1/2 h-5 w-[3px] bg-white rounded-r-md shadow-[0_0_8px_rgba(255,255,255,0.4)]" />
                  )}
                  <div style={{ width: 40, display: 'flex', justifyContent: 'center', alignItems: 'center', opacity: isActive ? 1 : 0.7, flexShrink: 0 }}>
                    <Activity size={20} />
                  </div>
                  {!isCollapsed && (
                    <div className="flex-1 min-w-0 flex items-center justify-between pr-4">
                      <span className="truncate">{robot.id.replace(/robot_/i, "Node-").toUpperCase()}</span>
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
        className="flex items-center gap-2"
        style={{
          padding: "1rem 0",
          borderTop: "1px solid var(--color-border-0)",
          background: "rgba(0,0,0,0.2)",
        }}
      >
           <Link href="/settings" className="flex items-center w-full px-4 group/link">
            <div
              className="flex items-center justify-center transition-colors flex-shrink-0"
              style={{
                width: 32,
                height: 32,
                borderRadius: "50%",
                background: "var(--color-primary)",
                fontSize: "0.6875rem",
                fontWeight: 600,
                color: "var(--color-primary-foreground)",
                fontFamily: "var(--font-mono), monospace",
                boxShadow: "0 0 10px rgba(16, 185, 129, 0.4)",
              }}
            >
              ME
            </div>
            {!isCollapsed && (
              <div className="flex-1 min-w-0 ml-3">
                <div style={{ fontSize: "0.75rem", fontWeight: 600, color: "var(--color-text-1)", fontFamily: "var(--font-mono), monospace", letterSpacing: "0.05em" }}>
                  Developer
                </div>
                <div style={{ fontSize: "0.6rem", color: "var(--color-primary)", fontFamily: "var(--font-mono), monospace", letterSpacing: "0.1em" }}>
                  Admin
                </div>
              </div>
            )}
           {!isCollapsed && <Settings size={16} className="text-zinc-500 group-hover/link:text-zinc-300 transition-colors" />}
         </Link>
      </div>
    </div>
  );
}
