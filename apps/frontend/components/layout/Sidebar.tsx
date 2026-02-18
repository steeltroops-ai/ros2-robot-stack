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
} from "lucide-react";
import Image from "next/image";

interface NavItem {
  id: string;
  label: string;
  href: string;
  icon: React.ReactNode;
}



const MAIN_NAV: NavItem[] = [
  {
    id: "overview",
    label: "Overview",
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
  {
    id: "settings",
    label: "Settings",
    href: "/settings",
    icon: <Settings size={16} />,
  },
];



import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";

import { useState, useEffect } from "react";

export default function Sidebar({ 
  isCollapsed, 
  onToggle 
}: { 
  isCollapsed: boolean; 
  onToggle: () => void; 
}) {
  const [mounted, setMounted] = useState(false);
  useEffect(() => setMounted(true), []);

  const pathname = usePathname();
  const { robots } = useFleetTelemetry();

  if (!mounted) return null;

  return (
    <div
      onClick={onToggle}
      className="w-full h-full flex flex-col group transition-all duration-300 ease-in-out cursor-pointer"
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
          justifyContent: isCollapsed ? "center" : "flex-start",
          padding: isCollapsed ? "0" : "0 1.25rem",
        }}
      >
        <div className="flex items-center gap-3">
          <div 
            className="flex-shrink-0 transition-transform duration-300 group-hover:scale-110"
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
            <span
              style={{
                fontSize: "1.0625rem",
                fontWeight: 700,
                letterSpacing: "-0.02em",
                color: "var(--color-text-0)",
              }}
            >
              Omniverse
            </span>
          )}
        </div>
      </div>

      <nav className="flex-1 overflow-y-auto" style={{ padding: "0.75rem" }}>
        <div style={{ marginBottom: "1.5rem" }}>
          <div
            style={{
              padding: "0 0.75rem",
              marginBottom: "0.5rem",
              fontSize: "0.6875rem",
              fontWeight: 500,
              textTransform: "uppercase",
              letterSpacing: "0.06em",
              color: "var(--color-text-4)",
            }}
          >
            {!isCollapsed && "Navigation"}
          </div>
          <div className="flex flex-col gap-0.5">
            {MAIN_NAV.map((item) => {
              const isActive = pathname === item.href;
              return (
                <Link
                  key={item.id}
                  href={item.href}
                  onClick={(e) => e.stopPropagation()}
                  className="flex items-center gap-2.5 transition-colors"
                  style={{
                    padding: "0.5rem 0.75rem",
                    borderRadius: "2rem", // Fully rounded pill
                    fontSize: "0.8125rem",
                    fontWeight: isActive ? 600 : 500,
                    color: isActive
                      ? "var(--color-primary-foreground)" // White text
                      : "var(--color-text-2)",
                    background: isActive
                      ? "var(--color-primary)" // Black pill
                      : "transparent",
                    transition: "all 0.2s ease",
                  }}
                >
                  <span style={{ opacity: isActive ? 1 : 0.8 }}>
                    {item.icon}
                  </span>
                  {!isCollapsed && item.label}
                </Link>
              );
            })}
          </div>
        </div>

        {/* Robots Section */}
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
                fontSize: "0.6875rem",
                fontWeight: 500,
                textTransform: "uppercase",
                letterSpacing: "0.06em",
                color: "var(--color-text-4)",
              }}
            >
              {isCollapsed ? "Fleet" : "Fleet Units"}
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
                  onClick={(e) => e.stopPropagation()}
                  className="flex items-center gap-2.5 transition-colors"
                  style={{
                    padding: "0.5rem 0.75rem",
                    borderRadius: "2rem", // Fully rounded pill
                    fontSize: "0.8125rem",
                    fontWeight: isActive ? 600 : 500,
                    color: isActive
                      ? "var(--color-primary-foreground)" // White text
                      : "var(--color-text-2)",
                    background: isActive
                      ? "var(--color-primary)" // Black pill
                      : "transparent",
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

      <div
        className="flex items-center gap-3"
        onClick={(e) => e.stopPropagation()}
        style={{
          padding: "1rem 1.25rem",
        }}
      >
        <div
          className="flex items-center justify-center"
          style={{
            width: 32,
            height: 32,
            borderRadius: "50%",
            background: "var(--color-surface-3)",
            fontSize: "0.6875rem",
            fontWeight: 600,
            color: "var(--color-text-2)",
            flexShrink: 0,
          }}
        >
          OP
        </div>
        {!isCollapsed && (
          <div className="flex-1 min-w-0">
            <div
              style={{
                fontSize: "0.8125rem",
                fontWeight: 500,
                color: "var(--color-text-1)",
              }}
            >
              Operator
            </div>
            <div
              style={{
                fontSize: "0.6875rem",
                color: "var(--color-text-4)",
              }}
            >
              System Admin
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
