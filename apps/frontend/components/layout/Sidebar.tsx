"use client";

import Link from "next/link";

interface NavItem {
  id: string;
  label: string;
  href: string;
  icon?: string; // Replace with proper Icon component later
}

const ROBOT_NAV: NavItem[] = [
  { id: "robot_1", label: "Robot 1 (AMR)", href: "/robots/robot_1" },
  { id: "robot_2", label: "Robot 2 (Arm)", href: "/robots/robot_2" },
];

const MAIN_NAV: NavItem[] = [
  { id: "overview", label: "Fleet Overview", href: "/" },
  { id: "ml-vision", label: "ML Vision", href: "/ml-vision" },
];

export default function Sidebar() {
  return (
    <div className="w-64 h-full bg-[rgb(var(--bg-primary))] border-r border-[rgb(var(--border-primary))] flex flex-col">
      {/* Brand Header */}
      <div className="p-6 border-b border-[rgb(var(--border-primary))]">
        <h1 className="text-xl font-bold tracking-tight text-[rgb(var(--text-primary))]">
          Omniverse
          <span className="text-[rgb(var(--brand-primary))]">Fleet</span>
        </h1>
        <p className="text-xs text-[rgb(var(--text-tertiary))] mt-1">
          Industrial Control v1.0
        </p>
      </div>

      {/* Main Navigation */}
      <nav className="flex-1 p-4 space-y-6 overflow-y-auto">
        {/* Section: Platform */}
        <div>
          <h2 className="px-4 text-xs font-semibold text-[rgb(var(--text-tertiary))] uppercase tracking-wider mb-2">
            Platform
          </h2>
          <div className="space-y-1">
            {MAIN_NAV.map((item) => (
              <Link key={item.id} href={item.href} className="nav-link">
                {item.label}
              </Link>
            ))}
          </div>
        </div>

        {/* Section: Robots */}
        <div>
          <div className="flex items-center justify-between px-4 mb-2">
            <h2 className="text-xs font-semibold text-[rgb(var(--text-tertiary))] uppercase tracking-wider">
              Active Units
            </h2>
            <span className="px-2 py-0.5 text-[10px] bg-emerald-100 text-emerald-800 rounded-full font-mono">
              {ROBOT_NAV.length} ONLINE
            </span>
          </div>

          <div className="space-y-1">
            {ROBOT_NAV.map((robot) => (
              <Link key={robot.id} href={robot.href} className="nav-link group">
                <span className="w-2 h-2 rounded-full bg-emerald-500 mr-3 group-hover:animate-pulse"></span>
                {robot.label}
              </Link>
            ))}
          </div>
        </div>
      </nav>

      {/* Footer / User Profile */}
      <div className="p-4 border-t border-[rgb(var(--border-primary))] bg-[rgb(var(--bg-secondary))]">
        <div className="flex items-center">
          <div className="w-8 h-8 rounded-full bg-[rgb(var(--brand-primary))] flex items-center justify-center text-white font-bold">
            OP
          </div>
          <div className="ml-3">
            <p className="text-sm font-medium text-[rgb(var(--text-primary))]">
              Operator
            </p>
            <p className="text-xs text-[rgb(var(--text-secondary))]">
              System Admin
            </p>
          </div>
        </div>
      </div>
    </div>
  );
}
