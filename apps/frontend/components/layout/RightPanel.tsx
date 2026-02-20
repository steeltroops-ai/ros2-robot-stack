"use client";

import { useRightPanel } from "./RightPanelContext";
import { motion, AnimatePresence } from "framer-motion";

// ─── Badge colour map ─────────────────────────────────────────────────────────
const BADGE_STYLES = {
  green: { bg: "rgba(16, 185, 129, 0.12)", dot: "#10b981", text: "#10b981" },
  amber: { bg: "rgba(245, 158, 11, 0.12)",  dot: "#f59e0b", text: "#f59e0b" },
  blue:  { bg: "rgba(56, 189, 248, 0.12)",  dot: "#38bdf8", text: "#38bdf8" },
  grey:  { bg: "rgba(255,255,255,0.07)",     dot: "#94a3b8", text: "#94a3b8" },
} as const;

const HEADER_H = 64; // Must match <Header /> height exactly
const PANEL_W  = 360;

// ─────────────────────────────────────────────────────────────────────────────
/**
 * RightPanel — permanent fixed-position chrome that visually matches the
 * Sidebar and Header exactly:
 *   • background: var(--color-surface-1) — identical to sidebar/header token
 *   • NO inline backdropFilter — backdrop blur comes from the body wallpaper
 *     layer, exactly as the sidebar and header work
 *   • position: fixed, right: 0, top: HEADER_H, bottom: 0 — full height below
 *     header, flush to the right viewport edge
 *   • The inverse concave fillet at top-left creates the unibody joint with
 *     the header's bottom-right, matching the header's own fillet on its
 *     bottom-left corner
 *   • Renders immediately — no hydration guard in the shell itself, only the
 *     injected content cross-fades in when ready
 */
export function RightPanel() {
  const { title, badge, badgeVariant = "grey", content } = useRightPanel();
  const bs = BADGE_STYLES[badgeVariant];

  return (
    <aside
      // shadow-2xl matches the drop-shadow used on fixed panels everywhere
      className="flex flex-col shadow-2xl"
      style={{
        position: "fixed",
        right: 0,
        top: HEADER_H,
        bottom: 0,
        width: PANEL_W,
        zIndex: 100,
        // ── Exact same background token as Sidebar + Header ──────────────
        // The parent .panel container in DashboardLayout provides the
        // frosted glass effect via backdrop-filter. Adding it here
        // again would cause double-blurring and over-saturation.
        background: "var(--color-surface-1)",
      }}
    >
      {/* ── Inverse concave fillet (top-left corner) ────────────────────────
          Mirrors the fillet the Header renders at its own bottom-left.
          Together they create a seamless unibody joint between the header
          and this panel — identical to the original dashboard/robot pages. */}
      <div
        className="absolute pointer-events-none"
        style={{
          right: "100%",
          top: 0,
          width: "1rem",
          height: "1rem",
          background:
            "radial-gradient(circle at 0% 100%, transparent 1rem, var(--color-surface-1) 1rem)",
          zIndex: 10,
        }}
      />

      {/* ── Panel Header ────────────────────────────────────────────────────
          height = HEADER_H so it aligns precisely with the global header row.
          Uses the same bottom-divider style as the sidebar's logo section. */}
      <div
        className="flex items-center justify-between px-6 flex-shrink-0"
        style={{
          height: HEADER_H,
          borderBottom: "1px solid var(--color-border-0)",
        }}
      >
        {/* Title — cross-fades when the page context pushes a new title */}
        <AnimatePresence mode="wait">
          <motion.span
            key={title}
            initial={{ opacity: 0, y: -4 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: 4 }}
            transition={{ duration: 0.14 }}
            style={{
              fontSize: "1rem",
              fontWeight: 600,
              color: "var(--color-text-0)",
              letterSpacing: "-0.01em",
              fontFamily: "var(--font-inter), system-ui, sans-serif",
            }}
          >
            {title}
          </motion.span>
        </AnimatePresence>

        {/* Badge — cross-fades independently so colour change is smooth */}
        <AnimatePresence mode="wait">
          {badge && (
            <motion.div
              key={badge + badgeVariant}
              initial={{ opacity: 0, scale: 0.8 }}
              animate={{ opacity: 1, scale: 1 }}
              exit={{ opacity: 0, scale: 0.8 }}
              transition={{ duration: 0.14 }}
              className="flex items-center gap-2 px-2.5 py-1 rounded-full"
              style={{ background: bs.bg }}
            >
              <span
                className="w-1.5 h-1.5 rounded-full animate-pulse"
                style={{ display: "inline-block", background: bs.dot }}
              />
              <span
                style={{
                  fontSize: "0.6875rem",
                  fontWeight: 700,
                  color: bs.text,
                  letterSpacing: "0.08em",
                  fontFamily: "var(--font-mono), monospace",
                  textTransform: "uppercase",
                }}
              >
                {badge}
              </span>
            </motion.div>
          )}
        </AnimatePresence>
      </div>

      {/* ── Panel Body ──────────────────────────────────────────────────────
          Content is injected by the active page via useRightPanel().setPanel().
          The key on motion.div ensures a cross-fade whenever the page changes. */}
      <div className="flex-1 overflow-hidden">
        <AnimatePresence mode="wait">
          <motion.div
            key={title}
            initial={{ opacity: 0, x: 14 }}
            animate={{ opacity: 1, x: 0 }}
            exit={{ opacity: 0, x: -14 }}
            transition={{ duration: 0.18, ease: "easeOut" }}
            className="flex flex-col h-full overflow-y-auto"
          >
            {content ?? (
              // ── Idle placeholder — visible before any page sets content ──
              <div className="flex flex-col items-center justify-center h-full gap-3 px-6">
                <div
                  className="w-10 h-10 rounded-full flex items-center justify-center"
                  style={{
                    background: "rgba(255,255,255,0.04)",
                    border: "1px solid var(--color-border-0)",
                  }}
                >
                  <span style={{ fontSize: "1.25rem" }}>📋</span>
                </div>
                <span
                  style={{
                    fontSize: "0.75rem",
                    color: "var(--color-text-3)",
                    textAlign: "center",
                    lineHeight: 1.6,
                  }}
                >
                  Select a unit or navigate to a page
                  <br />
                  to see its details here.
                </span>
              </div>
            )}
          </motion.div>
        </AnimatePresence>
      </div>
    </aside>
  );
}

export { PANEL_W, HEADER_H };
