"use client";

import {
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
} from "lucide-react";
import { useCallback, useEffect, useRef } from "react";

interface TeleopPanelProps {
  onControl: (linear: number, angular: number) => void;
}

export function TeleopPanel({ onControl }: TeleopPanelProps) {
  const LIN_SPEED = 0.5;
  const ANG_SPEED = 1.0;
  const LOOP_RATE_MS = 100; // 10Hz

  // Mutable state for the control loop
  const commandRef = useRef({ lin: 0, ang: 0 });
  const intervalRef = useRef<NodeJS.Timeout | null>(null);

  // Use a ref for the callback to keep the loop stable but always using freshest function
  const onControlRef = useRef(onControl);
  useEffect(() => {
    onControlRef.current = onControl;
  }, [onControl]);

  // Start the heartbeat loop
  const startLoop = useCallback(() => {
    if (intervalRef.current) return;
    intervalRef.current = setInterval(() => {
      onControlRef.current(commandRef.current.lin, commandRef.current.ang);
    }, LOOP_RATE_MS);
  }, []);

  // Stop the heartbeat loop
  const stopLoop = useCallback(() => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    onControlRef.current(0, 0); // Send final stop
  }, []);

  const updateCommand = useCallback(
    (lin: number, ang: number) => {
      commandRef.current = { lin, ang };
      startLoop();
    },
    [startLoop]
  );

  const handleStop = useCallback(() => {
    commandRef.current = { lin: 0, ang: 0 };
    stopLoop();
  }, [stopLoop]);

  // ─── Keyboard Controls (Supports Diagonals) ───────────────────
  const activeKeys = useRef<Set<string>>(new Set());

  const updateFromKeys = useCallback(() => {
    let lin = 0;
    let ang = 0;

    const keys = activeKeys.current;
    if (keys.has("w") || keys.has("arrowup")) lin += LIN_SPEED;
    if (keys.has("s") || keys.has("arrowdown")) lin -= LIN_SPEED;
    if (keys.has("a") || keys.has("arrowleft")) ang += ANG_SPEED;
    if (keys.has("d") || keys.has("arrowright")) ang -= ANG_SPEED;

    if (lin === 0 && ang === 0) {
       handleStop();
    } else {
       updateCommand(lin, ang);
    }
  }, [updateCommand, handleStop]);

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Don't capture keys when user is typing in an input
      if (document.activeElement?.tagName === "INPUT" || document.activeElement?.tagName === "TEXTAREA") return;
      
      const key = e.key.toLowerCase();
      const validKeys = ["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"];
      
      if (validKeys.includes(key)) {
         e.preventDefault();
         if (!activeKeys.current.has(key)) {
            activeKeys.current.add(key);
            updateFromKeys();
         }
      } else if (key === " " || key === "x") {
         e.preventDefault();
         activeKeys.current.clear(); // Clear all keys on emergency stop
         handleStop();
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      if (activeKeys.current.has(key)) {
         activeKeys.current.delete(key);
         updateFromKeys();
      }
    };

    // Safety: if window loses focus, stop everything
    const handleBlur = () => {
      activeKeys.current.clear();
      handleStop();
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    window.addEventListener("blur", handleBlur);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      window.removeEventListener("blur", handleBlur);
      stopLoop();
    };
  }, [updateFromKeys, handleStop, stopLoop]);

  // ─── Button Handlers ──────────────────────────────────────────
  // Track which buttons are currently pressed for proper diagonal support
  const pressedButtons = useRef<Set<string>>(new Set());

  const handleButtonDown = useCallback((direction: string) => {
    pressedButtons.current.add(direction);
    
    let lin = 0;
    let ang = 0;
    if (pressedButtons.current.has("forward")) lin += LIN_SPEED;
    if (pressedButtons.current.has("backward")) lin -= LIN_SPEED;
    if (pressedButtons.current.has("left")) ang += ANG_SPEED;
    if (pressedButtons.current.has("right")) ang -= ANG_SPEED;
    
    updateCommand(lin, ang);
  }, [updateCommand]);

  const handleButtonUp = useCallback((direction: string) => {
    pressedButtons.current.delete(direction);
    
    if (pressedButtons.current.size === 0) {
      handleStop();
    } else {
      // Other buttons still pressed — recompute
      let lin = 0;
      let ang = 0;
      if (pressedButtons.current.has("forward")) lin += LIN_SPEED;
      if (pressedButtons.current.has("backward")) lin -= LIN_SPEED;
      if (pressedButtons.current.has("left")) ang += ANG_SPEED;
      if (pressedButtons.current.has("right")) ang -= ANG_SPEED;
      updateCommand(lin, ang);
    }
  }, [handleStop, updateCommand]);

  return (
    <div className="flex flex-col rounded-xl border border-white/10 bg-black/40 backdrop-blur-xl shadow-2xl overflow-hidden">
      <div className="flex items-center justify-between px-4 py-3 border-b border-white/5 bg-white/5">
        <span className="text-[11px] font-inter font-bold tracking-widest uppercase text-white opacity-90">Manual Override</span>
        <div className="flex items-center gap-2 px-2 py-1 rounded bg-black/20">
           <span className="text-[9px] font-inter font-semibold uppercase tracking-wider text-zinc-400">Keyboard Active</span>
        </div>
      </div>
      <div
        style={{ padding: "1.25rem" }}
        className="flex-1 flex flex-col items-center justify-center gap-3"
      >
        {/* Forward */}
        <button
          className="w-12 h-12 flex items-center justify-center border border-[var(--color-primary)]/40 text-[var(--color-primary)] transition-all active:scale-95 hover:bg-[var(--color-primary)]/20 hover:border-[var(--color-primary)] hover:shadow-[0_0_20px_rgba(16,185,129,0.5)] disabled:opacity-50"
          style={{ borderRadius: "0.5rem", background: "rgba(16, 185, 129, 0.1)" }}
          onMouseDown={() => handleButtonDown("forward")}
          onMouseUp={() => handleButtonUp("forward")}
          onMouseLeave={() => handleButtonUp("forward")}
          onTouchStart={() => handleButtonDown("forward")}
          onTouchEnd={() => handleButtonUp("forward")}
          title="Drive Forward (W / ↑)"
        >
          <ArrowUp size={20} />
        </button>

        <div className="flex gap-3">
          {/* Left */}
          <button
            className="w-12 h-12 flex items-center justify-center border border-[var(--color-primary)]/40 text-[var(--color-primary)] transition-all active:scale-95 hover:bg-[var(--color-primary)]/20 hover:border-[var(--color-primary)] hover:shadow-[0_0_20px_rgba(16,185,129,0.5)]"
            style={{ borderRadius: "0.5rem", background: "rgba(16, 185, 129, 0.1)" }}
            onMouseDown={() => handleButtonDown("left")}
            onMouseUp={() => handleButtonUp("left")}
            onMouseLeave={() => handleButtonUp("left")}
            onTouchStart={() => handleButtonDown("left")}
            onTouchEnd={() => handleButtonUp("left")}
            title="Turn Left (A / ←)"
          >
            <ArrowLeft size={20} />
          </button>

          {/* Stop */}
          <button
            className="w-12 h-12 flex items-center justify-center border border-destructive/50 text-red-400 transition-all active:scale-90 font-inter font-bold text-[10px] uppercase tracking-wider hover:bg-destructive/20 hover:shadow-[0_0_20px_rgba(239,68,68,0.5)] hover:border-destructive"
            style={{ borderRadius: "0.5rem", background: "rgba(239, 68, 68, 0.1)" }}
            onClick={() => {
              pressedButtons.current.clear();
              handleStop();
            }}
            title="Emergency Stop (Space / X)"
          >
            STOP
          </button>

          {/* Right */}
          <button
            className="w-12 h-12 flex items-center justify-center border border-[var(--color-primary)]/40 text-[var(--color-primary)] transition-all active:scale-95 hover:bg-[var(--color-primary)]/20 hover:border-[var(--color-primary)] hover:shadow-[0_0_20px_rgba(16,185,129,0.5)]"
            style={{ borderRadius: "0.5rem", background: "rgba(16, 185, 129, 0.1)" }}
            onMouseDown={() => handleButtonDown("right")}
            onMouseUp={() => handleButtonUp("right")}
            onMouseLeave={() => handleButtonUp("right")}
            onTouchStart={() => handleButtonDown("right")}
            onTouchEnd={() => handleButtonUp("right")}
            title="Turn Right (D / →)"
          >
            <ArrowRight size={20} />
          </button>
        </div>

        {/* Backward */}
        <button
          className="w-12 h-12 flex items-center justify-center border border-[var(--color-primary)]/40 text-[var(--color-primary)] transition-all active:scale-95 hover:bg-[var(--color-primary)]/20 hover:border-[var(--color-primary)] hover:shadow-[0_0_20px_rgba(16,185,129,0.5)]"
          style={{ borderRadius: "0.5rem", background: "rgba(16, 185, 129, 0.1)" }}
          onMouseDown={() => handleButtonDown("backward")}
          onMouseUp={() => handleButtonUp("backward")}
          onMouseLeave={() => handleButtonUp("backward")}
          onTouchStart={() => handleButtonDown("backward")}
          onTouchEnd={() => handleButtonUp("backward")}
          title="Drive Reverse (S / ↓)"
        >
          <ArrowDown size={20} />
        </button>
      </div>
    </div>
  );
}
