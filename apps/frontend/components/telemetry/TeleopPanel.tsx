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
      console.log(`[Teleop] Dispatching: v=${commandRef.current.lin}, w=${commandRef.current.ang}`);
      onControlRef.current(commandRef.current.lin, commandRef.current.ang);
    }, LOOP_RATE_MS);
  }, []); // Stable loop

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

  // Keyboard controls
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
      if (document.activeElement?.tagName === "INPUT") return;
      
      const key = e.key.toLowerCase();
      const validKeys = ["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"];
      
      if (validKeys.includes(key)) {
         e.preventDefault(); // Prevent scroll
         if (!activeKeys.current.has(key)) {
            activeKeys.current.add(key);
            updateFromKeys();
         }
      } else if (key === " " || key === "x") {
         e.preventDefault();
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

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      stopLoop();
    };
  }, [updateFromKeys, handleStop, stopLoop]);

  return (
    <div className="panel flex flex-col">
      <div className="panel-header" style={{ padding: "0.75rem 1.25rem" }}>
        <span className="panel-title">Teleop Control</span>
        <div className="flex items-center gap-2">
           <span style={{ fontSize: 10, opacity: 0.5, fontWeight: 600 }}>WASD/Arrows</span>
        </div>
      </div>
      <div
        style={{ padding: "1.25rem" }}
        className="flex-1 flex flex-col items-center justify-center gap-3"
      >
        {/* Forward */}
        <button
          className="w-12 h-12 flex items-center justify-center bg-primary text-primary-foreground rounded-2xl transition-all active:scale-95 hover:shadow-lg disabled:opacity-50"
          onMouseDown={() => updateCommand(LIN_SPEED, 0)}
          onMouseUp={handleStop}
          onMouseLeave={handleStop}
          onTouchStart={() => updateCommand(LIN_SPEED, 0)}
          onTouchEnd={handleStop}
          title="Forward (W / Up)"
        >
          <ArrowUp size={20} />
        </button>

        <div className="flex gap-3">
          {/* Left */}
          <button
            className="w-12 h-12 flex items-center justify-center bg-primary text-primary-foreground rounded-2xl transition-all active:scale-95 hover:shadow-lg"
            onMouseDown={() => updateCommand(0, ANG_SPEED)}
            onMouseUp={handleStop}
            onMouseLeave={handleStop}
            onTouchStart={() => updateCommand(0, ANG_SPEED)}
            onTouchEnd={handleStop}
            title="Rotate Left (A / Left)"
          >
            <ArrowLeft size={20} />
          </button>

          {/* Stop */}
          <button
            className="w-12 h-12 flex items-center justify-center bg-destructive text-destructive-foreground rounded-2xl transition-all active:scale-90 shadow-md font-bold text-[0.6rem]"
            onClick={handleStop}
            title="Emergency Stop (Space)"
          >
            STOP
          </button>

          {/* Right */}
          <button
            className="w-12 h-12 flex items-center justify-center bg-primary text-primary-foreground rounded-2xl transition-all active:scale-95 hover:shadow-lg"
            onMouseDown={() => updateCommand(0, -ANG_SPEED)}
            onMouseUp={handleStop}
            onMouseLeave={handleStop}
            onTouchStart={() => updateCommand(0, -ANG_SPEED)}
            onTouchEnd={handleStop}
            title="Rotate Right (D / Right)"
          >
            <ArrowRight size={20} />
          </button>
        </div>

        {/* Backward */}
        <button
          className="w-12 h-12 flex items-center justify-center bg-primary text-primary-foreground rounded-2xl transition-all active:scale-95 hover:shadow-lg"
          onMouseDown={() => updateCommand(-LIN_SPEED, 0)}
          onMouseUp={handleStop}
          onMouseLeave={handleStop}
          onTouchStart={() => updateCommand(-LIN_SPEED, 0)}
          onTouchEnd={handleStop}
          title="Backward (S / Down)"
        >
          <ArrowDown size={20} />
        </button>
      </div>
    </div>
  );
}
