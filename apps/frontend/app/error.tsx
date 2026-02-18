"use client";

import { useEffect } from "react";
import { motion } from "framer-motion";
import { RiRefreshLine, RiHome5Line, RiAlertFill } from "react-icons/ri";

export default function GlobalError({
  error,
  reset,
}: {
  error: Error & { digest?: string };
  reset: () => void;
}) {
  useEffect(() => {
    // Log the error to an error reporting service
    console.error("Next.js App Router Error:", error);
  }, [error]);

  return (
    <div className="min-h-screen relative font-sans flex flex-col items-center justify-center bg-[#f5f5f7] overflow-hidden">
      {/* Background - Bento Light Mesh */}
      <div className="fixed top-0 w-full h-full -z-10">
        <div className="relative w-full h-full bg-[#f5f5f7]">
          <div className="absolute bottom-0 left-0 right-0 top-0 bg-[linear-gradient(to_right,#00000012_1px,transparent_1px),linear-gradient(to_bottom,#00000012_1px,transparent_1px)] bg-[size:14px_24px]"></div>
          <div className="absolute left-0 right-0 top-[-10%] h-[1000px] w-[1000px] rounded-full bg-[radial-gradient(circle_400px_at_50%_300px,#ffffff99,#f5f5f7)]"></div>
        </div>
      </div>

      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.6 }}
        className="text-center space-y-10 w-full max-w-2xl px-4"
      >
        <div className="space-y-4">
          <motion.div
            initial={{ scale: 0.5, opacity: 0 }}
            animate={{ scale: 1, opacity: 1 }}
            transition={{ delay: 0.1, type: "spring", stiffness: 100 }}
            className="flex flex-col items-center"
          >
             <h1 className="text-[10rem] font-black text-black leading-none tracking-tighter opacity-5">FATAL</h1>
             <div className="absolute top-1/2 -translate-y-1/2 flex flex-col items-center gap-4">
               <RiAlertFill size={64} className="text-red-500" />
               <h2 className="text-4xl font-black text-black tracking-tight">CRITICAL FAILURE</h2>
             </div>
          </motion.div>
        </div>

        <div className="space-y-3">
          <h3 className="text-xl font-bold text-zinc-900 leading-tight tracking-tight">
            {error.message || "An unexpected system breach occurred"}
          </h3>
          <p className="text-sm text-zinc-500 max-w-md mx-auto leading-relaxed font-medium">
            The application encountered a fatal exception during the render cycle.
            DDS telemetry has been suspended to prevent hardware drift.
          </p>
          {error.digest && (
            <div className="mt-4 flex flex-col items-center gap-1">
               <span className="text-[9px] uppercase font-black text-zinc-400 tracking-widest">Trace Identification</span>
               <span className="font-mono text-[10px] font-bold text-zinc-600 bg-white/50 px-2 py-0.5 rounded border border-zinc-200">{error.digest}</span>
            </div>
          )}
        </div>

        <div className="flex flex-col sm:flex-row gap-4 justify-center items-center pt-4">
          <button
            onClick={() => reset()}
            className="group relative inline-flex items-center justify-center gap-2.5 w-52 py-3.5 rounded-full bg-black text-white hover:opacity-90 transition-all duration-300 shadow-sm overflow-hidden"
          >
            <RiRefreshLine className="w-4 h-4 group-hover:rotate-180 transition-transform duration-700 ease-in-out" />
            <span className="text-sm font-bold tracking-wide">Perform Recovery</span>
          </button>
          
          <button
            onClick={() => (window.location.href = "/")}
            className="group inline-flex items-center justify-center gap-2.5 w-52 py-3.5 rounded-full bg-white text-zinc-900 border border-zinc-200 hover:bg-zinc-50 transition-all duration-300 shadow-sm"
          >
            <RiHome5Line className="w-4 h-4 group-hover:-translate-y-0.5 transition-transform duration-300" />
            <span className="text-sm font-bold tracking-wide">Return to Base</span>
          </button>
        </div>
      </motion.div>

      <div className="absolute bottom-8 text-center text-[10px] uppercase font-black tracking-[0.3em] text-zinc-600">
        Omniverse System Integrity Protocol • Critical Stabilized
      </div>
    </div>
  );
}
