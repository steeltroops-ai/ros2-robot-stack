"use client";

import Link from "next/link";
import { motion } from "framer-motion";
import { RiHome5Line, RiAlertLine } from "react-icons/ri";

export default function NotFound() {
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
             <h1 className="text-[12rem] font-black text-black leading-none tracking-tighter opacity-5">404</h1>
             <div className="absolute top-1/2 -translate-y-1/2 flex flex-col items-center gap-4">
               <RiAlertLine size={64} className="text-zinc-900" />
               <h2 className="text-4xl font-black text-black tracking-tight">PAGE NOT FOUND</h2>
             </div>
          </motion.div>
        </div>

        <p className="text-base text-zinc-500 max-w-md mx-auto leading-relaxed font-medium">
          The coordinates for this sector do not exist in the Omniverse registry. 
          Return to base for telemetry recalibration.
        </p>

        <div className="flex justify-center pt-4">
          <Link
            href="/"
            className="group inline-flex items-center justify-center gap-2.5 w-52 py-3.5 rounded-full bg-black text-white hover:opacity-90 transition-all duration-300 shadow-sm"
          >
            <RiHome5Line className="w-4 h-4 group-hover:-translate-y-0.5 transition-transform duration-300" />
            <span className="text-sm font-bold tracking-wide">Return to Base</span>
          </Link>
        </div>
      </motion.div>

      <div className="absolute bottom-8 text-center text-[10px] uppercase font-black tracking-[0.3em] text-zinc-600">
        Omniverse System Protocol • 404 Sector Not Found
      </div>
    </div>
  );
}
