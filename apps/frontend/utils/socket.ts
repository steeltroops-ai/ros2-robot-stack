"use client";

import { io, Socket } from "socket.io-client";

// Lazy Singleton Socket Instance
// Prevents connection attempts during SSR / when backend is offline
// Only connects when explicitly needed
const SOCKET_URL = process.env.NEXT_PUBLIC_SOCKET_URL || "http://localhost:4000";

let _socket: Socket | null = null;

export function getSocket(): Socket {
  if (!_socket) {
    _socket = io(SOCKET_URL, {
      transports: ["websocket"],
      autoConnect: false, // Don't connect until we explicitly call .connect()
      reconnectionAttempts: 5,
      reconnectionDelay: 2000,
      reconnectionDelayMax: 10000,
      timeout: 5000,
    });
  }
  return _socket;
}

// Legacy export — use getSocket() instead.
// This is a property getter so it's SSR-safe; only accesses the socket
// when code actually reads `socket`, not at module load time.
export const socket = typeof window !== "undefined" ? getSocket() : (null as unknown as Socket);

