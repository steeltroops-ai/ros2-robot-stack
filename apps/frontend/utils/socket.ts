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

// Legacy export for backward compatibility — returns the lazy singleton
// This is safe because autoConnect is false; callers must .connect() explicitly
export const socket: Socket = typeof window !== "undefined" ? getSocket() : (null as unknown as Socket);
