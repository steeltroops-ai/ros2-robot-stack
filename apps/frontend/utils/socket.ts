"use client";

import { io, Socket } from "socket.io-client";

// Singleton Socket Instance
// Prevent multiple connections in React Strict Mode or Fast Refresh
const SOCKET_URL = process.env.NEXT_PUBLIC_SOCKET_URL || "http://localhost:4000";

export const socket: Socket = io(SOCKET_URL, {
  transports: ["websocket"],
  autoConnect: true,
  reconnectionAttempts: 5,
});
