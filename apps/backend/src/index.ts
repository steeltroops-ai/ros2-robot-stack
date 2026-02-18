import Fastify from "fastify";
import { Server } from "socket.io";


const fastify = Fastify({
  logger: { level: "warn" },
});

const io = new Server(fastify.server, {
  cors: {
    origin: process.env.FRONTEND_URL || "*",
    methods: ["GET", "POST"],
  },
});

// Load Modules
import { TelemetryGateway } from "./modules/telemetry/telemetry.gateway";
const telemetry = new TelemetryGateway(io);

// Expose API for debugging
fastify.get("/api/robots", async (request, reply) => {
  return telemetry.getRobots();
});

// Start
const start = async () => {
  try {
    const port = parseInt(process.env.PORT || "4000");
    await telemetry.init(); // Initialize ROS 2
    await fastify.listen({ port, host: "0.0.0.0" });
    console.log(`[Backend] System Active on Port ${port}`);
  } catch (err) {
    fastify.log.error(err);
    process.exit(1);
  }
};
start();
