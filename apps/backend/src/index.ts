import Fastify from "fastify";
import { Server } from "socket.io";


const fastify = Fastify({
  logger: true,
});

const io = new Server(fastify.server, {
  cors: {
    origin: "*",
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
    await telemetry.init(); // Initialize ROS 2
    await fastify.listen({ port: 4000, host: "0.0.0.0" });
    console.log("[Backend] System Active on Port 4000");
  } catch (err) {
    fastify.log.error(err);
    process.exit(1);
  }
};
start();
