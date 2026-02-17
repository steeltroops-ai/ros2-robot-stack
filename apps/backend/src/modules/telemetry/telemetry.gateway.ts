import rclnodejs from "rclnodejs";
import { Server, Socket } from "socket.io";

// --- Shared Type Definition (Should be in shared-types, mirroring here for speed) ---
interface RobotTelemetry {
  id: string;
  x: number;
  y: number;
  theta: number; // yaw
  battery: number;
}

export class TelemetryGateway {
  private io: Server;
  private node: rclnodejs.Node | null = null;
  private robots: Map<string, RobotTelemetry> = new Map();
  private lastUpdate: Map<string, number> = new Map();

  constructor(io: Server) {
    this.io = io;
  }

  public async init() {
    console.log("[Telemetry] Initializing ROS 2 Node...");

    try {
      await rclnodejs.init();
      this.node = new rclnodejs.Node("backend_telemetry_bridge");

      this.setupSubscribers();

      // Start spinning
      this.node.spin();
      console.log("[Telemetry] ROS 2 Bridge Active.");
    } catch (e) {
      console.error("[Telemetry] Failed to init ROS 2:", e);
    }
  }

  private setupSubscribers() {
    if (!this.node) return;

    // TODO: Dynamic discovery. For now, hardcode 'robot_1'.
    const robotId = "robot_1";

    // 1. Subscribe to Odom
    this.node.createSubscription(
      "nav_msgs/msg/Odometry",
      "/odom", // Mock Robot publishes here
      (msg: any) => this.handleOdom(robotId, msg),
    );

    // 2. Subscribe to Battery
    this.node.createSubscription(
      "sensor_msgs/msg/BatteryState",
      "/battery",
      (msg: any) => this.handleBattery(robotId, msg),
    );
  }

  private handleOdom(robotId: string, msg: any) {
    // Throttle: Only process if > 50ms has passed (20Hz max)
    const now = Date.now();
    const last = this.lastUpdate.get(robotId) || 0;
    if (now - last < 50) return;

    // Extract Position
    const x = msg.pose.pose.position.x;
    const y = msg.pose.pose.position.y;

    // Extract Yaw (Quaternion to Euler - Simplified Z rotation)
    const qz = msg.pose.pose.orientation.z;
    const qw = msg.pose.pose.orientation.w;
    const theta = 2 * Math.atan2(qz, qw);

    // Update Internal State
    const current = this.robots.get(robotId) || {
      id: robotId,
      x: 0,
      y: 0,
      theta: 0,
      battery: 0,
    };
    current.x = x;
    current.y = y;
    current.theta = theta;

    this.robots.set(robotId, current);
    this.lastUpdate.set(robotId, now);

    // Emit to Frontend
    this.io.emit("telemetry", current);
  }

  private handleBattery(robotId: string, msg: any) {
    const current = this.robots.get(robotId);
    if (current) {
      current.battery = msg.percentage;
      this.robots.set(robotId, current);
      // We don't emit on battery alone, we wait for next odom pulse to bundle it
      // or emit immediately if unrelated. Let's emit to be safe.
      this.io.emit("telemetry", current);
    }
  }
}
