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

  private cmdVelPubs: Map<string, rclnodejs.Publisher<any>> = new Map();
  private goalPubs: Map<string, rclnodejs.Publisher<any>> = new Map();
  private initialPosePubs: Map<string, rclnodejs.Publisher<any>> = new Map();
  private initializedRobots: Set<string> = new Set();

  // ... (robots map) ...
  
  constructor(io: Server) {
    this.io = io;
  }

  public getRobots() {
    return Array.from(this.robots.values());
  }

  public async init() {
    console.log("[Telemetry] Initializing ROS 2 Node...");

    try {
      await rclnodejs.init();
      this.node = new rclnodejs.Node("backend_telemetry_bridge");

      // 0. Setup Publishers (Dynamic now)
      // We don't pre-create cmd_vel publisher for single robot anymore.
      // We create them on demand or on discovery.

      this.setupSubscribers();
      this.setupSocketListeners();

      // Start spinning
      this.node.spin();
      console.log("[Telemetry] ROS 2 Bridge Active.");
    } catch (e) {
      console.error("[Telemetry] Failed to init ROS 2:", e);
    }
  }

  private setupSocketListeners() {
    this.io.on("connection", (socket) => {
      console.log(`[Telemetry] Client connected: ${socket.id}`);
      
      socket.on("subscribe", (room: string) => {
        console.log(`[Telemetry] Client ${socket.id} joined ${room}`);
        socket.join(room);
      });

      socket.on("unsubscribe", (room: string) => {
        // console.log(`[Telemetry] Client ${socket.id} left ${room}`);
        socket.leave(room);
      });

      socket.on("control", (data: { robotId: string; linear: number; angular: number }) => {
        // console.log(`[Socket] Control for ${data.robotId}: v=${data.linear}, w=${data.angular}`);
        this.publishCmdVel(data);
      });

      socket.on("navigate", (data: { robotId: string; x: number; y: number; theta: number }) => {
        this.publishGoal(data);
      });

      socket.on("disconnect", () => {
        console.log(`[Telemetry] Client disconnected: ${socket.id}`);
      });
    });
  }

  private publishGoal(data: { robotId: string; x: number; y: number; theta: number }) {
    if (!this.node) return;

    let pub = this.goalPubs.get(data.robotId);
    if (!pub) {
        try {
            // Nav2 subscribes to /{namespace}/goal_pose
            pub = this.node.createPublisher(
                "geometry_msgs/msg/PoseStamped",
                `/${data.robotId}/goal_pose`
            );
            this.goalPubs.set(data.robotId, pub);
        } catch (e) {
            console.error(`[Telemetry] Failed to create goal publisher: ${e}`);
            return;
        }
    }

    // Convert Theta to Quaternion (approximate for 2D)
    // Yaw (Z rotation) -> Quaternion
    const c = Math.cos(data.theta / 2);
    const s = Math.sin(data.theta / 2);
    const q = { x: 0, y: 0, z: s, w: c };

    const msg = {
        header: {
            stamp: this.node.now(),
            frame_id: "map" // Goals are usually in map frame
        },
        pose: {
            position: { x: data.x, y: data.y, z: 0.0 },
            orientation: q
        }
    };
    
    // console.log(`[Navigation] Sending goal to ${data.robotId}: [${data.x}, ${data.y}]`);
    pub.publish(msg);
  }

  private publishCmdVel(data: { robotId: string; linear: number; angular: number }) {
    if (!this.node) return;
    
    // Get or Create Publisher for this robot
    let pub = this.cmdVelPubs.get(data.robotId);
    if (!pub) {
      console.log(`[Telemetry] Creating publisher for ${data.robotId}/cmd_vel`);
      try {
        pub = this.node.createPublisher(
          "geometry_msgs/msg/Twist",
          `/${data.robotId}/cmd_vel_manual`
        );
        this.cmdVelPubs.set(data.robotId, pub);
      } catch (e) {
        console.error(`[Telemetry] Failed to create publisher: ${e}`);
        return;
      }
    }

    // Create Twist Message
    const twist = {
      linear: { x: data.linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: data.angular },
    };

    // console.log(`[Control] Publishing to ${data.robotId}: v=${data.linear}, w=${data.angular}`);
    // console.log(`[Telemetry] Teleop for ${data.robotId}: lin=${data.linear.toFixed(2)}, ang=${data.angular.toFixed(2)}`);
    pub.publish(twist);
  }

  private setupSubscribers() {
    if (!this.node) return;

    // A. Global Log Subscription (Smart Filter)
    this.node.createSubscription(
      "rcl_interfaces/msg/Log",
      "/rosout",
      (msg: any) => this.handleLogs(msg)
    );

    const robots = ["robot_1", "robot_2", "robot_3"];

    robots.forEach((robotId) => {
      console.log(`[Telemetry] Subscribing to ${robotId} topics...`);

      // 1. Subscribe to Odom (Position)
      this.node?.createSubscription(
        "nav_msgs/msg/Odometry",
        `/${robotId}/odom`,
        (msg: any) => this.handleOdom(robotId, msg),
      );

      // 2. Subscribe to Battery
      this.node?.createSubscription(
        "sensor_msgs/msg/BatteryState",
        `/${robotId}/battery`,
        (msg: any) => this.handleBattery(robotId, msg),
      );

      // 3. Subscribe to Map
      // Map is HEAVY. Only send to 'robot:{id}' room.
      this.node?.createSubscription(
        "nav_msgs/msg/OccupancyGrid",
        `/${robotId}/map`,
        (msg: any) => this.handleMap(robotId, msg),
      );

      // 4. Subscribe to LiDAR Scan
      this.node?.createSubscription(
        "sensor_msgs/msg/LaserScan",
        `/${robotId}/scan`,
        (msg: any) => this.handleScan(robotId, msg)
      );

      // 5. Subscribe to Nav2 Plan (Global Path)
      this.node?.createSubscription(
        "nav_msgs/msg/Path",
        `/${robotId}/plan`,
        (msg: any) => this.handlePlan(robotId, msg)
      );
    });
  }

  private handlePlan(robotId: string, msg: any) {
    const room = `robot:${robotId}`;
    if (!this.io.sockets.adapter.rooms.get(room)?.size) return;

    // Path contains a list of poses.
    // We only need x, y for each pose to draw a line.
    const path = msg.poses.map((p: any) => ({
        x: p.pose.position.x,
        y: p.pose.position.y
    }));
    
    this.io.to(room).emit("plan", path);
  }

  private handleScan(robotId: string, msg: any) {
    const room = `robot:${robotId}`;
    if (!this.io.sockets.adapter.rooms.get(room)?.size) return;

    // LiDAR is high bandwidth (360 floats).
    // The frontend only needs it if the viewer is open.
    // We strictly send to 'robot:{id}' room.
    // Optimization: Compress or only send ranges?
    // For now, raw forwarding is fine for local/LAN.
    
    // Simplification: We only send ranges, angle_min, angle_increment
    // We strip header to save bytes
    const simplified = {
       ranges: msg.ranges,
       angle_min: msg.angle_min,
       angle_increment: msg.angle_increment,
       range_max: msg.range_max
    };
    
    this.io.to(`robot:${robotId}`).emit("scan", simplified);
  }

  private handleLogs(msg: any) {
    // Smart Filter: Only allow WARN (30) and ERROR (40) levels
    // Or if the node name contains a specific robot ID
    if (msg.level < 30) return; 

    const logEntry = {
      level: msg.level === 30 ? "WARN" : msg.level >= 40 ? "ERROR" : "INFO",
      msg: msg.msg,
      name: msg.name,
      timestamp: Date.now()
    };

    // Broadcast to 'diagnostics' room only
    this.io.to("diagnostics").emit("log", logEntry);
  }

  private handleMap(robotId: string, msg: any) {
    const room = `robot:${robotId}`;
    if (this.io.sockets.adapter.rooms.get(room)?.size) {
        this.io.to(room).emit("map", msg);
    }
  }

  private handleOdom(robotId: string, msg: any) {
    const now = Date.now();
    const last = this.lastUpdate.get(robotId) || 0;

    // Extract Data
    const x = msg.pose.pose.position.x;
    const y = msg.pose.pose.position.y;
    // Quaternion to Yaw
    const qz = msg.pose.pose.orientation.z;
    const qw = msg.pose.pose.orientation.w;
    const theta = 2 * Math.atan2(qz, qw);

    const data = {
      id: robotId,
      x,
      y,
      theta,
      battery: this.robots.get(robotId)?.battery || 100,
    };

    // Update internal state
    this.robots.set(robotId, data);

    // RATE LIMITING STRATEGY
    
    // 0. Auto-Init Pose for AMCL (One time)
    if (!this.initializedRobots.has(robotId)) {
        this.publishInitialPose(robotId, data);
        this.initializedRobots.add(robotId);
    }

    // 1. High Frequency (60Hz) -> To 'robot:{id}' (Active Viewer)
    if (now - last > 16) {
        this.io.to(`robot:${robotId}`).emit("telemetry", data);
        this.lastUpdate.set(robotId, now);

        // 2. Sampled Frequency (5Hz) -> To 'fleet' (Dashboard List)
        // We only send to fleet if it's been ~200ms
        if (Math.random() < 0.1) {
            this.io.to("fleet").emit("telemetry", data);
        }
    }
  }

  private publishInitialPose(robotId: string, data: any) {
    if (!this.node) return;
    let pub = this.initialPosePubs.get(robotId);
    if (!pub) {
        pub = this.node.createPublisher("geometry_msgs/msg/PoseWithCovarianceStamped", `/${robotId}/initialpose`);
        this.initialPosePubs.set(robotId, pub);
    }
    
    // Quaternion from theta
    const qz = Math.sin(data.theta / 2);
    const qw = Math.cos(data.theta / 2);

    const msg = {
        header: { stamp: this.node.now(), frame_id: "map" },
        pose: {
            pose: {
                position: { x: data.x, y: data.y, z: 0 },
                orientation: { x: 0, y: 0, z: qz, w: qw }
            },
            covariance: new Array(36).fill(0).map((_, i) => (i % 7 === 0 ? 0.25 : 0))
        }
    };
    console.log(`[Telemetry] Auto-initializing AMCL for ${robotId}`);
    pub.publish(msg);
  }

  private handleBattery(robotId: string, msg: any) {
    const current = this.robots.get(robotId);
    if (current) {
      current.battery = Math.round(msg.percentage);
      this.robots.set(robotId, current);
      // Battery is low freq, send to both
      this.io.to(`robot:${robotId}`).emit("telemetry", current);
      this.io.to("fleet").emit("telemetry", current);
    }
  }
}
