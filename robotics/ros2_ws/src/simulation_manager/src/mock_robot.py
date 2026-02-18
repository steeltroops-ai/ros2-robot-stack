import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Pose
import math
import random
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class MockRobot(Node):

    def __init__(self, robot_id='robot_1'):
        super().__init__(f'mock_robot_{robot_id}')

        # Declare parameters
        self.declare_parameter('robot_id', robot_id)
        self.declare_parameter('initial_x', 2.0)
        self.declare_parameter('initial_y', 2.0)
        self.declare_parameter('publish_map_tf', True)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        initial_x = self.get_parameter('initial_x').get_parameter_value().double_value
        initial_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.publish_map_tf = self.get_parameter('publish_map_tf').get_parameter_value().bool_value

        print(f"--- [DEBUG] Global Node Name: {self.get_name()} ---")
        print(f"--- [DEBUG] Main Odom Topic: /{self.robot_id}/odom ---")
        print(f"--- [DEBUG] Manual Teleop: /{self.robot_id}/cmd_vel_manual ---")

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, f'/{self.robot_id}/odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, f'/{self.robot_id}/battery', 10)
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            f'/{self.robot_id}/map',
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
            )
        )

        # Subscribers
        self.create_subscription(Twist, f'/{self.robot_id}/cmd_vel', self.nav_cmd_vel_callback, 10)
        self.create_subscription(Twist, f'/{self.robot_id}/cmd_vel_manual', self.manual_cmd_vel_callback, 10)

        # Simulation State
        self.x = initial_x
        self.y = initial_y
        self.theta = 0.0

        # Robot Velocity State
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # Targets for acceleration ramping
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        self.last_manual_time = 0.0

        self.battery = 100.0
        self.battery_drain_rate = 0.05

        # Generate Map Once
        self.map_msg = self.generate_map()

        # Loop Rate
        self.timer_period = 0.033  # 30Hz
        self.timer = self.create_timer(self.timer_period, self.physics_callback)  # 30Hz Physics
        self.create_timer(1.0, self.map_callback)  # 1Hz Map Broadcast

        self.get_logger().info(f'Mock Robot "{self.robot_id}" Started. Map Generated.')

    def generate_map(self):
        # Create a simple 20x20m map with walls
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.1  # 10cm per pixel
        grid.info.width = 100  # 10m width
        grid.info.height = 100  # 10m height
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0

        # Data: -1 = unknown, 0 = free, 100 = occupied
        # Initialize with free space (0)
        data = [0] * (grid.info.width * grid.info.height)

        # Draw Walls
        w, h = grid.info.width, grid.info.height
        for i in range(w):
            data[i] = 100  # Bottom wall
            data[(h-1)*w + i] = 100  # Top wall
        for i in range(h):
            data[i*w] = 100  # Left wall
            data[i*w + (w-1)] = 100  # Right wall

        # Draw some obstacles
        for i in range(30, 40):
            for j in range(30, 40):
                data[j*w + i] = 100

        grid.data = data
        return grid

    def nav_cmd_vel_callback(self, msg):
        # Only use Nav2 velocity if we haven't had manual input in the last 0.5s
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_manual_time > 0.5:
            if abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01:
                # Log occasionally (every 10th message or so) to avoid spam
                if getattr(self, '_nav_log_count', 0) % 10 == 0:
                    print(f"--- [NAV2] Recv CMD: v={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, w={msg.angular.z:.2f} (Heading: {self.theta:.2f} rad) ---")
                self._nav_log_count = getattr(self, '_nav_log_count', 0) + 1

            self.target_vx = msg.linear.x
            self.target_vy = msg.linear.y
            self.target_wz = msg.angular.z

    def manual_cmd_vel_callback(self, msg):
        print(f"--- [TELEOP] Recv CMD: v={msg.linear.x:.2f}, w={msg.angular.z:.2f} ---")
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_wz = msg.angular.z
        self.last_manual_time = self.get_clock().now().nanoseconds / 1e9

    def map_callback(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def physics_callback(self):
        # 1. Physics Update (Momentum & Omnidirectional)
        dt = self.timer_period

        # Acceleration Parameters (m/s^2)
        # Non-Holonomic Constraint: Explicitly ignore lateral velocity (Car-mode)
        self.target_vy = 0.0
        self.vy = 0.0

        ACCEL = 2.0  # Snappier m/s^2
        ANG_ACCEL = 4.0  # Snappier rad/s^2

        # Ramp velocities towards targets
        dvx = self.target_vx - self.vx
        if abs(dvx) > ACCEL * dt:
            self.vx += math.copysign(ACCEL * dt, dvx)
        else:
            self.vx = self.target_vx

        dwz = self.target_wz - self.wz
        if abs(dwz) > ANG_ACCEL * dt:
            self.wz += math.copysign(ANG_ACCEL * dt, dwz)
        else:
            self.wz = self.target_wz

        # Precise Kinematic Integration (Arc Integration)
        # We calculate the change in position based on the average heading during this step
        avg_theta = self.theta + (self.wz * dt / 2.0)

        # Transform local velocity to global (vy is locked to 0)
        v_global_x = self.vx * math.cos(avg_theta)
        v_global_y = self.vx * math.sin(avg_theta)

        # Update Pose
        self.x += v_global_x * dt
        self.y += v_global_y * dt
        self.theta += self.wz * dt

        # Standardize theta [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        if abs(self.vx) > 0.01 or abs(self.vy) > 0.01 or abs(self.wz) > 0.01:
            print(f"--- [MOTION] Pose: ({self.x:.2f}, {self.y:.2f}) Theta: {self.theta:.2f} (v={self.vx:.2f}, vy={self.vy:.2f}, w={self.wz:.2f}) ---")

        # Collision Check (Simple bounds)
        if self.x < 0.2: self.x = 0.2
        if self.x > 9.8: self.x = 9.8
        if self.y < 0.2: self.y = 0.2
        if self.y > 9.8: self.y = 9.8

        # 2. Battery Update
        drain = self.battery_drain_rate * 0.1
        if abs(self.vx) > 0.1 or abs(self.wz) > 0.1:
            drain *= 2.0

        self.battery -= drain
        if self.battery < 0:
            self.battery = 100.0

        # 3. Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = f"{self.robot_id}/odom"
        odom_msg.child_frame_id = f"{self.robot_id}/base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(odom_msg)

        # 4. Publish Battery
        bat_msg = BatteryState()
        bat_msg.percentage = self.battery
        bat_msg.voltage = 24.0
        bat_msg.current = 2.5
        bat_msg.charge = self.battery

        self.battery_pub.publish(bat_msg)

        # 5. Publish TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f"{self.robot_id}/odom"
        t.child_frame_id = f"{self.robot_id}/base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

        # 6. Publish Static TF (map -> odom) if enabled
        if self.publish_map_tf:
            t_static = TransformStamped()
            t_static.header.stamp = self.get_clock().now().to_msg()
            t_static.header.frame_id = "map"
            t_static.child_frame_id = f"{self.robot_id}/odom"

            t_static.transform.translation.x = 0.0
            t_static.transform.translation.y = 0.0
            t_static.transform.translation.z = 0.0
            t_static.transform.rotation.x = 0.0
            t_static.transform.rotation.y = 0.0
            t_static.transform.rotation.z = 0.0
            t_static.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t_static)

        # 7. Publish Fake LiDAR Scan (10Hz throttle inside this 30Hz loop)
        # Simple counter to downsample
        if not hasattr(self, 'scan_tick'): self.scan_tick = 0
        self.scan_tick += 1
        if self.scan_tick >= 3:
            self.scan_tick = 0
            self.publish_fake_scan()

    def publish_fake_scan(self):
        from sensor_msgs.msg import LaserScan
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = f"{self.robot_id}/base_link"  # Mounted on robot

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree resolution
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0

        num_readings = 360
        ranges = []

        # Room bounds (from match generate_map)
        # 0 to 10m in X and Y
        room_min_x, room_max_x = 0.0, 10.0
        room_min_y, room_max_y = 0.0, 10.0

        for i in range(num_readings):
            # Angle in global frame
            angle_rel = scan.angle_min + i * scan.angle_increment
            angle_global = self.theta + angle_rel

            # Simple Raycast against 4 walls
            # Ray: origin (x,y), dir (cos, sin)
            dx = math.cos(angle_global)
            dy = math.sin(angle_global)

            dist = 10.0  # Max range

            # Check vertical walls (x = 0, x = 10)
            if dx != 0:
                # Wall X=0
                d = (room_min_x - self.x) / dx
                if d > 0 and d < dist: dist = d
                # Wall X=10
                d = (room_max_x - self.x) / dx
                if d > 0 and d < dist: dist = d

            # Check horizontal walls (y = 0, y = 10)
            if dy != 0:
                # Wall Y=0
                d = (room_min_y - self.y) / dy
                if d > 0 and d < dist: dist = d
                # Wall Y=10
                d = (room_max_y - self.y) / dy
                if d > 0 and d < dist: dist = d

            # Add obstacles (simple box at 3,3 to 4,4)
            # This is hard to do efficiently in python loop, let's skip obstacles for now
            # Just add some noise
            dist += random.gauss(0, 0.02)

            ranges.append(dist)

        scan.ranges = ranges

        # Create publisher if not exists
        if not hasattr(self, 'scan_pub'):
            self.scan_pub = self.create_publisher(LaserScan, f'/{self.robot_id}/scan', 10)

        self.scan_pub.publish(scan)

from rclpy.executors import ExternalShutdownException

def main(args=None):
    rclpy.init(args=args)

    # Improved arg parsing for robot_id from ROS args
    import sys
    robot_id = 'robot_1'
    for i, arg in enumerate(sys.argv):
        if 'robot_id:=' in arg:
            robot_id = arg.split(':=')[1]
        elif arg == '-p' and i + 1 < len(sys.argv) and 'robot_id:=' in sys.argv[i+1]:
            robot_id = sys.argv[i+1].split(':=')[1]

    print(f"--- DEBUG: Initializing Mock Robot with ID: {robot_id} ---")
    node = MockRobot(robot_id=robot_id)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
