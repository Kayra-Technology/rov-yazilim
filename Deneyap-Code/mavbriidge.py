#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from std_msgs.msg import Bool, String, Float32

from pymavlink import mavutil

R_EARTH = 6378137.0  # meters

def ned_to_geodetic(x, y, z, origin_lat, origin_lon, origin_alt):
    """Convert NED (meters) to lat/lon/alt relative to origin (degrees/meters)."""
    d_lat = (x / R_EARTH) * (180.0 / math.pi)               # N (x) → latitude
    d_lon = (y / (R_EARTH * math.cos(math.radians(origin_lat)))) * (180.0 / math.pi)  # E (y) → longitude
    lat = origin_lat + d_lat
    lon = origin_lon + d_lon
    alt = origin_alt - z  # z is down in NED
    return lat, lon, alt

def euler_to_quaternion(roll, pitch, yaw):
    """Convert ENU Euler to quaternion (x,y,z,w)."""
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    qx = sr*cp*cy - cr*sp*cy
    qy = cr*sp*sy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    qw = cr*cp*cy + sr*sp*sy
    return qx, qy, qz, qw

def wrap_pi(angle):
    """Wrap to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

class MavlinkBridge(Node):
    def _init_(self):
        super()._init_('mavlink_bridge')
        self.get_logger().info("INIT MAVLINK BRIDGE")

        # ── Parameters
        self.declare_parameter('connection', '/dev/ttyACM0')      # e.g. '/dev/ttyU 'udp:0.0.0.0:14550'SB0' or
        self.declare_parameter('baud', 57600)
        self.declare_parameter('publish_local_global', True)

        connection = self.get_parameter('connection').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.publish_local_global = self.get_parameter('publish_local_global').get_parameter_value().bool_value

        # ── Publishers
        self.gps_pub = self.create_publisher(NavSatFix,    '/bq/global_position', 10)
        self.local_pub = self.create_publisher(PointStamped, '/bq/local_position', 10)
        self.imu_pub = self.create_publisher(Imu,          '/bq/imu', 10)
        self.heading_pub = self.create_publisher(Float32,  '/bq/heading', 10)  # degrees [0..360)
        self.armed_pub = self.create_publisher(Bool,       '/bq/armed', 10)
        self.mode_pub = self.create_publisher(String,      '/bq/mode', 10)
        self.target_vel_sub = self.create_subscription(String,      '/target_vel', target_vel_callback, 10)
        self.target_heading_sub = self.create_publisher(String,      '/target_heading', 10)
        self.target_wp_sub = self.create_publisher(String,      '/target_wp', 10)

        # ── MAVLink connection
        try:
            if connection.startswith('udp:') or connection.startswith('tcp:'):
                self.master = mavutil.mavlink_connection(connection)
            else:
                self.master = mavutil.mavlink_connection(connection, baud=baud)

            self.master.wait_heartbeat()
            self.get_logger().info(f"Connected to vehicle via {connection}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            raise

        # ── EKF Origin (for LOCAL_POSITION_NED → lat/lon/alt)
        self.origin_lat = 0.0
        self.origin_lon = 0.0
        self.origin_alt = 0.0

        try:
            origin = self.master.recv_match(type='GPS_GLOBAL_ORIGIN', blocking=True, timeout=5)
            if origin:
                self.origin_lat = origin.latitude / 1e7
                self.origin_lon = origin.longitude / 1e7
                self.origin_alt = origin.altitude / 1000.0
                self.get_logger().info(f"EKF Origin: {self.origin_lat:.7f}, {self.origin_lon:.7f}, {self.origin_alt:.2f} m")
            else:
                self.get_logger().warn("No GPS_GLOBAL_ORIGIN received; will still publish LOCAL_POSITION_NED raw.")
        except Exception:
            self.get_logger().warn("No GPS_GLOBAL_ORIGIN; using 0,0,0")

        # ── Timers
        self.create_timer(0.02, self.poll_fast)  # 50 Hz: ATTITUDE/HEARTBEAT
        #self.create_timer(0.05, self.poll_slow)  # 20 Hz: LOCAL_POSITION_NED, VFR_HUD

    # ──────────────────────────────────────────────────────────────────────────
    # FAST LOOP: attitude, heartbeat → IMU, armed/mode
    def poll_fast(self):
        # Drain a few messages each tick
        for _ in range(10):
            msg = self.master.recv_msg()
            if msg is None:
                continue
            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                mode = mavutil.mode_string_v10(msg)
                self.armed_pub.publish(Bool(data=armed))
                self.mode_pub.publish(String(data=mode))
                if mode == "XYZ":
                    #başlat 2. parkuru
                    #buraya değil ama, gn5'i geçtikten sonra parkur 3 moduna al
                    pass

            elif msg_type == "ATTITUDE":
                # ATTITUDE: roll/pitch/yaw are in radians in NED
                roll_ned = msg.roll
                pitch_ned = msg.pitch
                yaw_ned = msg.yaw

                # Convert to ENU convention for ROS:
                # relationship: yaw_enu = pi/2 - yaw_ned
                yaw_enu = wrap_pi((math.pi / 2.0) - yaw_ned)
                # roll/pitch mapping between NED↔ENU is subtle; for marine use-cases
                # consumers typically care about yaw. We map them approximately:
                # roll_enu ≈ pitch_ned, pitch_enu ≈ roll_ned (best-effort for small angles).
                roll_enu = pitch_ned
                pitch_enu = roll_ned

                qx, qy, qz, qw = euler_to_quaternion(roll_enu, pitch_enu, yaw_enu)

                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'base_link'
                imu_msg.orientation.x = qx
                imu_msg.orientation.y = qy
                imu_msg.orientation.z = qz
                imu_msg.orientation.w = qw

                # If you want angular velocities / linear accels, you can also
                # parse SCALED_IMU* messages. We leave them zeroed for now.
                imu_msg.orientation_covariance[0] = 0.05  # rough covariance
                imu_msg.orientation_covariance[4] = 0.05
                imu_msg.orientation_covariance[8] = 0.05

                self.imu_pub.publish(imu_msg)

            elif msg_type == "VFR_HUD":
                # Heading in degrees [0..360)
                self.heading_pub.publish(Float32(data=float(msg.heading)))
            elif msg_type == 'MISSION_CURRENT':
                if msg and msg.seq == 0:
                    self.is_first_parkour_finished = True

            now = self.get_clock().now().to_msg()
            got_local = False

            # Read the most recent LOCAL_POSITION_NED (non-blocking)
            if msg_type == "LOCAL_POSITION_NED":
                got_local = True
                x, y, z = float(msg.x), float(msg.y), float(msg.z)

                local_msg = PointStamped()
                local_msg.header.stamp = now
                local_msg.header.frame_id = 'ekf_origin_ned'  # NED frame
                local_msg.point.x = x   # North (m)
                local_msg.point.y = y   # East  (m)
                local_msg.point.z = z   # Down  (m)
                self.local_pub.publish(local_msg)

                if self.publish_local_global and (self.origin_lat != 0.0 or self.origin_lon != 0.0 or self.origin_alt != 0.0):
                    lat, lon, alt = ned_to_geodetic(x, y, z, self.origin_lat, self.origin_lon, self.origin_alt)
                    nav = NavSatFix()
                    nav.header.stamp = now
                    nav.header.frame_id = 'earth'
                    nav.status.status = NavSatStatus.STATUS_FIX
                    nav.status.service = NavSatStatus.SERVICE_GPS
                    nav.latitude = lat
                    nav.longitude = lon
                    nav.altitude = alt
                    # Covariances unknown: set to -1 or a rough value if available from GPS
                    nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    self.gps_pub.publish(nav)

            # Optionally, you can also read GPS_RAW_INT to publish raw GPS if needed.
            # We prioritize EKF global above for navigation stability.

def target_vel_callback(msg):
    pass
def target_heading_callback(msg):
    pass
def target_wp_callback(msg):
    pass
def enable_guided_mode(connection):
    connection.mav.set_mode_send(
    connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4 # GUIDED = 4
    )

def arm_ship(connection):
    connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0 # 1 = ARM
    )

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if _name_ == '_main_':
    main()