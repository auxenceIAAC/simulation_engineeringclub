"""
lidar_simulator_node.py — Simulated 2D LIDAR sensor

Simulates a 360° planar LIDAR mounted on the boat.  At 10 Hz it
casts 360 rays (1° resolution, 10 m max range) and computes the
distance to the nearest circular obstacle for each ray using
ray-circle intersection geometry.

=========================================================
RAY-CIRCLE INTERSECTION
=========================================================
Ray: P + t·d, where P = boat position, d = unit direction vector.
Circle: centre C = (ox, oy), radius r.

Let v = P - C.  Substituting into |P + t·d - C|² = r²:
  t² + 2t(v·d) + (|v|² - r²) = 0
  discriminant = (v·d)² - (|v|² - r²)

  disc < 0        → no intersection (return RANGE_MAX)
  t = -b - √disc  → nearest intersection (front face of circle)
  if t < RANGE_MIN → boat is inside or behind circle, try far face

=========================================================
TOPICS
=========================================================
Subscribes:
  /sim2d/pose          (geometry_msgs/PoseStamped)      — boat pose

Publishes:
  /scan                (sensor_msgs/LaserScan)           — 360° scan at 10 Hz
                         frame_id=odom, angle_min=boat_heading
                         → rays directly renderable in RViz2 without TF
  /obstacles/markers   (visualization_msgs/MarkerArray)  — grey cylinders in RViz2
"""

import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

try:
    import yaml
except ImportError:
    raise ImportError("PyYAML requis : pip install pyyaml")

# LIDAR sensor parameters
NUM_RAYS = 360
ANGLE_MIN = 0.0
ANGLE_MAX = 2.0 * math.pi
ANGLE_INCREMENT = math.pi / 180.0   # 1° in radians
RANGE_MIN = 0.1                     # metres
RANGE_MAX = 10.0                    # metres


def ray_circle_distance(ox, oy, bx, by, world_angle, radius):
    """
    Distance from (bx, by) along direction world_angle to a circle at (ox, oy).

    Returns a float in [RANGE_MIN, RANGE_MAX].
    Returns RANGE_MAX when there is no intersection ahead of the ray.
    """
    dx = math.cos(world_angle)
    dy = math.sin(world_angle)

    # v = ray origin − circle centre
    vx = bx - ox
    vy = by - oy

    b = vx * dx + vy * dy                      # v · d
    c = vx * vx + vy * vy - radius * radius    # |v|² - r²
    discriminant = b * b - c

    if discriminant < 0.0:
        return RANGE_MAX  # Ray misses circle

    sqrt_disc = math.sqrt(discriminant)
    t = -b - sqrt_disc  # Nearest (front-face) intersection

    if t < RANGE_MIN:
        t = -b + sqrt_disc  # Try far-face intersection
        if t < RANGE_MIN:
            return RANGE_MAX  # Both intersections behind / too close

    return min(t, RANGE_MAX)


class LidarSimulatorNode(Node):
    """Simulate a 360° planar LIDAR against circular obstacles."""

    def __init__(self):
        super().__init__('lidar_simulator')

        # =========================================================
        # LOAD OBSTACLES
        # =========================================================
        obstacles_file = self.declare_parameter(
            'obstacles_file', ''
        ).get_parameter_value().string_value

        if not obstacles_file:
            here = os.path.dirname(os.path.abspath(__file__))
            obstacles_file = os.path.join(
                here, '..', '..', '..', '..', 'share', 'asket_ec_sim2d',
                'config', 'obstacles.yaml')
            if not os.path.exists(obstacles_file):
                obstacles_file = os.path.join(
                    here, '..', 'config', 'obstacles.yaml')

        self.obstacles = self._load_obstacles(obstacles_file)

        # =========================================================
        # BOAT STATE
        # =========================================================
        self.boat_x = 0.0
        self.boat_y = 0.0
        self.boat_heading = 0.0
        self.pose_received = False

        # =========================================================
        # SUBSCRIPTIONS
        # =========================================================
        self.sub_pose = self.create_subscription(
            PoseStamped,
            '/sim2d/pose',
            self.pose_callback,
            10,
        )

        # =========================================================
        # PUBLICATIONS
        # =========================================================
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/obstacles/markers', 100)

        # =========================================================
        # TIMER 10 Hz
        # =========================================================
        self.create_timer(0.1, self.publish_scan)

        self.get_logger().info(
            f'LIDAR simulator started\n'
            f'  {len(self.obstacles)} obstacle(s) loaded\n'
            f'  Range: {RANGE_MIN}–{RANGE_MAX} m  |  {NUM_RAYS} rays  |  1° resolution'
        )

    def _load_obstacles(self, filepath):
        """Load obstacles from YAML.  Returns list of (x, y, radius) tuples."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f'Obstacles file not found: {filepath}')
            return []
        except yaml.YAMLError as e:
            self.get_logger().error(f'YAML error in obstacles file: {e}')
            return []

        raw = data.get('obstacles', [])
        obstacles = [
            (float(o['x']), float(o['y']), float(o['radius']))
            for o in raw
        ]
        for i, (x, y, r) in enumerate(obstacles):
            self.get_logger().info(
                f'  Obstacle {i + 1}: center=({x:.1f}, {y:.1f}) radius={r:.1f} m')
        return obstacles

    def pose_callback(self, msg):
        """Update boat position and heading from /sim2d/pose."""
        self.boat_x = msg.pose.position.x
        self.boat_y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.boat_heading = 2.0 * math.atan2(qz, qw)
        self.pose_received = True

    def publish_scan(self):
        """Cast 360 rays and publish LaserScan + obstacle markers at 10 Hz."""
        now = self.get_clock().now().to_msg()

        # Obstacle markers are published regardless of boat pose
        self._publish_markers(now)

        if not self.pose_received:
            return

        ranges = []
        for i in range(NUM_RAYS):
            # Ray direction in world frame: boat heading + scan angle
            world_angle = self.boat_heading + i * ANGLE_INCREMENT
            min_dist = RANGE_MAX
            for ox, oy, radius in self.obstacles:
                d = ray_circle_distance(
                    ox, oy,
                    self.boat_x, self.boat_y,
                    world_angle, radius,
                )
                if d < min_dist:
                    min_dist = d
            ranges.append(min_dist)

        # Publish in the odom frame so RViz2 (Fixed Frame = odom) can render
        # the rays without a TF lookup.  angle_min is set to the boat's current
        # heading so ray 0 points forward and all angles are in world-frame
        # coordinates.  Ray endpoints in odom frame:
        #   x = boat_x + ranges[i] * cos(boat_heading + i * ANGLE_INCREMENT)
        #   y = boat_y + ranges[i] * sin(boat_heading + i * ANGLE_INCREMENT)
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'odom'
        scan.angle_min = self.boat_heading
        scan.angle_max = self.boat_heading + 2.0 * math.pi
        scan.angle_increment = ANGLE_INCREMENT
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = RANGE_MIN
        scan.range_max = RANGE_MAX
        scan.ranges = ranges
        self.pub_scan.publish(scan)

    def _publish_markers(self, now):
        """Publish grey cylinder markers for every obstacle."""
        marker_array = MarkerArray()
        for i, (ox, oy, radius) in enumerate(self.obstacles):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'odom'
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = ox
            m.pose.position.y = oy
            m.pose.position.z = 0.5     # Half-height so base sits at z=0
            m.pose.orientation.w = 1.0
            m.scale.x = radius * 2.0   # Diameter
            m.scale.y = radius * 2.0
            m.scale.z = 1.0            # 1 m tall cylinder
            m.color.r = 0.3
            m.color.g = 0.3
            m.color.b = 0.3
            m.color.a = 0.8
            marker_array.markers.append(m)
        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LIDAR simulator stopped (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
