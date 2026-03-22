"""
buoy_simulator_node.py — Simulated camera sensor for buoy/gate detection

=========================================================
PURPOSE
=========================================================
Simulates what the boat's camera sees when navigating through a slalom
course made of buoy gates. Each gate has one red buoy (starboard) and
one green buoy (port); the boat must pass between them.

On startup:
  1. Load gate definitions from config/buoys.yaml
  2. Convert all GPS buoy positions to local coordinates (metres)
  3. Compute the centre of each gate (midpoint red ↔ green)

At 10 Hz:
  • For every buoy, compute distance and bearing from the boat.
  • Add Gaussian noise to simulate camera measurement uncertainty.
  • A buoy is "detected" if the noisy distance < 15 m AND the noisy
    relative bearing is within ±60° of the boat heading.
  • Publish visualisation markers and the gate-centre path.

=========================================================
TOPICS PUBLISHED
=========================================================
  /buoys/all      (visualization_msgs/MarkerArray)
      All buoys in the world (detected or not).
      Sphere markers, radius 0.3 m, frame odom.
      Red (1,0,0) / Green (0,1,0).
      Alpha 0.3 = not currently visible, 1.0 = visible.

  /buoys/detected (visualization_msgs/MarkerArray)
      Only currently visible buoys, full opacity.
      Includes a TEXT_VIEW_FACING marker above each buoy:
        "GATE X - RED" or "GATE X - GREEN".

  /gates/centers  (nav_msgs/Path)
      Midpoints of all gates in gate order, frame odom.
      Consumed by waypoint_navigator_node as high-priority waypoints.

TOPIC SUBSCRIBED
  /sim2d/pose (geometry_msgs/PoseStamped) — boat position and heading

=========================================================
SENSOR MODEL
=========================================================
  Camera range      : 15.0 m
  Camera half-FOV   : 60° (total 120°)
  Distance noise    : Gaussian σ = 0.3 m
  Bearing noise     : Gaussian σ = 2°
"""

import math
import os
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

try:
    import yaml
except ImportError:
    raise ImportError("PyYAML required: pip install pyyaml")

# GPS origin — must match simulator_node.py
ORIGIN_LAT = 41.3851
ORIGIN_LON = 2.1734
EARTH_RADIUS = 6_371_000.0  # metres

# Camera sensor parameters
CAMERA_RANGE = 15.0                    # metres
CAMERA_HALF_FOV = math.radians(60.0)  # ±60° field of view
DIST_SIGMA = 0.3                       # distance noise σ (m)
ANGLE_SIGMA = math.radians(2.0)        # bearing noise σ (rad)


def gps_to_local(lat, lon):
    """Convert GPS (lat, lon) to local (x, y) in metres from the origin."""
    lat_rad = math.radians(ORIGIN_LAT)
    x = (lon - ORIGIN_LON) * math.cos(lat_rad) * EARTH_RADIUS * math.pi / 180.0
    y = (lat - ORIGIN_LAT) * EARTH_RADIUS * math.pi / 180.0
    return x, y


def normalize_angle(angle):
    """Normalise an angle to [-π, π]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class BuoySimulatorNode(Node):
    """
    Camera-sensor simulation node for buoy gate detection.

    Reads gate definitions from buoys.yaml, tracks boat pose from
    /sim2d/pose, and publishes marker arrays at 10 Hz.
    """

    def __init__(self):
        super().__init__('buoy_simulator')

        # =========================================================
        # LOAD BUOYS.YAML
        # =========================================================
        buoys_file = self.declare_parameter(
            'buoys_file', ''
        ).get_parameter_value().string_value

        if not buoys_file:
            here = os.path.dirname(os.path.abspath(__file__))
            # Installed path (colcon build)
            installed = os.path.join(
                here, '..', '..', '..', '..', 'share',
                'asket_ec_sim2d', 'config', 'buoys.yaml')
            # Source path (pip install -e .)
            source = os.path.join(here, '..', 'config', 'buoys.yaml')
            buoys_file = installed if os.path.exists(installed) else source

        self.gates = self._load_gates(buoys_file)

        if not self.gates:
            self.get_logger().error(
                f'No gates loaded from {buoys_file} — node idle.')
            return

        # =========================================================
        # BOAT STATE
        # =========================================================
        self.boat_x = 0.0
        self.boat_y = 0.0
        self.boat_heading = 0.0
        self.pose_received = False

        # =========================================================
        # SUBSCRIPTION
        # =========================================================
        self.create_subscription(
            PoseStamped, '/sim2d/pose', self._pose_callback, 10)

        # =========================================================
        # PUBLISHERS
        # =========================================================
        self.pub_all = self.create_publisher(
            MarkerArray, '/buoys/all', 10)
        self.pub_detected = self.create_publisher(
            MarkerArray, '/buoys/detected', 10)
        self.pub_gate_centers = self.create_publisher(
            Path, '/gates/centers', 10)

        # =========================================================
        # 10 Hz TIMER
        # =========================================================
        self.create_timer(0.1, self._update)

        summary = '\n'.join(
            f'  Gate {g["id"]}: '
            f'red=({g["rx"]:.1f}, {g["ry"]:.1f})m  '
            f'green=({g["gx"]:.1f}, {g["gy"]:.1f})m  '
            f'center=({g["cx"]:.1f}, {g["cy"]:.1f})m'
            for g in self.gates
        )
        self.get_logger().info(
            f'Buoy simulator started — {len(self.gates)} gates\n{summary}')

    # ------------------------------------------------------------------
    # STARTUP: LOAD GATES
    # ------------------------------------------------------------------

    def _load_gates(self, filepath):
        """Load gates from YAML and compute local coordinates + centres."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f'File not found: {filepath}')
            return []
        except yaml.YAMLError as e:
            self.get_logger().error(f'YAML parse error: {e}')
            return []

        gates = []
        for entry in data.get('gates', []):
            rx, ry = gps_to_local(entry['red']['lat'],   entry['red']['lon'])
            gx, gy = gps_to_local(entry['green']['lat'], entry['green']['lon'])
            gates.append({
                'id': entry['id'],
                'rx': rx, 'ry': ry,   # red buoy local position
                'gx': gx, 'gy': gy,   # green buoy local position
                'cx': (rx + gx) / 2.0,  # gate centre x
                'cy': (ry + gy) / 2.0,  # gate centre y
            })
        return gates

    # ------------------------------------------------------------------
    # POSE CALLBACK
    # ------------------------------------------------------------------

    def _pose_callback(self, msg):
        self.boat_x = msg.pose.position.x
        self.boat_y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.boat_heading = 2.0 * math.atan2(qz, qw)
        self.pose_received = True

    # ------------------------------------------------------------------
    # VISIBILITY CHECK (with sensor noise)
    # ------------------------------------------------------------------

    def _is_visible(self, bx, by):
        """
        Return True if the buoy at (bx, by) would be detected by the camera.

        Computes true distance and relative bearing, adds Gaussian noise
        to simulate measurement uncertainty, then applies range/FOV thresholds.
        """
        dx = bx - self.boat_x
        dy = by - self.boat_y
        true_dist = math.hypot(dx, dy)
        true_rel_angle = normalize_angle(
            math.atan2(dy, dx) - self.boat_heading)

        noisy_dist = true_dist + random.gauss(0.0, DIST_SIGMA)
        noisy_angle = true_rel_angle + random.gauss(0.0, ANGLE_SIGMA)

        return noisy_dist < CAMERA_RANGE and abs(noisy_angle) <= CAMERA_HALF_FOV

    # ------------------------------------------------------------------
    # MARKER FACTORIES
    # ------------------------------------------------------------------

    def _sphere_marker(self, marker_id, x, y, r, g, b, alpha, stamp):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = stamp
        m.ns = 'buoys'
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        # scale = diameter; radius 0.3 m → diameter 0.6 m
        m.scale.x = 0.6
        m.scale.y = 0.6
        m.scale.z = 0.6
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = alpha
        return m

    def _text_marker(self, marker_id, x, y, text, stamp):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = stamp
        m.ns = 'buoy_labels'
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 1.0   # 1 m above the buoy sphere
        m.pose.orientation.w = 1.0
        m.scale.z = 0.5           # text height in metres
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.text = text
        return m

    # ------------------------------------------------------------------
    # MAIN LOOP (10 Hz)
    # ------------------------------------------------------------------

    def _update(self):
        now = self.get_clock().now().to_msg()

        all_markers = MarkerArray()
        detected_markers = MarkerArray()

        for gate in self.gates:
            gid = gate['id']
            # Stable marker IDs: red buoy = gid*2, green buoy = gid*2+1
            # Text labels offset by 1000 to avoid collision with sphere IDs
            red_sphere_id   = gid * 2
            green_sphere_id = gid * 2 + 1
            red_text_id     = 1000 + red_sphere_id
            green_text_id   = 1000 + green_sphere_id

            red_visible = (
                self._is_visible(gate['rx'], gate['ry'])
                if self.pose_received else False
            )
            green_visible = (
                self._is_visible(gate['gx'], gate['gy'])
                if self.pose_received else False
            )

            # /buoys/all — every buoy, alpha indicates visibility
            all_markers.markers.append(self._sphere_marker(
                red_sphere_id, gate['rx'], gate['ry'],
                1.0, 0.0, 0.0,
                1.0 if red_visible else 0.3,
                now))
            all_markers.markers.append(self._sphere_marker(
                green_sphere_id, gate['gx'], gate['gy'],
                0.0, 1.0, 0.0,
                1.0 if green_visible else 0.3,
                now))

            # /buoys/detected — only visible buoys, with text labels
            if red_visible:
                detected_markers.markers.append(self._sphere_marker(
                    red_sphere_id, gate['rx'], gate['ry'],
                    1.0, 0.0, 0.0, 1.0, now))
                detected_markers.markers.append(self._text_marker(
                    red_text_id, gate['rx'], gate['ry'],
                    f'GATE {gid} - RED', now))

            if green_visible:
                detected_markers.markers.append(self._sphere_marker(
                    green_sphere_id, gate['gx'], gate['gy'],
                    0.0, 1.0, 0.0, 1.0, now))
                detected_markers.markers.append(self._text_marker(
                    green_text_id, gate['gx'], gate['gy'],
                    f'GATE {gid} - GREEN', now))

        self.pub_all.publish(all_markers)
        self.pub_detected.publish(detected_markers)

        # /gates/centers — Path through gate midpoints in gate order
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = now
        for gate in self.gates:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.header.stamp = now
            ps.pose.position.x = gate['cx']
            ps.pose.position.y = gate['cy']
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_gate_centers.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = BuoySimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Buoy simulator stopped (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
