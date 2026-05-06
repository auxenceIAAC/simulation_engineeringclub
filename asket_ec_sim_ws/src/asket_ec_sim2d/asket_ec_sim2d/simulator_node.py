"""
simulator_node.py — Simulateur 2D léger pour le bateau Asket EC

=========================================================
POURQUOI CE SIMULATEUR ?
=========================================================
Gazebo est trop instable sur WSL2 (segfaults, rendu OpenGL cassé, gz-transport
inaccessible depuis ROS2). Ce simulateur Python pur remplace entièrement Gazebo
pour le développement sur WSL2 / machines sans GPU.

Il tourne à 50 Hz et calcule :
  - La physique différentielle du bateau (propulsion + traînée)
  - La position et l'orientation dans un référentiel 2D (plan XY)
  - Une position GPS simulée depuis l'origine à Barcelone

=========================================================
PHYSIQUE SIMPLIFIÉE
=========================================================
Le bateau est modélisé comme un corps rigide 2D avec :
  - Propulsion différentielle (2 thrusters indépendants)
  - Traînée visqueuse proportionnelle à la vitesse (résistance de l'eau)
  - Pas de masse, pas d'inertie de rotation (modèle cinématique)

Formules à chaque pas de temps dt = 0.02s (50 Hz) :

  v_port      = linear.x + (L/2) * angular.z   -- thruster bâbord
  v_starboard = linear.x - (L/2) * angular.z   -- thruster tribord

  acceleration = (v_port + v_starboard) / 2    -- poussée nette
  drag         = -3.0 * speed                  -- résistance eau (Stokes)

  speed   += (acceleration + drag) * dt        -- intégration Euler
  heading += angular.z * dt                    -- cap (rad)
  x       += speed * cos(heading) * dt         -- est
  y       += speed * sin(heading) * dt         -- nord

=========================================================
COORDONNÉES GPS SIMULÉES
=========================================================
Origine locale (0,0) = port de Barcelone : 41.3851°N, 2.1734°E

Conversion position locale (m) → latitude/longitude :
  lat = ORIGIN_LAT + y / R_terre * (180/π)
  lon = ORIGIN_LON + x / (R_terre * cos(lat_rad)) * (180/π)

=========================================================
TOPICS PUBLIÉS
=========================================================
  /sim2d/pose   (geometry_msgs/PoseStamped)  — position + orientation
  /sim2d/path   (nav_msgs/Path)              — trajectoire complète
  /sim2d/odom   (nav_msgs/Odometry)          — odométrie standard ROS2
  /sim2d/navsat (sensor_msgs/NavSatFix)      — position GPS simulée

TOPICS SOUSCRITS
  /cmd_vel     (geometry_msgs/Twist) — commandes de vitesse
  /manual_mode (std_msgs/Bool)       — True=MANUAL, False=AUTO (log only)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

# Origine GPS : port de Barcelone
ORIGIN_LAT = 41.3851   # degrés Nord
ORIGIN_LON = 2.1734    # degrés Est
EARTH_RADIUS = 6_371_000.0  # mètres (rayon moyen)


class Sim2DNode(Node):
    """
    Nœud ROS2 du simulateur 2D.

    Souscrit à /cmd_vel, calcule la physique à 50 Hz, publie
    pose / trajectoire / odométrie / GPS sur /sim2d/*.
    """

    def __init__(self):
        super().__init__('sim2d_simulator')

        # =========================================================
        # ÉTAT DU BATEAU
        # =========================================================
        self.x = 0.0        # Position est (m), référentiel local
        self.y = 0.0        # Position nord (m)
        self.heading = 0.0  # Cap en radians (0 = est, π/2 = nord)
        self.speed = 0.0    # Vitesse longitudinale (m/s)

        # Dernière commande /cmd_vel reçue
        self.linear_x = 0.0   # m/s : avance / recul
        self.angular_z = 0.0  # rad/s : rotation

        # Mode courant (pour log uniquement — le pass-through /cmd_vel est inchangé)
        self.manual_mode = False

        # Historique de la trajectoire pour /sim2d/path
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        # =========================================================
        # SOUSCRIPTIONS
        # =========================================================
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.sub_manual_mode = self.create_subscription(
            Bool,
            '/manual_mode',
            self.manual_mode_callback,
            10
        )

        # =========================================================
        # PUBLICATIONS
        # =========================================================
        self.pub_pose = self.create_publisher(
            PoseStamped, '/sim2d/pose', 10)
        self.pub_path = self.create_publisher(
            Path, '/sim2d/path', 10)
        self.pub_odom = self.create_publisher(
            Odometry, '/sim2d/odom', 10)
        self.pub_navsat = self.create_publisher(
            NavSatFix, '/sim2d/navsat', 10)
        self.pub_boat_shape = self.create_publisher(
            MarkerArray, '/sim2d/boat_shape', 10)

        # =========================================================
        # BROADCASTER TF2 — odom → base_link
        # =========================================================
        # Sans cette transform, RViz2 affiche une erreur orange sur Fixed Frame
        # car il ne sait pas où est base_link par rapport à odom.
        self.tf_broadcaster = TransformBroadcaster(self)

        # =========================================================
        # TIMER 50 Hz
        # =========================================================
        self.dt = 0.02  # secondes
        self.timer = self.create_timer(self.dt, self.physics_step)

        self.get_logger().info(
            'Simulateur 2D Asket EC démarré\n'
            '  Fréquence : 50 Hz\n'
            f'  Origine GPS : {ORIGIN_LAT}°N {ORIGIN_LON}°E (Barcelone)\n'
            '  En attente de commandes sur /cmd_vel...'
        )

    def cmd_vel_callback(self, msg):
        """Reçoit les commandes de vitesse depuis /cmd_vel."""
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def manual_mode_callback(self, msg):
        """Log mode changes from /manual_mode. cmd_vel pass-through is unchanged."""
        new_manual = msg.data
        if new_manual != self.manual_mode:
            self.manual_mode = new_manual
            mode_str = 'MANUAL' if new_manual else 'AUTO'
            self.get_logger().info(f'Mode changed to {mode_str}')

    def physics_step(self):
        """
        Calcule un pas de physique à 50 Hz et publie l'état du bateau.

        Physique différentielle simplifiée :
          - Propulsion : deux thrusters indépendants
          - Traînée : -0.3 * vitesse (résistance visqueuse de l'eau)
          - Intégration Euler explicite
        """
        dt = self.dt
        L = 0.5  # écartement entre les thrusters (m)

        # --- Propulsion différentielle ---
        v_port = self.linear_x + (L / 2.0) * self.angular_z
        v_starboard = self.linear_x - (L / 2.0) * self.angular_z

        # Accélération nette = moyenne des deux thrusters
        acceleration = (v_port + v_starboard) / 2.0

        # Traînée visqueuse (résistance de l'eau).
        # Coefficient 0.6 : vitesse plafonne à ~2.5 m/s pour linear.x=1.5
        # → couvre ~45m entre portes en 15-20 secondes
        drag = -0.6 * self.speed

        # --- Intégration Euler ---
        self.speed += (acceleration + drag) * dt
        self.heading += self.angular_z * dt
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt

        # --- Quaternion depuis le cap (rotation autour de Z) ---
        qz = math.sin(self.heading / 2.0)
        qw = math.cos(self.heading / 2.0)

        now = self.get_clock().now().to_msg()

        # --- Construire le PoseStamped ---
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # --- Publier la transform TF odom → base_link ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # --- Publier /sim2d/pose ---
        self.pub_pose.publish(pose)

        # --- Mettre à jour et publier /sim2d/path ---
        self.path_msg.header.stamp = now
        self.path_msg.poses.append(pose)
        # Limiter la mémoire : garder les 10 000 derniers points
        if len(self.path_msg.poses) > 10_000:
            self.path_msg.poses = self.path_msg.poses[-5_000:]
        self.pub_path.publish(self.path_msg)

        # --- Publier /sim2d/odom ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x = self.speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.angular_z
        self.pub_odom.publish(odom)

        # --- Publier /sim2d/boat_shape ---
        self.pub_boat_shape.publish(self._make_boat_markers(now, qz, qw))

        # --- Publier /sim2d/navsat (GPS simulé) ---
        # Conversion position locale (m) → lat/lon depuis l'origine Barcelone
        lat = ORIGIN_LAT + (self.y / EARTH_RADIUS) * (180.0 / math.pi)
        lon = ORIGIN_LON + (
            self.x / (EARTH_RADIUS * math.cos(math.radians(ORIGIN_LAT)))
        ) * (180.0 / math.pi)

        navsat = NavSatFix()
        navsat.header.stamp = now
        navsat.header.frame_id = 'base_link'
        navsat.latitude = lat
        navsat.longitude = lon
        navsat.altitude = 0.0
        navsat.status.status = NavSatStatus.STATUS_FIX
        navsat.status.service = NavSatStatus.SERVICE_GPS
        self.pub_navsat.publish(navsat)


    def _make_boat_markers(self, stamp, qz, qw):
        """
        Build a MarkerArray representing the boat as a top-down silhouette.

        Two TRIANGLE_LIST markers share the boat's pose so vertices are
        defined in local frame (x = forward, y = left):
          - hull: white/cream filled polygon
          - bow accent: small orange triangle so heading is obvious
        """
        array = MarkerArray()

        base_pose_args = dict(
            frame_id='odom',
            px=self.x, py=self.y,
            qz=qz, qw=qw,
            stamp=stamp,
        )

        # --- Hull polygon (fan-triangulated around local origin) ---
        hull_pts = [
            (2.5,  0.00),
            (1.5,  0.70),
            (0.0,  0.90),
            (-1.5, 0.80),
            (-2.0, 0.25),
            (-2.0, -0.25),
            (-1.5, -0.80),
            (0.0,  -0.90),
            (1.5,  -0.70),
        ]
        hull = Marker()
        hull.header.frame_id = 'odom'
        hull.header.stamp = stamp
        hull.ns = 'boat'
        hull.id = 0
        hull.type = Marker.TRIANGLE_LIST
        hull.action = Marker.ADD
        hull.pose.position.x = self.x
        hull.pose.position.y = self.y
        hull.pose.position.z = 0.0
        hull.pose.orientation.z = qz
        hull.pose.orientation.w = qw
        hull.scale.x = hull.scale.y = hull.scale.z = 1.0
        hull.color.r = 0.95
        hull.color.g = 0.95
        hull.color.b = 0.85
        hull.color.a = 1.0
        center = Point(x=0.0, y=0.0, z=0.0)
        n = len(hull_pts)
        for i in range(n):
            p1 = Point(x=hull_pts[i][0],           y=hull_pts[i][1],           z=0.0)
            p2 = Point(x=hull_pts[(i + 1) % n][0], y=hull_pts[(i + 1) % n][1], z=0.0)
            hull.points += [center, p1, p2]
        array.markers.append(hull)

        # --- Bow accent: small orange triangle so direction is obvious ---
        bow = Marker()
        bow.header.frame_id = 'odom'
        bow.header.stamp = stamp
        bow.ns = 'boat'
        bow.id = 1
        bow.type = Marker.TRIANGLE_LIST
        bow.action = Marker.ADD
        bow.pose.position.x = self.x
        bow.pose.position.y = self.y
        bow.pose.position.z = 0.05
        bow.pose.orientation.z = qz
        bow.pose.orientation.w = qw
        bow.scale.x = bow.scale.y = bow.scale.z = 1.0
        bow.color.r = 1.0
        bow.color.g = 0.45
        bow.color.b = 0.0
        bow.color.a = 1.0
        for vx, vy in [(2.5, 0.0), (1.5, 0.55), (1.5, -0.55)]:
            bow.points.append(Point(x=vx, y=vy, z=0.0))
        array.markers.append(bow)

        return array


def main(args=None):
    rclpy.init(args=args)
    node = Sim2DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt du simulateur 2D (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
