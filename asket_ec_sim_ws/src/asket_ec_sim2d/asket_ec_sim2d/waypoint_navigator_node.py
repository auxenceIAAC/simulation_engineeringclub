"""
waypoint_navigator_node.py — Navigateur automatique par waypoints GPS

=========================================================
FONCTIONNEMENT GÉNÉRAL
=========================================================
Ce nœud pilote automatiquement le bateau à travers une séquence de
waypoints GPS définis dans config/waypoints.yaml.

Séquence de démarrage :
  1. Chargement des waypoints depuis waypoints.yaml
  2. Tri du plus proche au plus éloigné (par rapport à l'origine 0,0)
  3. Navigation vers chaque waypoint dans l'ordre
  4. Passage au suivant quand distance < 2m
  5. Arrêt (cmd_vel = 0) quand tous les waypoints sont atteints

=========================================================
ALGORITHME : PURE PURSUIT SIMPLIFIÉ
=========================================================
À chaque tick (10 Hz) :
  1. Calculer le vecteur bateau → waypoint cible (en coordonnées locales m)
  2. Calculer l'angle de ce vecteur dans le référentiel monde (atan2)
  3. Calculer l'erreur angulaire = angle_cible - heading_bateau
     → normalisée dans [-π, π]
  4. Publier /cmd_vel :
       angular.z = 2.0 * sin(erreur)   # correction cap proportionnelle
       linear.x  = 0.5 * cos(erreur)   # ralentir dans les virages serrés

Propriétés :
  - Si le bateau est dans l'axe (erreur ≈ 0) : avance à pleine vitesse
  - Si le bateau est à 90° hors axe : avance à ~0 (vire sur place)
  - Si le bateau est à 180° : recule légèrement (cos négatif)

=========================================================
CONVERSION GPS → COORDONNÉES LOCALES
=========================================================
Même origine que simulator_node.py : 41.3851°N, 2.1734°E (Barcelone)

  x_local = (lon - ORIGIN_LON) * cos(ORIGIN_LAT_rad) * R_terre * π/180
  y_local = (lat - ORIGIN_LAT) * R_terre * π/180

=========================================================
TOPICS
=========================================================
Souscrit :
  /sim2d/pose        (geometry_msgs/PoseStamped) — position + cap courant

Publie :
  /cmd_vel           (geometry_msgs/Twist)        — commandes de navigation
  /waypoints/path    (nav_msgs/Path)              — tous les waypoints (rouge RViz2)
  /waypoints/current (geometry_msgs/PointStamped) — waypoint cible actuel
"""

import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Path

try:
    import yaml
except ImportError:
    raise ImportError("PyYAML requis : pip install pyyaml")

# Origine GPS — doit correspondre à simulator_node.py
ORIGIN_LAT = 41.3851
ORIGIN_LON = 2.1734
EARTH_RADIUS = 6_371_000.0

# Seuil d'arrivée à un waypoint (mètres)
WAYPOINT_RADIUS = 2.0


def gps_to_local(lat, lon):
    """Convertit (lat, lon) en coordonnées locales (x, y) en mètres."""
    lat_rad = math.radians(ORIGIN_LAT)
    x = (lon - ORIGIN_LON) * math.cos(lat_rad) * EARTH_RADIUS * math.pi / 180.0
    y = (lat - ORIGIN_LAT) * EARTH_RADIUS * math.pi / 180.0
    return x, y


def normalize_angle(angle):
    """Normalise un angle dans [-π, π]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointNavigatorNode(Node):
    """
    Nœud de navigation automatique par waypoints.

    Lit les waypoints depuis waypoints.yaml, les trie par distance
    croissante depuis l'origine, puis pilote le bateau via /cmd_vel
    en utilisant un pure pursuit simplifié.
    """

    def __init__(self):
        super().__init__('waypoint_navigator')

        # =========================================================
        # CHARGEMENT DES WAYPOINTS
        # =========================================================
        waypoints_file = self.declare_parameter(
            'waypoints_file', ''
        ).get_parameter_value().string_value

        if not waypoints_file:
            # Chemin par défaut : à côté de ce fichier Python (dev),
            # ou dans share/asket_ec_sim2d/config/ (install)
            here = os.path.dirname(os.path.abspath(__file__))
            waypoints_file = os.path.join(here, '..', '..', '..', '..',
                                          'share', 'asket_ec_sim2d',
                                          'config', 'waypoints.yaml')
            # Fallback : chercher dans le répertoire source
            if not os.path.exists(waypoints_file):
                waypoints_file = os.path.join(here, '..', 'config',
                                              'waypoints.yaml')

        self.waypoints = self._load_waypoints(waypoints_file)

        if not self.waypoints:
            self.get_logger().error(
                f'Aucun waypoint chargé depuis {waypoints_file} — arrêt.')
            return

        # =========================================================
        # ÉTAT DU NAVIGATEUR
        # =========================================================
        self.current_x = 0.0       # Position courante du bateau (m)
        self.current_y = 0.0
        self.current_heading = 0.0  # Cap courant (rad)
        self.pose_received = False   # En attente de la première pose

        self.current_wp_idx = 0     # Index du waypoint cible courant
        self.mission_complete = False

        # =========================================================
        # SOUSCRIPTION
        # =========================================================
        self.sub_pose = self.create_subscription(
            PoseStamped,
            '/sim2d/pose',
            self.pose_callback,
            10
        )

        # =========================================================
        # PUBLICATIONS
        # =========================================================
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_wp_path = self.create_publisher(
            Path, '/waypoints/path', 10)
        self.pub_wp_current = self.create_publisher(
            PointStamped, '/waypoints/current', 10)

        # =========================================================
        # TIMER 10 Hz — boucle de navigation
        # =========================================================
        self.timer = self.create_timer(0.1, self.navigation_step)

        self.get_logger().info(
            f'Navigateur waypoints démarré\n'
            f'  {len(self.waypoints)} waypoints chargés (triés par distance)\n'
            f'  Seuil arrivée : {WAYPOINT_RADIUS} m\n'
            f'  Premier waypoint : {self.waypoints[0]}'
        )

    def _load_waypoints(self, filepath):
        """
        Charge et trie les waypoints depuis le fichier YAML.

        Tri : distance croissante par rapport à l'origine (0, 0).
        Retourne une liste de tuples (x_local, y_local).
        """
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f'Fichier introuvable : {filepath}')
            return []
        except yaml.YAMLError as e:
            self.get_logger().error(f'Erreur YAML : {e}')
            return []

        raw = data.get('waypoints', [])
        if not raw:
            self.get_logger().error('Clé "waypoints" absente ou vide dans le YAML')
            return []

        # Convertir GPS → coordonnées locales
        local_pts = [gps_to_local(wp['lat'], wp['lon']) for wp in raw]

        # Trier par distance depuis l'origine (position initiale du bateau)
        local_pts.sort(key=lambda p: math.hypot(p[0], p[1]))

        self.get_logger().info(
            f'Waypoints triés (distance depuis origine) :\n' +
            '\n'.join(f'  [{i}] x={p[0]:.1f}m y={p[1]:.1f}m '
                      f'(d={math.hypot(p[0],p[1]):.1f}m)'
                      for i, p in enumerate(local_pts))
        )
        return local_pts

    def pose_callback(self, msg):
        """Met à jour la position et le cap du bateau."""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

        # Extraire le cap depuis le quaternion (rotation Z uniquement)
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.current_heading = 2.0 * math.atan2(qz, qw)

        self.pose_received = True

    def navigation_step(self):
        """
        Boucle de navigation à 10 Hz.

        Publie /waypoints/path et /waypoints/current à chaque tick.
        Publie /cmd_vel uniquement si une pose a été reçue et que
        la mission n'est pas terminée.
        """
        now = self.get_clock().now().to_msg()

        # --- Publier le chemin de tous les waypoints ---
        self._publish_waypoints_path(now)

        if not self.pose_received or self.mission_complete:
            return

        # --- Waypoint cible courant ---
        wp_x, wp_y = self.waypoints[self.current_wp_idx]

        # Publier /waypoints/current
        current_pt = PointStamped()
        current_pt.header.stamp = now
        current_pt.header.frame_id = 'odom'
        current_pt.point.x = wp_x
        current_pt.point.y = wp_y
        current_pt.point.z = 0.0
        self.pub_wp_current.publish(current_pt)

        # --- Distance au waypoint ---
        dx = wp_x - self.current_x
        dy = wp_y - self.current_y
        distance = math.hypot(dx, dy)

        if distance < WAYPOINT_RADIUS:
            self.get_logger().info(
                f'Waypoint [{self.current_wp_idx}] atteint '
                f'(distance={distance:.2f}m)'
            )
            self.current_wp_idx += 1

            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info(
                    'Mission complète — tous les waypoints atteints !')
                self.mission_complete = True
                # Arrêt moteurs
                self.pub_cmd_vel.publish(Twist())
                return

            self.get_logger().info(
                f'→ Prochain waypoint [{self.current_wp_idx}] : '
                f'x={self.waypoints[self.current_wp_idx][0]:.1f}m '
                f'y={self.waypoints[self.current_wp_idx][1]:.1f}m'
            )
            return

        # --- Pure pursuit simplifié ---
        # Angle vers le waypoint dans le référentiel monde
        angle_to_wp = math.atan2(dy, dx)

        # Erreur angulaire = différence entre cap voulu et cap actuel
        angle_error = normalize_angle(angle_to_wp - self.current_heading)

        # Commandes :
        #   angular.z proportionnel à sin(erreur) → correction douce du cap
        #   linear.x  proportionnel à cos(erreur) → ralentir dans les virages
        cmd = Twist()
        cmd.angular.z = 2.0 * math.sin(angle_error)
        cmd.linear.x = 0.5 * math.cos(angle_error)
        self.pub_cmd_vel.publish(cmd)

    def _publish_waypoints_path(self, now):
        """Publie /waypoints/path avec tous les waypoints (pour RViz2)."""
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = 'odom'

        for wp_x, wp_y in self.waypoints:
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = 'odom'
            ps.pose.position.x = wp_x
            ps.pose.position.y = wp_y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.pub_wp_path.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt du navigateur (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
