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
       linear.x  = 1.5 * cos(erreur)   # ralentir dans les virages serrés

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
  /gates/centers     (nav_msgs/Path)              — centres de portes (priorité)
  /manual_mode       (std_msgs/Bool)              — True=MANUAL, False=AUTO

Publie :
  /cmd_vel           (geometry_msgs/Twist)        — commandes de navigation
  /waypoints/path    (nav_msgs/Path)              — tous les waypoints (rouge RViz2)
  /waypoints/current (geometry_msgs/PointStamped) — waypoint cible actuel
  /current_mode      (std_msgs/String)            — mode courant à 1 Hz

=========================================================
PRIORITÉ DE SOURCE DES WAYPOINTS
=========================================================
  1. /gates/centers (depuis buoy_simulator_node) — si reçu, utilise les
     centres de portes comme waypoints et log "Gate X passed ✓" à chaque
     franchissement.
  2. waypoints.yaml — fallback si aucune porte n'est publiée.
"""

import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

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

    Supporte le basculement MANUAL/AUTO via /manual_mode :
      - MANUAL : stoppe le bateau et cède le contrôle
      - AUTO   : reprend depuis la porte la plus proche
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

        self.yaml_waypoints = self._load_waypoints(waypoints_file)
        # Active waypoint list — replaced by gate centres when available
        self.waypoints = list(self.yaml_waypoints)

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
        self.using_gates = False     # True when navigating gate centres
        self.passed_gates = []       # Indices of gates already passed (never re-targeted)

        # =========================================================
        # MODE MANUEL / AUTOMATIQUE
        # =========================================================
        self.manual_mode = False    # False = AUTO, True = MANUAL

        # =========================================================
        # SOUSCRIPTIONS
        # =========================================================
        self.sub_pose = self.create_subscription(
            PoseStamped,
            '/sim2d/pose',
            self.pose_callback,
            10
        )

        # Gate centres published by buoy_simulator_node.
        # If received, they replace waypoints.yaml as the navigation target.
        self.sub_gates = self.create_subscription(
            Path,
            '/gates/centers',
            self.gate_centers_callback,
            10
        )

        # Manual mode toggle from keyboard_teleop or any other source
        self.sub_manual_mode = self.create_subscription(
            Bool,
            '/manual_mode',
            self.manual_mode_callback,
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
        self.pub_current_mode = self.create_publisher(
            String, '/current_mode', 10)

        # =========================================================
        # TIMER 10 Hz — boucle de navigation
        # =========================================================
        self.timer = self.create_timer(0.1, self.navigation_step)

        # =========================================================
        # TIMER 1 Hz — publication du mode courant
        # =========================================================
        self.mode_timer = self.create_timer(1.0, self.publish_current_mode)

        self.get_logger().info(
            f'Navigateur waypoints démarré\n'
            f'  {len(self.waypoints)} waypoints chargés (triés par distance)\n'
            f'  Seuil arrivée : {WAYPOINT_RADIUS} m\n'
            f'  Premier waypoint : {self.waypoints[0]}\n'
            f'  En attente de /gates/centers (priorité sur waypoints.yaml)…'
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

    def manual_mode_callback(self, msg):
        """Handle MANUAL/AUTO mode switching from /manual_mode topic."""
        new_manual = msg.data

        if new_manual == self.manual_mode:
            return  # No state change

        if new_manual:
            # Switching AUTO → MANUAL
            self.manual_mode = True
            # Immediately stop the boat
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info('Switching to MANUAL — navigator paused')
        else:
            # Switching MANUAL → AUTO
            self.manual_mode = False
            if self.waypoints and self.pose_received:
                closest_idx = self._find_closest_gate()
                if closest_idx is None:
                    # All gates already passed — nothing left to do
                    self.get_logger().info(
                        'Switching to AUTO — all gates already passed, mission complete')
                else:
                    self.current_wp_idx = closest_idx
                    self.mission_complete = False
                    gate_label = closest_idx + 1  # 1-based display
                    self.get_logger().info(
                        f'Switching to AUTO — resuming from gate {gate_label}')
            else:
                self.get_logger().info('Switching to AUTO — navigator resumed')

    def _find_next_unpassed_gate(self):
        """Return the lowest index not yet in passed_gates, or None if all passed."""
        for i in range(len(self.waypoints)):
            if i not in self.passed_gates:
                return i
        return None

    def _find_closest_gate(self):
        """Return the index of the closest waypoint that has NOT been passed yet.

        Falls back to the current index if every gate has already been passed.
        """
        min_dist = float('inf')
        closest_idx = None
        for i, (wx, wy) in enumerate(self.waypoints):
            if i in self.passed_gates:
                continue
            dist = math.hypot(wx - self.current_x, wy - self.current_y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        return closest_idx  # None when all gates are passed

    def publish_current_mode(self):
        """Publish the current navigation mode on /current_mode at 1 Hz."""
        mode_msg = String()
        mode_msg.data = 'MANUAL' if self.manual_mode else 'AUTO'
        self.pub_current_mode.publish(mode_msg)

    def gate_centers_callback(self, msg):
        """
        Receive gate centres from buoy_simulator_node.

        On first reception, switch navigation source from waypoints.yaml
        to gate centres and reset the waypoint index.  Subsequent calls
        keep the list up to date without resetting progress.
        """
        if not msg.poses:
            return

        gate_centers = [
            (ps.pose.position.x, ps.pose.position.y)
            for ps in msg.poses
        ]

        if not self.using_gates:
            self.waypoints = gate_centers
            self.current_wp_idx = 0
            self.mission_complete = False
            self.passed_gates = []
            self.using_gates = True
            self.get_logger().info(
                f'Gate navigation activated — '
                f'{len(gate_centers)} gates received (replacing waypoints.yaml)\n' +
                '\n'.join(
                    f'  Gate {i + 1}: center=({cx:.1f}m, {cy:.1f}m)'
                    for i, (cx, cy) in enumerate(gate_centers)
                )
            )
        else:
            # Update positions in case buoy_simulator published an update
            self.waypoints = gate_centers

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
        Publie /cmd_vel uniquement si une pose a été reçue, que la mission
        n'est pas terminée, et que le mode est AUTO.
        """
        now = self.get_clock().now().to_msg()

        # --- Publier le chemin de tous les waypoints ---
        self._publish_waypoints_path(now)

        # En mode MANUAL, le navigateur ne publie pas de commandes
        if self.manual_mode:
            return

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
            if self.using_gates:
                gate_number = self.current_wp_idx + 1  # gate IDs start at 1
                self.get_logger().info(
                    f'Gate {gate_number} passed ✓  (distance={distance:.2f}m)')
            else:
                self.get_logger().info(
                    f'Waypoint [{self.current_wp_idx}] atteint '
                    f'(distance={distance:.2f}m)')

            # Mark this gate as passed — it will never be re-targeted
            self.passed_gates.append(self.current_wp_idx)

            next_idx = self._find_next_unpassed_gate()
            if next_idx is None:
                if self.using_gates:
                    self.get_logger().info(
                        'All gates passed ✓  Mission complete!')
                else:
                    self.get_logger().info(
                        'Mission complète — tous les waypoints atteints !')
                self.mission_complete = True
                self.pub_cmd_vel.publish(Twist())
                return

            self.current_wp_idx = next_idx
            if self.using_gates:
                next_x, next_y = self.waypoints[self.current_wp_idx]
                self.get_logger().info(
                    f'→ Heading to gate {self.current_wp_idx + 1}: '
                    f'x={next_x:.1f}m y={next_y:.1f}m')
            else:
                self.get_logger().info(
                    f'→ Prochain waypoint [{self.current_wp_idx}] : '
                    f'x={self.waypoints[self.current_wp_idx][0]:.1f}m '
                    f'y={self.waypoints[self.current_wp_idx][1]:.1f}m')
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
