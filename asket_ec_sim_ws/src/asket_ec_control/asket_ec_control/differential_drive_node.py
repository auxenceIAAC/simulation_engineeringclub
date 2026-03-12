"""
differential_drive_node.py — Contrôleur différentiel pour le Asket EC

=========================================================
QU'EST-CE QUE LA PROPULSION DIFFÉRENTIELLE ?
=========================================================
La propulsion différentielle est le même principe qu'un tank ou un robot
Roomba : deux moteurs indépendants, un à gauche et un à droite.

- Les deux moteurs à la même vitesse  → avancer/reculer en ligne droite
- Moteur gauche plus rapide           → tourner à droite
- Moteur droit plus rapide            → tourner à gauche
- Moteurs en sens opposés             → rotation sur place

C'est simple mais très efficace pour la manœuvrabilité !

=========================================================
CONVERSION TWIST → COMMANDES MOTEURS
=========================================================
Le topic /cmd_vel reçoit des messages Twist :
  linear.x  = vitesse d'avance voulue (m/s)
              positif = avance, négatif = recule
  angular.z = vitesse de rotation voulue (rad/s)
              positif = tourne à gauche, négatif = tourne à droite

La formule de la propulsion différentielle :
  v_port      = v + (L/2) × ω    (moteur gauche)
  v_starboard = v - (L/2) × ω    (moteur droit)

Où :
  v = vitesse linéaire (m/s)
  ω = vitesse angulaire (rad/s)
  L = écartement entre les moteurs (0.50 m)

INTUITION : Si ω > 0 (tourner à gauche), le moteur gauche doit
tourner PLUS vite que le moteur droit pour créer la rotation.
C'est exactement ce que fait la formule (L/2 × ω est positif pour port,
négatif pour starboard).

=========================================================
CONVERSION EN COMMANDE THRUSTER
=========================================================
Le ThrusterPlugin de Gazebo attend une vitesse angulaire d'hélice en rad/s.
On multiplie la vitesse de la roue (m/s) par un facteur MAX_THRUST
pour obtenir la commande finale.

Ce facteur est empirique et doit être ajusté selon les tests.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class DifferentialDriveNode(Node):
    """
    Nœud ROS2 de contrôle différentiel pour le Asket EC.

    Ce nœud écoute les commandes de vitesse sur /cmd_vel et publie
    les vitesses d'hélice appropriées sur les topics des propulseurs.

    Architecture de communication :
    ┌─────────────────┐        /cmd_vel         ┌─────────────────────────┐
    │  Téléopération  │ ────────────────────→   │  DifferentialDriveNode  │
    │  (clavier, joy) │   geometry_msgs/Twist   │     (ce nœud)           │
    └─────────────────┘                         └────────┬────────────────┘
                                                         │
                            /asket_ec/thruster/port/cmd  │
                            ─────────────────────────────┤
                                                         │  std_msgs/Float64
                            /asket_ec/thruster/stbd/cmd  │
                            ─────────────────────────────┘
                                                         │
                                                    Gazebo
                                               (ThrusterPlugin)
    """

    def __init__(self):
        """
        Initialisation du nœud.
        super().__init__('nom_du_noeud') initialise la communication ROS2.
        """
        super().__init__('differential_drive_controller')

        # =========================================================
        # PARAMÈTRES (configurables depuis le launch file ou CLI)
        # =========================================================
        # Les paramètres ROS2 permettent de configurer un nœud sans
        # modifier le code. On peut les changer au lancement :
        # ros2 run asket_ec_control differential_drive_node --ros-args -p wheel_separation:=0.6

        self.declare_parameter('wheel_separation', 0.50)
        # Écartement entre les deux propulseurs (m)
        # 0.50m = 25cm de chaque côté de l'axe central

        self.declare_parameter('max_thrust_rpm', 300.0)
        # Vitesse angulaire max des hélices (rad/s)
        # 300 rad/s ≈ 2865 RPM
        # Ajuster selon la puissance réelle des moteurs BLDC

        self.declare_parameter('max_linear_vel', 2.0)
        # Vitesse linéaire max du bateau (m/s)
        # Utilisée pour normaliser les commandes

        self.declare_parameter('max_angular_vel', 1.0)
        # Vitesse de rotation max (rad/s)
        # π rad/s ≈ 0.5 tour complet par seconde

        # Lire les valeurs des paramètres
        self.L = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.max_thrust = self.get_parameter('max_thrust_rpm').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_vel').get_parameter_value().double_value

        # =========================================================
        # PUBLICATIONS (ce que ce nœud envoie)
        # =========================================================
        # create_publisher(TypeMessage, 'topic', queue_size)
        # queue_size = 10 : bufferiser jusqu'à 10 messages si le subscriber est lent

        # Commande pour le propulseur gauche (port)
        self.pub_port = self.create_publisher(
            Float64,
            '/asket_ec/thruster/port/cmd',
            10
        )

        # Commande pour le propulseur droit (starboard)
        self.pub_starboard = self.create_publisher(
            Float64,
            '/asket_ec/thruster/starboard/cmd',
            10
        )

        # =========================================================
        # SOUSCRIPTIONS (ce que ce nœud reçoit)
        # =========================================================
        # create_subscription(TypeMessage, 'topic', callback, queue_size)
        # callback = fonction appelée à chaque nouveau message reçu

        self.sub_cmd_vel = self.create_subscription(
            Twist,                  # Type de message
            '/cmd_vel',             # Topic ROS2 standard pour les commandes de vitesse
            self.cmd_vel_callback,  # Fonction appelée à chaque message
            10                      # Queue size
        )

        # =========================================================
        # TIMER DE SÉCURITÉ — Watchdog
        # =========================================================
        # Si aucune commande n'est reçue pendant X secondes,
        # arrêter les moteurs automatiquement (sécurité).
        # Cela évite que le bateau continue à avancer si la connexion est coupée.
        self.last_cmd_time = self.get_clock().now()
        self.safety_timeout = 1.0  # secondes avant arrêt d'urgence

        # Timer qui vérifie toutes les 0.1s si une commande a été reçue récemment
        self.safety_timer = self.create_timer(0.1, self.safety_check_callback)

        self.get_logger().info(
            f'Contrôleur différentiel Asket EC démarré !\n'
            f'  Écartement moteurs : {self.L} m\n'
            f'  Poussée max        : {self.max_thrust} rad/s\n'
            f'  En attente de commandes sur /cmd_vel...'
        )

    def cmd_vel_callback(self, msg):
        """
        Callback appelé à chaque message reçu sur /cmd_vel.
        Signature sans annotation de type : rclpy lève un RuntimeError
        "Unable to convert call argument" si la signature est typée.

        Args:
            msg (geometry_msgs.msg.Twist):
                 msg.linear.x  = vitesse d'avance (m/s)
                 msg.angular.z = vitesse de rotation (rad/s)
        """
        # =========================================================
        # ÉTAPE 1 : Extraire et limiter les commandes
        # =========================================================
        # On lit la vitesse linéaire (avance) et angulaire (rotation)
        v = msg.linear.x    # m/s : positif = avance
        w = msg.angular.z   # rad/s : positif = tourne à gauche

        # Saturation des commandes pour éviter des valeurs aberrantes
        # (par exemple si quelqu'un envoie v=100.0 par erreur)
        v = max(-self.max_linear, min(self.max_linear, v))
        w = max(-self.max_angular, min(self.max_angular, w))

        # =========================================================
        # ÉTAPE 2 : Conversion différentielle
        # =========================================================
        # Formule de la propulsion différentielle :
        #
        #   v_port = v + (L/2) * w
        #   v_stbd = v - (L/2) * w
        #
        # Exemple concret avec v=1.0 m/s, w=0.5 rad/s, L=0.5m :
        #   v_port = 1.0 + (0.25 * 0.5) = 1.0 + 0.125 = 1.125 m/s  (plus rapide)
        #   v_stbd = 1.0 - (0.25 * 0.5) = 1.0 - 0.125 = 0.875 m/s  (plus lent)
        # → Le moteur gauche tourne plus vite → le bateau tourne à droite ✓
        v_port = v + (self.L / 2.0) * w       # Vitesse roue gauche (m/s)
        v_starboard = v - (self.L / 2.0) * w  # Vitesse roue droite (m/s)

        # =========================================================
        # ÉTAPE 3 : Conversion en commande ThrusterPlugin
        # =========================================================
        # Le ThrusterPlugin de Gazebo attend une vitesse angulaire d'hélice
        # en rad/s (pas en m/s).
        #
        # On utilise un facteur de conversion linéaire simple :
        # commande = vitesse_roue / vitesse_max * poussée_max_hélice
        #
        # Exemple : v_port = 1.0 m/s, max_linear = 2.0, max_thrust = 300
        # → cmd_port = (1.0 / 2.0) * 300 = 150 rad/s
        if self.max_linear > 0:
            cmd_port = (v_port / self.max_linear) * self.max_thrust
            cmd_starboard = (v_starboard / self.max_linear) * self.max_thrust
        else:
            cmd_port = 0.0
            cmd_starboard = 0.0

        # =========================================================
        # ÉTAPE 4 : Publier les commandes
        # =========================================================
        # On crée un message Float64 pour chaque moteur et on publie.
        msg_port = Float64()
        msg_port.data = float(cmd_port)
        self.pub_port.publish(msg_port)

        msg_starboard = Float64()
        msg_starboard.data = float(cmd_starboard)
        self.pub_starboard.publish(msg_starboard)

        # Mettre à jour le timestamp de dernière commande (pour le watchdog)
        self.last_cmd_time = self.get_clock().now()

        # Log de débogage (s'affiche dans le terminal, utile pour vérifier)
        self.get_logger().debug(
            f'cmd_vel reçu: v={v:.2f} m/s, w={w:.2f} rad/s | '
            f'port={cmd_port:.1f} rad/s, stbd={cmd_starboard:.1f} rad/s'
        )

    def safety_check_callback(self):
        """
        Watchdog de sécurité.
        Appelé toutes les 100ms. Si aucune commande depuis plus de
        `safety_timeout` secondes → arrêt d'urgence des moteurs.

        C'est une mesure de sécurité importante pour éviter que le bateau
        continue à avancer si la connexion avec le poste de contrôle est perdue.
        """
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9  # Convertir en secondes

        if elapsed > self.safety_timeout:
            # Plus de commandes depuis trop longtemps → arrêter les moteurs
            msg_zero = Float64()
            msg_zero.data = 0.0
            self.pub_port.publish(msg_zero)
            self.pub_starboard.publish(msg_zero)
            # Note : on ne log pas ici pour éviter de spammer le terminal


def main(args=None):
    """
    Point d'entrée du nœud ROS2.

    Cette fonction est appelée quand on lance :
      ros2 run asket_ec_control differential_drive_node

    rclpy.init()     : initialise la communication ROS2
    rclpy.spin()     : démarre la boucle d'événements (écoute les topics)
    rclpy.shutdown() : nettoie proprement quand on fait Ctrl+C
    """
    # Initialiser ROS2
    rclpy.init(args=args)

    # Créer le nœud
    node = DifferentialDriveNode()

    try:
        # spin() met le nœud en écoute active.
        # À chaque message reçu sur /cmd_vel, cmd_vel_callback() est appelé.
        # À chaque tick du timer (0.1s), safety_check_callback() est appelé.
        # Cette fonction ne retourne que quand on fait Ctrl+C.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C pressé → arrêt propre
        node.get_logger().info('Arrêt du contrôleur par l\'utilisateur (Ctrl+C)')
    finally:
        # Nettoyer les ressources ROS2
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Permet d'exécuter le fichier directement avec Python (pour les tests)
    # Normalement lancé via `ros2 run` qui appelle main()
    main()
