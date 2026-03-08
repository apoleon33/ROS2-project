#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64 


class PID:
    """
    classe créée à partir de https://www.digikey.fr/en/maker/tutorials/2024/implementing-a-pid-controller-algorithm-in-python
    """

    previousError: float  # erreur precedente, pour le D de PID
    integral: float  # le I de PID, on incremente dessus donc on le garde en attribut

    # gains
    Kp: float
    Ki: float
    Kd: float

    def __init__(self, Kp=1.0, Kd=1.0, Ki=1.0):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

        self.previousError = 0.0
        self.integral = 0.0

    def correct(self, error: float, dt: float) -> float:
        """Applique le PID à l'erreur"""
        proportionnal = error
        self.integral += error * dt
        derive = (error - self.previousError) / dt

        return self.Kp * proportionnal + self.Ki * self.integral + self.Kd * derive


class ClosedLoopController(Node):
    def __init__(self):
        super().__init__("closed_loop_controller")
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/trajectory_real', 10)
        self.ref_pub = self.create_publisher(Path, '/trajectory_ref', 10)
        self.error_rho_pub = self.create_publisher(Float64, '/debug/rho', 10) # Pour afficher la réponser temporelle

        # Subscriber
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Trajectoires
        self.path_real = Path()
        self.path_real.header.frame_id = 'odom'
        self.path_ref = Path()
        self.path_ref.header.frame_id = 'odom'

        # Référence (point cible)
        self.xr = 2.0
        self.yr = 2.0

        # Orientation finale désirée (theta_p). Tu peux la changer.
        self.theta_p = 0.0

        # Gains (loi polaire)
        self.k_rho = 0.6
        # > 0
        self.k_alpha = 1.5
        # > 0
        self.k_beta = -0.6
        # < 0

        # Sécurité / saturation (optionnel mais recommandé)
        self.v_max = 0.22
        self.w_max = 1.0
        self.rho_stop = 0.05 # seuil d'arrêt proche de la cible

        # PID
        self.pidV = PID(0.6, 0.0, 0.4)
        self.pidOmega = PID(1.5,0.0,0.6)
        self.last_time = self.get_clock().now().nanoseconds * 1e-9 # On conserve une trace pour le dt

        self.get_logger().info("Contrôleur PID")

    @staticmethod
    def wrap_to_pi(a: float) -> float:
        """Ramener un angle dans ]-pi, pi]."""
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def odom_callback(self, msg: Odometry):
        # Etat du robot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Conversion quaternion -> yaw
        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # Erreurs polaires
        dx = self.xr - x
        dy = self.yr - y
        rho = math.sqrt(dx * dx + dy * dy)
        angle_to_goal = math.atan2(dy, dx)

        # alpha = -theta + atan2(dy, dx)
        alpha = self.wrap_to_pi(angle_to_goal - theta)

        # beta = theta_p - theta - alpha (pour imposer une orientation finale theta_p)
        beta = self.wrap_to_pi(self.theta_p - theta - alpha)

        current_time = self.get_clock().now().nanoseconds * 1e-9
        dt = current_time - self.last_time
        self.last_time = current_time


        # Calcul des commandes de vitesse via PID
        v = self.pidV.correct(rho, dt)
        w = self.pidOmega.correct(alpha, dt)

        # Arrêt proche de la cible (évite de tourner/osciller sur place)
        if rho < self.rho_stop:
            v = 0.0
            w = 0.0

        # Saturations (optionnel)
        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))

        # Publier commande
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        # Trajectoire réelle
        pose_real = PoseStamped()
        pose_real.header = msg.header
        pose_real.pose = msg.pose.pose
        self.path_real.poses.append(pose_real)
        self.path_real.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_real)

        # Trajectoire de référence (point cible)
        pose_ref = PoseStamped()
        pose_ref.header.frame_id = 'odom'
        pose_ref.header.stamp = self.get_clock().now().to_msg()
        pose_ref.pose.position.x = self.xr
        pose_ref.pose.position.y = self.yr
        
        # orientation de référence (theta_p)
        pose_ref.pose.orientation.w = math.cos(self.theta_p / 2.0)
        pose_ref.pose.orientation.z = math.sin(self.theta_p / 2.0)
        self.path_ref.poses.append(pose_ref)
        self.path_ref.header.stamp = self.get_clock().now().to_msg()
        self.ref_pub.publish(self.path_ref)

        # Erreur rho
        msg_rho = Float64()
        msg_rho.data = float(rho)
        self.error_rho_pub.publish(msg_rho)



def main():
    rclpy.init()
    node = ClosedLoopController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
