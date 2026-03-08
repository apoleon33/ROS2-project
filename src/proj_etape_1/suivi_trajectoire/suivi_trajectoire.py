#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path


class SuiviTrajectoire(Node):
    def __init__(self):
        super().__init__("suivi_de_trajectoire")
        # Publishers et Subscriber
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/trajectory_real", 10)
        self.ref_pub = self.create_publisher(Path, "/trajectory_ref", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Initialisation des messages Path
        self.path_real = Path()
        self.path_real.header.frame_id = "odom"
        self.path_ref = Path()
        self.path_ref.header.frame_id = "odom"

        # Gains de la loi de commande (à ajuster lors de vos tests)
        self.k_rho = 0.6
        self.k_alpha = 1.5
        self.k_beta = -0.6

        # Sécurités / Saturations
        self.v_max = 0.22
        self.w_max = 1.0
        self.rho_stop = 0.05  # seuil d'arrêt proche de la cible

        # suivi de trajectoire
        self.waypoints = [  # de la forme [x,y,theta]
            (0.0, 0.0, 0.0),
            (1.0, 1.0, math.pi / 2),
            (-2.0, 4.0, math.pi / 2),
            (-2.0, 0.0, 0.0)
        ]
        self.currentWaypoints = 0 # l'objectif prochain
        self.xr, self.yr, self.theta_p = self.waypoints[self.currentWaypoints]

        self.get_logger().info("Suivi de trajectoire")

    @staticmethod
    def wrap_to_pi(a: float) -> float:
        """Ramener un angle dans ]-pi, pi]."""
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def odom_callback(self, msg: Odometry):
        # MAJ de la ref
        self.xr, self.yr, self.theta_p = self.waypoints[self.currentWaypoints]

        # Etat actuel du robot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Conversion quaternion -> yaw pour theta
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # Calcul des variables d'erreur polaires
        dx_err = self.xr - x
        dy_err = self.yr - y
        rho = math.sqrt(dx_err * dx_err + dy_err * dy_err)

        angle_to_goal = math.atan2(dy_err, dx_err)
        alpha = self.wrap_to_pi(angle_to_goal - theta)
        beta = self.wrap_to_pi(self.theta_p - theta - alpha)

        # Calcul des commandes de vitesse via PID
        v = rho * self.k_rho
        w = self.k_alpha * alpha + self.k_beta * beta

        if rho < self.rho_stop:
            # Au lieu d'arrêter le robot, on passe au waypoint suivant
            if self.currentWaypoints < len(self.waypoints) - 1:
                self.currentWaypoints += 1
            else:
                v = 0.0
                w = 0.0

        # Saturations des vitesses
        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))

        # Publication de la commande
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

        # Trajectoire de référence
        pose_ref = PoseStamped()
        pose_ref.header.frame_id = "odom"
        pose_ref.header.stamp = self.get_clock().now().to_msg()
        pose_ref.pose.position.x = self.xr
        pose_ref.pose.position.y = self.yr
        pose_ref.pose.orientation.w = math.cos(self.theta_p / 2.0)
        pose_ref.pose.orientation.z = math.sin(self.theta_p / 2.0)

        self.path_ref.poses.append(pose_ref)
        self.path_ref.header.stamp = self.get_clock().now().to_msg()
        self.ref_pub.publish(self.path_ref)


def main():
    rclpy.init()
    node = SuiviTrajectoire()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
