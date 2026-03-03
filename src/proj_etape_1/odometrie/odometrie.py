#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path


class Odometrie(Node):
    def __init__(self):
        super().__init__("odometrie")

        # Publisher commande vitesse
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Publisher trajectoire pour RViz
        self.path_pub = self.create_publisher(Path, "/trajectory", 10)

        # Subscriber odométrie
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Message Path
        self.path = Path()
        self.path.header.frame_id = "odom"

        # Paramètres de la boucle ouverte
        self.v = 0.20

        # vitesse linéaire (m/s)
        self.w = 0.20

        # vitesse angulaire (rad/s)
        self.duration = 10.0  # durée du mouvement (s)

        # Temps initial
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # Timer de commande (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Projet Etape 1: localisation par odométrie.")

    def control_loop(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        cmd = Twist()
        if t < self.duration:
            cmd.linear.x = self.v
            cmd.angular.z = self.w
        else:
            # Arrêt propre du robot
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)
        


def main():
    rclpy.init()
    node = Odometrie()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
