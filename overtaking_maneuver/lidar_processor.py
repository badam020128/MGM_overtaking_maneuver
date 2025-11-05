#!/usr/bin/env python3
"""
ROS 2 Node a LIDAR adatok feldolgozásához.
Feliratkozik egy LaserScan topicra, és kinyeri a távolságadatokat.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarProcessorNode(Node):
    def __init__(self):
        # A node neve 'lidar_processor', de a ROS 2 átnevezi majd
        # a launch fájlban megadott 'name' attribútum alapján (pl. 'lidar_robot1')
        super().__init__('lidar_processor')

        # Paraméter deklarálása, hogy megadjuk, melyik robot topicjára iratkozzunk fel
        # Alapértelmezett értéke 'robot1'
        self.declare_parameter('robot_name', 'robot1')
        robot_name = self.get_parameter('robot_name').value

        # A topic nevének összeállítása a paraméter alapján
        scan_topic = f'/{robot_name}/scan'

        # Subscriber létrehozása a LaserScan topicra
        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10) # QoS profil, 10-es "mélység"
        
        self.get_logger().info(f"LIDAR processzor elindult, figyel: {scan_topic}")

    def format_distance(self, distance):
        """Segédfüggvény a távolság formázására (NaN és Inf kezelése)."""
        if math.isinf(distance):
            return "Végtelen (Nincs fal)"
        elif math.isnan(distance):
            return "Érvénytelen (NaN)"
        else:
            return f"{distance:.2f} m"

    def scan_callback(self, msg):
        """
        Ez a függvény minden alkalommal lefut, amikor új LaserScan üzenet érkezik.
        """
        
        # Ellenőrizzük, hogy a LIDAR ad-e elég adatpontot (minimum 271-et a 270-es indexhez)
        if len(msg.ranges) < 271:
            self.get_logger().warn(
                f"A LIDAR kevesebb ({len(msg.ranges)}) pontot ad, mint a várt 271. "
                f"Nem tudom olvasni az oldalsó távolságokat."
            )
            return

        # Távolságok kiolvasása a megfelelő indexekről
        distance_forward = msg.ranges[0]   # 0 fok (Előre)
        distance_left = msg.ranges[90]     # 90 fok (Balra)
        distance_right = msg.ranges[270]   # 270 fok (Jobbra)

        # Formázzuk a kiírásokat
        f_str = self.format_distance(distance_forward)
        l_str = self.format_distance(distance_left)
        r_str = self.format_distance(distance_right)

        # --- VÁLTOZTATÁS KEZDETE ---
        # Kiírjuk az információt a node logjába, a kérésnek megfelelően 3 sorba
        log_message = (
            f"LIDAR Mérés:\n"
            f"  1. Előre: [{f_str}]\n"
            f"  2. Jobbra: [{r_str}]\n"
            f"  3. Balra: [{l_str}]"
        )
        self.get_logger().info(log_message)
        # --- VÁLTOZTATÁS VÉGE ---

def main(args=None):
    rclpy.init(args=args)
    
    try:
        lidar_processor_node = LidarProcessorNode()
        rclpy.spin(lidar_processor_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            lidar_processor_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()