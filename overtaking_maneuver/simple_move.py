import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class simple_move(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # --- VÁLTOZTATÁS KEZDETE ---

        # Paraméterek: alapértelmezett sebességek robot1 és robot2 számára
        # Lineáris (előre) és anguláris (fordulás) sebességek
        self.declare_parameter('robot1_speed', 0.4)
        self.declare_parameter('robot2_speed', 0.2)
        # Új paraméterek a kanyarodáshoz (rad/s), alapértelmezetten 0.0 (nem kanyarodik)
        self.declare_parameter('robot1_angular_speed', 0.0) 
        self.declare_parameter('robot2_angular_speed', 0.0)
        self.declare_parameter('hz', 10.0)

        # Paraméterek beolvasása
        self.robot1_speed = float(self.get_parameter('robot1_speed').value)
        self.robot2_speed = float(self.get_parameter('robot2_speed').value)
        self.robot1_angular_speed = float(self.get_parameter('robot1_angular_speed').value)
        self.robot2_angular_speed = float(self.get_parameter('robot2_angular_speed').value)
        hz = float(self.get_parameter('hz').value)
        
        # --- VÁLTOZTATÁS VÉGE ---

        # Publisher-ek létrehozása mindkét robot cmd_vel topic-jához
        self.publisher_robot1 = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.publisher_robot2 = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        # Időzítő létrehozása a megadott frekvenciával
        period = 1.0 / hz if hz > 0 else 0.1
        self.timer = self.create_timer(period, self.timer_callback)
        
        # Frissített log üzenet, hogy az új paramétereket is mutassa
        self.get_logger().info(
            f'Robot Controller elindult: \n'
            f'Robot1: linear={self.robot1_speed} m/s, angular={self.robot1_angular_speed} rad/s\n'
            f'Robot2: linear={self.robot2_speed} m/s, angular={self.robot2_angular_speed} rad/s\n'
            f'Frekvencia: {hz} Hz'
        )

    def timer_callback(self):
        # Előre mozgást ÉS kanyarodást definiáló Twist üzenetek
        
        # --- VÁLTOZTATÁS KEZDETE ---
        
        msg1 = Twist()
        msg1.linear.x = self.robot1_speed  # robot1 lineáris sebessége (m/s)
        msg1.linear.y = 0.0
        msg1.linear.z = 0.0
        msg1.angular.x = 0.0
        msg1.angular.y = 0.0
        msg1.angular.z = self.robot1_angular_speed # robot1 szögsebessége (rad/s)

        msg2 = Twist()
        msg2.linear.x = self.robot2_speed  # robot2 lineáris sebessége (m/s)
        msg2.linear.y = 0.0
        msg2.linear.z = 0.0
        msg2.angular.x = 0.0
        msg2.angular.y = 0.0
        msg2.angular.z = self.robot2_angular_speed # robot2 szögsebessége (rad/s)
        
        # --- VÁLTOZTATÁS VÉGE ---

        # Parancs publikálása mindkét robotnak
        self.publisher_robot1.publish(msg1)
        self.publisher_robot2.publish(msg2)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = simple_move()
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Node leállítása és erőforrások felszabadítása
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()