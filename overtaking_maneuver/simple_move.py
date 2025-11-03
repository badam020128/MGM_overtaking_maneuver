import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class simple_move(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Paraméterek: alapértelmezett sebességek robot1 és robot2 számára
        # robot2_speed alapértelmezésben nagyobb, így a hátul jövő robot gyorsabb lesz
        # Make robot1 the faster one by default
        self.declare_parameter('robot1_speed', 0.4)
        self.declare_parameter('robot2_speed', 0.2)
        self.declare_parameter('hz', 10.0)

        self.robot1_speed = float(self.get_parameter('robot1_speed').value)
        self.robot2_speed = float(self.get_parameter('robot2_speed').value)
        hz = float(self.get_parameter('hz').value)

        # Publisher-ek létrehozása mindkét robot cmd_vel topic-jához
        self.publisher_robot1 = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.publisher_robot2 = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        # Időzítő létrehozása a megadott frekvenciával
        period = 1.0 / hz if hz > 0 else 0.1
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(f'Robot Controller elindult: robot1_speed={self.robot1_speed}, robot2_speed={self.robot2_speed}, hz={hz}')

    def timer_callback(self):
        # Előre mozgást definiáló Twist üzenetek külön a két robothoz
        msg1 = Twist()
        msg1.linear.x = self.robot1_speed  # robot1 sebessége (m/s)
        msg1.linear.y = 0.0
        msg1.linear.z = 0.0
        msg1.angular.x = 0.0
        msg1.angular.y = 0.0
        msg1.angular.z = 0.0

        msg2 = Twist()
        msg2.linear.x = self.robot2_speed  # robot2 (hátul jövő) sebessége (m/s)
        msg2.linear.y = 0.0
        msg2.linear.z = 0.0
        msg2.angular.x = 0.0
        msg2.angular.y = 0.0
        msg2.angular.z = 0.0

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