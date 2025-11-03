import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class simple_move(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Publisher-ek létrehozása mindkét robot cmd_vel topic-jához
        self.publisher_robot1 = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.publisher_robot2 = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        
        # Időzítő létrehozása, ami 10 Hz-en (0.1 másodpercenként) futtatja a callback-et
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Robot Controller elindult, mindkét robotot előre mozgatja.')

    def timer_callback(self):
        # Előre mozgást definiáló Twist üzenet
        msg = Twist()
        msg.linear.x = 0.2  # Előre mozgás sebessége (m/s)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # Parancs publikálása mindkét robotnak
        self.publisher_robot1.publish(msg)
        self.publisher_robot2.publish(msg)

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