#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped

class StanleyControllerNode(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        # params
        self.declare_parameter('k', 1.0)              # gain for crosstrack term
        self.declare_parameter('k_soft', 1.0)         # softening term for low speeds
        self.declare_parameter('max_angular', 1.5)    # rad/s limit
        self.declare_parameter('target_speed', 0.8)   # desired forward speed (m/s)
        self.declare_parameter('cmd_topic', '/robot1/cmd_vel')
        self.declare_parameter('path_topic', '/robot1/planned_path')
        self.declare_parameter('odom_topic', '/robot1/odom')

        self.k = self.get_parameter('k').value
        self.k_soft = self.get_parameter('k_soft').value
        self.max_angular = self.get_parameter('max_angular').value
        self.target_speed = self.get_parameter('target_speed').value

        # state
        self.current_odom = None
        self.current_path = None

        # subs & pubs
        self.sub_path = self.create_subscription(Path, self.get_parameter('path_topic').value, self.path_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 30)
        self.pub_cmd = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)

        # timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('Stanley controller started.')

    def path_cb(self, msg: Path):
        # store path as list of (x,y,yaw)
        pts = []
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            # yaw extraction (fallback if orientation is zero)
            q = p.pose.orientation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            pts.append((x, y, yaw))
        self.current_path = pts

    def odom_cb(self, msg: Odometry):
        self.current_odom = msg

    def control_loop(self):
        if self.current_odom is None or self.current_path is None or len(self.current_path) < 2:
            return

        # current pose
        p = self.current_odom.pose.pose.position
        q = self.current_odom.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        vx = self.current_odom.twist.twist.linear.x

        cx, cy, cyaw, c_idx = self.find_nearest_point_on_path(p.x, p.y, self.current_path)

        # heading error
        heading_error = self.normalize_angle(cyaw - yaw)

        # cross-track error: signed distance from vehicle to path
        # compute vector from closest path point to vehicle and sign by cross product with path tangent
        dx = p.x - cx
        dy = p.y - cy
        # path tangent
        tx = math.cos(cyaw)
        ty = math.sin(cyaw)
        # signed crosstrack = cross(t, vec)
        crosstrack = (dx * ty - dy * tx)

        # Stanley control law -> produce angular velocity (for differential drive)
        v = max(vx, 1e-3)  # avoid divide by zero, use current speed if available
        steer_term = math.atan2(self.k * crosstrack, v + self.k_soft)
        angular = heading_error + steer_term

        # clamp
        angular = max(-self.max_angular, min(self.max_angular, angular))

        # produce Twist: forward speed is target_speed (could be adapted)
        cmd = Twist()
        cmd.linear.x = self.target_speed
        cmd.angular.z = angular

        self.pub_cmd.publish(cmd)

    def find_nearest_point_on_path(self, x, y, path):
        # returns closest point (x,y,yaw) and its index
        d_min = float('inf')
        closest = (path[0][0], path[0][1], path[0][2], 0)
        for i in range(len(path)-1):
            x1, y1, _ = path[i]
            x2, y2, _ = path[i+1]
            # project point (x,y) onto segment [p1,p2]
            dx = x2 - x1
            dy = y2 - y1
            if dx == 0 and dy == 0:
                proj_x, proj_y = x1, y1
            else:
                t = ((x - x1)*dx + (y - y1)*dy) / (dx*dx + dy*dy)
                t = max(0.0, min(1.0, t))
                proj_x = x1 + t*dx
                proj_y = y1 + t*dy
            d = math.hypot(x - proj_x, y - proj_y)
            if d < d_min:
                d_min = d
                yaw_seg = math.atan2(dy, dx)
                closest = (proj_x, proj_y, yaw_seg, i)
        return closest

    def normalize_angle(self, ang):
        while ang > math.pi:
            ang -= 2*math.pi
        while ang < -math.pi:
            ang += 2*math.pi
        return ang

def main(args=None):
    rclpy.init(args=args)
    node = StanleyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()