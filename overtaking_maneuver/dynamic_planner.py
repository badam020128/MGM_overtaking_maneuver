#!/usr/bin/env python3
"""
Dinamikus, 3-pontos trajektória tervező az előzési manőverhez.

Feliratkozik mindkét robot odometriájára, hogy kövesse a célpontot.
A manővert a felhasználó által kért P1->P2->P3 logika alapján tervezi meg
két lépcsőben (két Quintic Polynomial pálya).

ÁLLAPOTGÉP:
1. FOLLOWING: Követi robot2-t. Figyeli a távolságot.
2. PLANNING_CHANGEOVER: Túl közel került. Tervezi az 1. pályát (P1->P2).
3. CHANGING_LANE: Végzi a sávváltást (vár, amíg eléri P2-t).
4. PLANNING_MERGE: Elérte P2-t. Tervezi a 2. pályát (P2->P3).
5. MERGING_BACK: Végzi a visszatérést (vár, amíg eléri P3-at).
6. (Vissza FOLLOWING-ra)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import tf_transformations

# --- Innen másold be a Quintic Polynomial Planner kódot ---
# (Ugyanaz, mint korábban, itt most a helykihagyás miatt nem ismétlem meg)

# parameter
MAX_T = 100.0  # maximum time to the goal [s]
MIN_T = 4.0  # minimum time to the goal[s]

class QuinticPolynomial:
    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):
        # calc coefficient of quintic polynomial
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0
        A = np.array([[time ** 3, time ** 4, time ** 5],
                      [3 * time ** 2, 4 * time ** 3, 5 * time ** 4],
                      [6 * time, 12 * time ** 2, 20 * time ** 3]])
        b = np.array([xe - self.a0 - self.a1 * time - self.a2 * time ** 2,
                      vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)
        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]
    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5
        return xt
    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4
        return xt
    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3
        return xt
    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2
        return xt

def quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt):
    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)
    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)
    time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []
    for T in np.arange(MIN_T, MAX_T, MIN_T):
        xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T)
        yqp = QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, T)
        time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []
        for t in np.arange(0.0, T + dt, dt):
            time.append(t)
            rx.append(xqp.calc_point(t))
            ry.append(yqp.calc_point(t))
            vx = xqp.calc_first_derivative(t)
            vy = yqp.calc_first_derivative(t)
            v = np.hypot(vx, vy)
            yaw = math.atan2(vy, vx)
            rv.append(v)
            ryaw.append(yaw)
            ax = xqp.calc_second_derivative(t)
            ay = yqp.calc_second_derivative(t)
            a = np.hypot(ax, ay)
            if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
                a *= -1
            ra.append(a)
            jx = xqp.calc_third_derivative(t)
            jy = yqp.calc_third_derivative(t)
            j = np.hypot(jx, jy)
            if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
                j *= -1
            rj.append(j)
        if max([abs(i) for i in ra]) <= max_accel and max([abs(i) for i in rj]) <= max_jerk:
            break
    return time, rx, ry, ryaw, rv, ra, rj

# --- Eddig tart a Quintic Polynomial Planner kód ---


class DynamicTrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('dynamic_trajectory_planner')

        # Tervezési paraméterek (a te logikád alapján)
        # A sávközéppontok Y koordinátái (a road.world alapján)
        self.declare_parameter('home_lane_y', -1.75)
        self.declare_parameter('oncoming_lane_y', 1.75)
        
        # Milyen közel engedjük, mielőtt előzünk (P1 trigger)
        self.declare_parameter('trigger_distance', 4.0) 
        
        # Milyen messze P2 "mellette" legyen P1-hez képest (X-ben)
        self.declare_parameter('p2_lookahead_x', 8.0) 
        
        # Milyen messze "előtte" legyen P3 P2-höz képest (X-ben)
        self.declare_parameter('p3_lookahead_x', 8.0) 
        
        # Mennyivel menjünk gyorsabban előzés közben
        self.declare_parameter('overtake_speed_boost', 0.4) # m/s
        
        # Biztonsági távolság robot2 előtt, mielőtt visszatérünk
        self.declare_parameter('merge_safety_distance', 2.0)  # meters


        # Állapotgép
        self.state = 'FOLLOWING'
        self.odom_robot1 = None
        self.odom_robot2 = None
        self.scan_robot1 = None
        
        # A P2 és P3 célpontok tárolása (mivel dinamikusak)
        self.goal_P2 = None # (x, y, yaw, v, a)
        self.goal_P3 = None # (x, y, yaw, v, a)

        # Subscriberek
        self.sub_odom1 = self.create_subscription(
            Odometry, '/robot1/odom', self.odom1_callback, 10)
        
        # KULCSFONTOSSÁGÚ: Feliratkozás robot2-re is!
        self.sub_odom2 = self.create_subscription(
            Odometry, '/robot2/odom', self.odom2_callback, 10)
        
        self.sub_scan1 = self.create_subscription(
            LaserScan, '/robot1/scan', self.scan1_callback, 10)
        
        # Publisher a tervezett pályának
        self.path_pub = self.create_publisher(Path, '/robot1/planned_path', 10)
        
        # Az állapotgép futtatása időzítővel (10 Hz)
        self.timer = self.create_timer(0.1, self.main_loop)
        
        self.get_logger().info("Dinamikus 3-pontos tervező elindult.")

    # --- Adatgyűjtő Callback-ek ---
    def odom1_callback(self, msg): self.odom_robot1 = msg
    def odom2_callback(self, msg): self.odom_robot2 = msg
    def scan1_callback(self, msg): self.scan_robot1 = msg

    # --- Az ÁLLAPOTGÉP ---
    def main_loop(self):
        # Csak akkor futunk, ha minden adat megérkezett
        if not all([self.odom_robot1, self.odom_robot2, self.scan_robot1]):
            self.get_logger().warn("Várakozás az odometria és LIDAR adatokra...", skip_first=True, throttle_duration_sec=5)
            return

        # Állapotok kezelése
        if self.state == 'FOLLOWING':
            self.check_for_overtake_trigger()
        
        elif self.state == 'PLANNING_CHANGEOVER':
            self.plan_P1_to_P2()
            self.state = 'CHANGING_LANE' # Tervezés után azonnal váltunk

        elif self.state == 'CHANGING_LANE':
            self.check_if_P2_reached()
        
        elif self.state == 'PLANNING_MERGE':
            self.plan_P2_to_P3()
            self.state = 'MERGING_BACK'

        elif self.state == 'MERGING_BACK':
            self.check_if_P3_reached()

    # --- ÁLLAPOTVÁLTÓ FÜGGVÉNYEK ---

    def check_for_overtake_trigger(self):
        # Távolság számítása a két robot ODOM-ja alapján
        pos1 = self.odom_robot1.pose.pose.position
        pos2 = self.odom_robot2.pose.pose.position
        dist_x = pos2.x - pos1.x
        dist_y = pos2.y - pos1.y
        
        trigger_dist = self.get_parameter('trigger_distance').value

        # LIDAR ellenőrzés (bal sáv)
        if len(self.scan_robot1.ranges) < 91: return
        dist_left = self.scan_robot1.ranges[90]
        
        # A te logikád:
        # 1. Ha robot2 ELŐTTÜNK van (dist_x > 0)
        # 2. ÉS túl közel van (dist_x < trigger_dist)
        # 3. ÉS a sávunkban van (abs(dist_y) < 1.0)
        # 4. ÉS a bal sáv TISZTA (LIDAR 90 fok > sávváltás helye)
        if (dist_x > 0 and dist_x < trigger_dist and 
            abs(dist_y) < 1.0 and dist_left > 3.0):
            
            self.get_logger().info(f"*** ELŐZÉS INDÍTÁSA (Táv: {dist_x:.2f}m) ***")
            self.get_logger().info("1. LÉPÉS: Pálya tervezése P2-be (mellette).")
            self.state = 'PLANNING_CHANGEOVER'

    def plan_P1_to_P2(self):
        # --- P1 (Start) ---
        # Az aktuális állapotunk
        (sx, sy, syaw, sv, sa) = self.get_current_state(self.odom_robot1)

        # --- P2 (Cél 1: Mellette) ---
        # A P2-t robot2 POZÍCIÓJÁHOZ ÉS SEBESSÉGÉHEZ viszonyítjuk
        pos2 = self.odom_robot2.pose.pose.position
        vel2 = self.odom_robot2.twist.twist.linear.x
        
        p2_lookahead = self.get_parameter('p2_lookahead_x').value
        
        # P2 Cél Y: A szembesáv közepe
        gy = self.get_parameter('oncoming_lane_y').value
        
        # P2 Cél X: Az aktuális pozíciónk + X eltolás
        # (Dinamikusabb lenne robot2 X-éhez kötni, de maradjunk az egyszerűbbnél)
        gx = sx + p2_lookahead
        
        # P2 Cél Yaw: Egyenesen
        gyaw = 0.0
        
        # P2 Cél Sebesség: Gyorsabban, mint robot2
        gv = vel2 + self.get_parameter('overtake_speed_boost').value
        ga = 0.0 # Cél gyorsulás
        
        # Elmentjük a P2 célt, hogy tudjuk, mikor értük el
        self.goal_P2 = (gx, gy) 

        self.get_logger().info(f"Tervezés P1->P2: Start=({sx:.1f}, {sy:.1f}), Cél=({gx:.1f}, {gy:.1f}), CélSebesség={gv:.1f} m/s")
        
        # Tervezés és Publikálás
        self.run_planner_and_publish(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga)

    def check_if_P2_reached(self):
        if self.goal_P2 is None: return
        
        pos1 = self.odom_robot1.pose.pose.position
        pos2 = self.odom_robot2.pose.pose.position
        
        # Euklideszi távolság a P2 célponthoz
        dist_to_P2 = math.sqrt((pos1.x - self.goal_P2[0])**2 + (pos1.y - self.goal_P2[1])**2)
        
        # Távolság ellenőrzése robot2-től (X irányban mennyivel vagyunk előtte)
        safety_distance = self.get_parameter('merge_safety_distance').value
        distance_ahead = pos1.x - pos2.x
        
        # Ha elértük P2-t és megvan a biztonsági távolság
        if distance_ahead > safety_distance:
            self.get_logger().info(f"P2 elérve és biztonságos távolság ({distance_ahead:.1f}m) > {safety_distance}m")
            
            self.get_logger().info("Elkezdjük a visszatérést.")
            
            self.get_logger().info("2. LÉPÉS: Pálya tervezése P3-ba (előtte).")
            self.goal_P2 = None # Töröljük a célt
            self.state = 'PLANNING_MERGE'

    def plan_P2_to_P3(self):
        # --- P2_aktuális (Start) ---
        # Az aktuális állapotunk (ami ~P2)
        (sx, sy, syaw, sv, sa) = self.get_current_state(self.odom_robot1)

        # --- P3 (Cél 2: Előtte) ---
        # P3-at robot2 AKTUÁLIS pozíciójához viszonyítjuk
        pos2 = self.odom_robot2.pose.pose.position
        vel2 = self.odom_robot2.twist.twist.linear.x
        
        p3_lookahead = self.get_parameter('p3_lookahead_x').value

        # P3 Cél Y: A hazai sáv közepe
        gy = self.get_parameter('home_lane_y').value
        
        # P3 Cél X: robot2 X-e + biztonságos távolság
        # (Itt már a robot2 X-e számít, nem a sajátunk!)
        gx = pos2.x + p3_lookahead
        
        # P3 Cél Yaw: Egyenesen
        gyaw = 0.0
        
        # P3 Cél Sebesség: Tartsuk az előzési sebességet (sv)
        gv = sv 
        ga = 0.0
        
        # Elmentjük a P3 célt
        self.goal_P3 = (gx, gy)

        self.get_logger().info(f"Tervezés P2->P3: Start=({sx:.1f}, {sy:.1f}), Cél=({gx:.1f}, {gy:.1f})")

        # Tervezés és Publikálás
        self.run_planner_and_publish(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga)

    def check_if_P3_reached(self):
        if self.goal_P3 is None: return

        pos1 = self.odom_robot1.pose.pose.position
        dist_to_P3 = math.sqrt((pos1.x - self.goal_P3[0])**2 + (pos1.y - self.goal_P3[1])**2)
        
        if dist_to_P3 < 1.0: # 1.0 méteres tolerancia
            self.get_logger().info("*** ELŐZÉS BEFEJEZVE (P3 elérve) ***")
            self.goal_P3 = None
            self.state = 'FOLLOWING' # Vissza a ciklus elejére


    # --- SEGÉDFÜGGVÉNYEK ---

    def get_current_state(self, odom_msg):
        """Kinyeri a pillanatnyi állapotot az odometria üzenetből."""
        pos = odom_msg.pose.pose.position
        orient = odom_msg.pose.pose.orientation
        twist = odom_msg.twist.twist
        
        sx = pos.x
        sy = pos.y
        syaw = self.get_yaw_from_quaternion(orient)
        sv = twist.linear.x
        sa = 0.0 # Gyorsulást most nem becsülünk, 0-nak vesszük
        
        return (sx, sy, syaw, sv, sa)

    def run_planner_and_publish(self, sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga):
        """Lefuttatja a tervezőt és publikálja az eredményt."""
        max_accel = 1.0  # m/ss
        max_jerk = 0.5   # m/sss
        dt = 0.1         # time tick [s]

        try:
            time, x, y, yaw, v, a, j = quintic_polynomials_planner(
                sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt)
            
            self.publish_path(x, y, time)
        except np.linalg.LinAlgError as e:
            self.get_logger().warn(f"Tervezés sikertelen (LinAlgError): {e}")
            # Visszaállunk követő módba, ha a tervezés hibát dob
            self.state = 'FOLLOWING'


    def publish_path(self, x_coords, y_coords, times):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map" # Vagy "odom"

        for i in range(len(x_coords)):
            pose = PoseStamped()
            # Az időbélyeg a ROS szimulációs idő, nem a pályapont ideje
            pose.header.stamp = path_msg.header.stamp 
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = x_coords[i]
            pose.pose.position.y = y_coords[i]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Tervezett pálya publikálva ({len(path_msg.poses)} pont).")

    def get_yaw_from_quaternion(self, orientation_q):
        q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        euler = tf_transformations.euler_from_quaternion(q)
        return euler[2] # yaw


def main(args=None):
    rclpy.init(args=args)
    try:
        planner_node = DynamicTrajectoryPlannerNode()
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            planner_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()