#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np

def euler_from_quaternion(quaternion):
    """
    Konversi quaternion ke Euler angles (roll, pitch, yaw).
    yaw adalah rotasi pada sumbu z.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class SMCController(Node):
    def __init__(self):
        super().__init__('smc_path_follower')

        # --- Deklarasi Parameter Kontrol SMC dan Robot ---
        # Anda dapat menyesuaikan nilai-nilai ini melalui file launch atau parameter
        self.declare_parameter('lambda_cte', 1.0)      # Bobot untuk cross-track error dalam sliding surface
        self.declare_parameter('lambda_psi', 2.0)      # Gain Proportional untuk heading error
        self.declare_parameter('k_gain', 1.5)          # Gain SMC (seberapa agresif kembali ke jalur)
        self.declare_parameter('phi_boundary', 0.5)    # Tebal boundary layer (untuk memperhalus kontrol)
        self.declare_parameter('forward_velocity', 0.8)  # Kecepatan linear konstan robot (m/s)
        self.declare_parameter('goal_tolerance', 0.2)  # Jarak toleransi untuk mencapai tujuan (meter)

        # --- Ambil nilai parameter ---
        self.lambda_cte_ = self.get_parameter('lambda_cte').get_parameter_value().double_value
        self.lambda_psi_ = self.get_parameter('lambda_psi').get_parameter_value().double_value
        self.k_ = self.get_parameter('k_gain').get_parameter_value().double_value
        self.phi_ = self.get_parameter('phi_boundary').get_parameter_value().double_value
        self.v_ = self.get_parameter('forward_velocity').get_parameter_value().double_value
        self.goal_tol_ = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        # --- Definisikan Trajektori (Jalur Lurus) ---
        # Cukup definisikan titik awal dan akhir untuk jalur lurus
        self.start_waypoint_ = (0.0, 0.0)
        self.goal_waypoint_ = (20.0, 0.0) # Tujuan: 20 meter lurus ke depan
        self.path_completed = False

        # --- Inisialisasi State Robot ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # --- Subscriber & Publisher ---
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10) # Publikasi ke input twist_mux
        self.timer = self.create_timer(0.02, self.control_loop) # Loop kontrol berjalan setiap 20ms

        self.get_logger().info('SMC Path Follower Node has been started.')
        self.get_logger().info(f"Parameters: lambda_cte={self.lambda_cte_}, k={self.k_}, phi={self.phi_}")

    def odom_callback(self, msg):
        """Callback untuk mengupdate posisi dan orientasi robot dari topic /odom."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = euler_from_quaternion(msg.pose.pose.orientation)
        
        if not self.odom_received:
            # Set titik awal aktual robot saat odometry pertama kali diterima
            self.start_waypoint_ = (self.robot_x, self.robot_y)
            self.odom_received = True

    def control_loop(self):
        """Loop utama yang menghitung dan mengirimkan perintah kontrol."""
        if not self.odom_received or self.path_completed:
            self.stop_robot()
            return

        # Cek apakah robot sudah mencapai tujuan
        dist_to_goal = math.sqrt((self.goal_waypoint_[0] - self.robot_x)**2 + (self.goal_waypoint_[1] - self.robot_y)**2)
        if dist_to_goal < self.goal_tol_:
            if not self.path_completed:
                self.get_logger().info('Goal Reached!')
                self.path_completed = True
            self.stop_robot()
            return

        # =================================================================
        # === INTI PERHITUNGAN SLIDING MODE CONTROL (SMC) ===
        # =================================================================

        # 1. Hitung Error
        #    a. Cross-Track Error (e_cte): Jarak tegak lurus robot ke garis trajektori
        y2, x2 = self.goal_waypoint_
        y1, x1 = self.start_waypoint_
        e_cte = ((y2 - y1) * self.robot_x - (x2 - x1) * self.robot_y + x2 * y1 - y2 * x1) / (math.sqrt((y2-y1)**2 + (x2-x1)**2) + 1e-9)

        #    b. Heading Error (e_psi): Selisih orientasi robot dengan orientasi jalur
        path_yaw = math.atan2(y2 - y1, x2 - x1)
        e_psi = path_yaw - self.robot_yaw
        # Normalisasi sudut agar berada di antara -pi dan pi
        if e_psi > math.pi: e_psi -= 2 * math.pi
        if e_psi < -math.pi: e_psi += 2 * math.pi

        # 2. Definisikan Sliding Surface (sigma)
        #    Menggabungkan kedua error menjadi satu variabel.
        sigma = self.lambda_cte_ * e_cte + e_psi

        # 3. Hitung Control Law (omega)
        #    Menghasilkan kecepatan sudut (belok) untuk membuat sigma -> 0
        #    Ini adalah kombinasi kontrol proporsional dan SMC
        omega_p = self.lambda_psi_ * e_psi
        omega_smc = self.k_ * np.tanh(sigma / self.phi_)
        omega = omega_p + omega_smc

        # =================================================================

        # Publikasikan perintah kecepatan
        cmd_msg = Twist()
        cmd_msg.linear.x = self.v_
        cmd_msg.angular.z = omega
        self.cmd_vel_publisher.publish(cmd_msg)

    def stop_robot(self):
        """Mengirimkan perintah berhenti ke robot."""
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    smc_node = SMCController()
    rclpy.spin(smc_node)
    smc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# import math
# import numpy as np
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped

# def euler_from_quaternion(quaternion):
#     x = quaternion.x; y = quaternion.y; z = quaternion.z; w = quaternion.w
#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     return math.atan2(t3, t4)

# class SMCController(Node):
#     def __init__(self):
#         super().__init__('smc_controller_path_follower')

#         # --- Deklarasi Parameter ---
#         self.declare_parameter('lambda_psi', 1.5)
#         self.declare_parameter('lambda_cte', 0.5)
#         self.declare_parameter('k_val', 0.8)
#         self.declare_parameter('phi_val', 0.2)
#         self.declare_parameter('forward_velocity', 0.8)
#         self.declare_parameter('goal_tolerance', 0.3)

#         # --- Dapatkan nilai parameter ---
#         self.lambda_psi_ = self.get_parameter('lambda_psi').get_parameter_value().double_value
#         self.lambda_cte_ = self.get_parameter('lambda_cte').get_parameter_value().double_value
#         self.k_ = self.get_parameter('k_val').get_parameter_value().double_value
#         self.phi_ = self.get_parameter('phi_val').get_parameter_value().double_value
#         self.v_ = self.get_parameter('forward_velocity').get_parameter_value().double_value
#         self.goal_tol_ = self.get_parameter('goal_tolerance').get_parameter_value().double_value

#         # --- [PATH] Path lurus yang efisien hanya butuh titik akhir ---
#         self.path = [(10.0, 0.0)] 
#         self.waypoint_idx_ = 0
#         self.path_completed = False
#         self.start_waypoint_ = (0.0, 0.0)

#         # --- Inisialisasi State ---
#         self.robot_x = 0.0; self.robot_y = 0.0; self.robot_yaw = 0.0
#         self.odom_received = False

#         # --- Subscriber & Publisher ---
#         self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         # self.cmd_vel_publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_smc', 10)
#         self.timer = self.create_timer(0.02, self.control_loop)

#         # [BARU] Publisher untuk visualisasi jejak di RViz
#         self.path_publisher_ = self.create_publisher(Path, '/robot_path', 10)
#         self.robot_path_msg_ = Path() # Pesan untuk menyimpan jejak

#         self.get_logger().info('SMC Path Follower Node (CTE Enhanced) has been started.')

#     def odom_callback(self, msg):
#         self.robot_x = msg.pose.pose.position.x
#         self.robot_y = msg.pose.pose.position.y
#         self.robot_yaw = euler_from_quaternion(msg.pose.pose.orientation)
#         if not self.odom_received:
#             self.start_waypoint_ = (self.robot_x, self.robot_y)
#         self.odom_received = True

#         # [BARU] Logika untuk merekam dan mempublikasikan jejak
#         current_pose = PoseStamped()
#         current_pose.header = msg.header
#         current_pose.pose = msg.pose.pose
#         self.robot_path_msg_.poses.append(current_pose)
#         self.path_publisher_.publish(self.robot_path_msg_)

#     def control_loop(self):
#         if not self.odom_received or self.path_completed:
#             self.stop_robot(); return

#         target_waypoint = self.path[self.waypoint_idx_]
        
#         dist_to_goal = math.sqrt((target_waypoint[0] - self.robot_x)**2 + (target_waypoint[1] - self.robot_y)**2)
#         if dist_to_goal < self.goal_tol_:
#             if not self.path_completed: # Cek agar pesan tidak berulang
#                 self.get_logger().info('Goal Reached!')
#                 self.path_completed = True
#             return

#         # --- Perhitungan Error (Orientasi + Jarak ke Garis) ---
#         desired_yaw = math.atan2(target_waypoint[1] - self.robot_y, target_waypoint[0] - self.robot_x)
#         e_psi = desired_yaw - self.robot_yaw
#         if e_psi > math.pi: e_psi -= 2 * math.pi
#         if e_psi < -math.pi: e_psi += 2 * math.pi

#         y2 = target_waypoint[1]; y1 = self.start_waypoint_[1]
#         x2 = target_waypoint[0]; x1 = self.start_waypoint_[0]
#         e_cte = ((y2 - y1) * self.robot_x - (x2 - x1) * self.robot_y + x2 * y1 - y2 * x1) / (math.sqrt((y2-y1)**2 + (x2-x1)**2) + 1e-9)

#         sigma = self.lambda_cte_ * e_cte + e_psi
#         omega = self.lambda_psi_ * e_psi + self.k_ * np.tanh(sigma / self.phi_)

#         cmd_msg = Twist(); cmd_msg.linear.x = self.v_; cmd_msg.angular.z = omega
#         self.cmd_vel_publisher.publish(cmd_msg)

#     def stop_robot(self):
#         cmd_msg = Twist(); cmd_msg.linear.x = 0.0; cmd_msg.angular.z = 0.0
#         self.cmd_vel_publisher.publish(cmd_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     smc_node = SMCController()
#     rclpy.spin(smc_node)
#     smc_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()