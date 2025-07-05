#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import numpy as np

def euler_from_quaternion(x, y, z, w):
    """
    Converts quaternion (x, y, z, w) to euler roll, pitch, yaw
    """
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if sinp > +1:
        pitch = math.pi / 2
    elif sinp < -1:
        pitch = -math.pi / 2
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class SMCController(Node):
    def __init__(self):
        super().__init__('smc_path_follower')

        # --- Deklarasi Parameter Kontrol SMC dan Robot ---
        self.declare_parameter('lambda_cte', 1.0)
        self.declare_parameter('lambda_psi', 2.0)
        self.declare_parameter('k_gain', 1.5)
        self.declare_parameter('phi_boundary', 0.5)
        self.declare_parameter('forward_velocity', 0.8)
        self.declare_parameter('goal_tolerance', 0.2)

        # --- Ambil nilai parameter ---
        self.lambda_cte_ = self.get_parameter('lambda_cte').get_parameter_value().double_value
        self.lambda_psi_ = self.get_parameter('lambda_psi').get_parameter_value().double_value
        self.k_ = self.get_parameter('k_gain').get_parameter_value().double_value
        self.phi_ = self.get_parameter('phi_boundary').get_parameter_value().double_value
        self.v_ = self.get_parameter('forward_velocity').get_parameter_value().double_value
        self.goal_tol_ = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        # --- Definisikan Trajektori (Jalur Lurus) ---
        self.start_waypoint_ = (0.0, 0.0)
        self.goal_waypoint_ = (20.0, 0.0)
        self.path_completed = False

        # --- Inisialisasi State Robot dari Odometry ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.odom_received = False

        # --- Inisialisasi State Robot dari IMU ---
        self.robot_yaw = 0.0
        self.imu_received = False

        # --- Subscriber & Publisher ---
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('SMC Path Follower Node has been started.')
        self.get_logger().info('Subscribing to /odom and /imu/data topics.')

    def odom_callback(self, msg):
        """Callback untuk mengupdate posisi robot dari topic /odom."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        if not self.odom_received:
            self.start_waypoint_ = (self.robot_x, self.robot_y)
            self.odom_received = True

    def imu_callback(self, msg):
        """Callback untuk mengupdate data orientasi robot dari IMU."""
        _, _, yaw = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.robot_yaw = yaw
        self.imu_received = True

    def control_loop(self):
        """Loop utama yang menghitung dan mengirimkan perintah kontrol."""
        if not self.odom_received or not self.imu_received or self.path_completed:
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
        y2, x2 = self.goal_waypoint_
        y1, x1 = self.start_waypoint_
        e_cte = ((y2 - y1) * self.robot_x - (x2 - x1) * self.robot_y + x2 * y1 - y2 * x1) / (math.sqrt((y2-y1)**2 + (x2-x1)**2) + 1e-9)

        path_yaw = math.atan2(y2 - y1, x2 - x1)
        e_psi = path_yaw - self.robot_yaw
        if e_psi > math.pi:
            e_psi -= 2 * math.pi
        if e_psi < -math.pi:
            e_psi += 2 * math.pi

        # 2. Definisikan Sliding Surface (sigma)
        sigma = self.lambda_cte_ * e_cte + e_psi

        # 3. Hitung Control Law (omega)
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







# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu # <-- BARU: Impor tipe pesan Imu
# from geometry_msgs.msg import Twist
# import math
# import numpy as np

# def euler_from_quaternion(quaternion):
#     """
#     Converts quaternion (w in last place) to euler roll, pitch, yaw
#     quaternion = [w, x, y, z]
#     Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
#     """
#     x = quaternion.x
#     y = quaternion.y
#     z = quaternion.z
#     w = quaternion.w

#     sinr_cosp = 2 * (w * x + y * z)
#     cosr_cosp = 1 - 2 * (x * x + y * y)
#     roll = np.arctan2(sinr_cosp, cosr_cosp)

#     sinp = 2 * (w * y - z * x)
#     pitch = np.arcsin(sinp)

#     siny_cosp = 2 * (w * z + x * y)
#     cosy_cosp = 1 - 2 * (y * y + z * z)
#     yaw = np.arctan2(siny_cosp, cosy_cosp)

#     return roll, pitch, yaw

# class Quaternion:
#     w: float
#     x: float
#     y: float
#     z: float

# def quaternion_from_euler(roll, pitch, yaw):
#     """
#     Converts euler roll, pitch, yaw to quaternion
#     """
#     cy = math.cos(yaw * 0.5)
#     sy = math.sin(yaw * 0.5)
#     cp = math.cos(pitch * 0.5)
#     sp = math.sin(pitch * 0.5)
#     cr = math.cos(roll * 0.5)
#     sr = math.sin(roll * 0.5)

#     q = Quaternion()
#     q.w = cy * cp * cr + sy * sp * sr
#     q.x = cy * cp * sr - sy * sp * cr
#     q.y = sy * cp * sr + cy * sp * cr
#     q.z = sy * cp * cr - cy * sp * sr
#     return q 

# class SMCController(Node):
#     def __init__(self):
#         super().__init__('smc_path_follower')

#         # --- Deklarasi Parameter Kontrol SMC dan Robot ---
#         self.declare_parameter('lambda_cte', 1.0)
#         self.declare_parameter('lambda_psi', 2.0)
#         self.declare_parameter('k_gain', 1.5)
#         self.declare_parameter('phi_boundary', 0.5)
#         self.declare_parameter('forward_velocity', 0.8)
#         self.declare_parameter('goal_tolerance', 0.2)

#         # --- Ambil nilai parameter ---
#         self.lambda_cte_ = self.get_parameter('lambda_cte').get_parameter_value().double_value
#         self.lambda_psi_ = self.get_parameter('lambda_psi').get_parameter_value().double_value
#         self.k_ = self.get_parameter('k_gain').get_parameter_value().double_value
#         self.phi_ = self.get_parameter('phi_boundary').get_parameter_value().double_value
#         self.v_ = self.get_parameter('forward_velocity').get_parameter_value().double_value
#         self.goal_tol_ = self.get_parameter('goal_tolerance').get_parameter_value().double_value

#         # --- Definisikan Trajektori (Jalur Lurus) ---
#         self.start_waypoint_ = (0.0, 0.0)
#         self.goal_waypoint_ = (20.0, 0.0)
#         self.path_completed = False

#         # --- Inisialisasi State Robot dari Odometry ---
#         self.robot_x = 0.0
#         self.robot_y = 0.0
#         self.robot_yaw = 0.0
#         self.odom_received = False

#         # --- Inisialisasi State Robot dari IMU --- # <-- BARU
#         self.imu_orientation = None # Quaternion
#         self.imu_angular_velocity = None # Vector3
#         self.imu_linear_acceleration = None # Vector3
#         self.imu_received = False

#         # --- Subscriber & Publisher ---
#         self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.imu_subscriber = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10) # <-- BARU: Subscriber untuk IMU
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.timer = self.create_timer(0.02, self.control_loop)

#         self.get_logger().info('SMC Path Follower Node has been started.')
#         self.get_logger().info(f"Subscribing to /odom and /imu/data topics.")

#     def odom_callback(self, msg):
#         """Callback untuk mengupdate posisi dan orientasi robot dari topic /odom."""
#         self.robot_x = msg.pose.pose.position.x
#         self.robot_y = msg.pose.pose.position.y
#         # Yaw dari odometry tetap digunakan untuk perhitungan error
#         self.robot_yaw = euler_from_quaternion(msg.pose.pose.orientation)
        
#         if not self.odom_received:
#             self.start_waypoint_ = (self.robot_x, self.robot_y)
#             self.odom_received = True

#     # --- FUNGSI BARU UNTUK MEMBACA IMU --- #
#     def imu_callback(self, msg):
#         """Callback untuk mengupdate data dari topic /imu/data."""
#         self.imu_orientation = msg.orientation
#         self.imu_angular_velocity = msg.angular_velocity
#         self.imu_linear_acceleration = msg.linear_acceleration
#         self.imu_received = True

#         # Contoh cara mengakses dan menampilkan data (opsional)
#         # yaw_from_imu = euler_from_quaternion(self.imu_orientation)
#         # self.get_logger().info(f'IMU Yaw: {yaw_from_imu:.2f}, Angular Vel Z: {self.imu_angular_velocity.z:.2f}', throttle_duration_sec=1.0)
        
#     def control_loop(self):
#         """Loop utama yang menghitung dan mengirimkan perintah kontrol."""
#         if not self.odom_received or self.path_completed:
#             self.stop_robot()
#             return

#         # Cek apakah robot sudah mencapai tujuan
#         dist_to_goal = math.sqrt((self.goal_waypoint_[0] - self.robot_x)**2 + (self.goal_waypoint_[1] - self.robot_y)**2)
#         if dist_to_goal < self.goal_tol_:
#             if not self.path_completed:
#                 self.get_logger().info('Goal Reached!')
#                 self.path_completed = True
#             self.stop_robot()
#             return

#         # =================================================================
#         # === INTI PERHITUNGAN SLIDING MODE CONTROL (SMC) ===
#         # =================================================================

#         # 1. Hitung Error
#         y2, x2 = self.goal_waypoint_
#         y1, x1 = self.start_waypoint_
#         e_cte = ((y2 - y1) * self.robot_x - (x2 - x1) * self.robot_y + x2 * y1 - y2 * x1) / (math.sqrt((y2-y1)**2 + (x2-x1)**2) + 1e-9)

#         path_yaw = math.atan2(y2 - y1, x2 - x1)
#         e_psi = path_yaw - self.robot_yaw
#         if e_psi > math.pi: e_psi -= 2 * math.pi
#         if e_psi < -math.pi: e_psi += 2 * math.pi

#         # 2. Definisikan Sliding Surface (sigma)
#         sigma = self.lambda_cte_ * e_cte + e_psi

#         # 3. Hitung Control Law (omega)
#         omega_p = self.lambda_psi_ * e_psi
#         omega_smc = self.k_ * np.tanh(sigma / self.phi_)
#         omega = omega_p + omega_smc

#         # =================================================================

#         # Publikasikan perintah kecepatan
#         cmd_msg = Twist()
#         cmd_msg.linear.x = self.v_
#         cmd_msg.angular.z = omega
#         self.cmd_vel_publisher.publish(cmd_msg)

#     def stop_robot(self):
#         """Mengirimkan perintah berhenti ke robot."""
#         cmd_msg = Twist()
#         cmd_msg.linear.x = 0.0
#         cmd_msg.angular.z = 0.0
#         self.cmd_vel_publisher.publish(cmd_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     smc_node = SMCController()
#     rclpy.spin(smc_node)
#     smc_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

