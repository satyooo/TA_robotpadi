#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import numpy as np
from nav_msgs.msg import Path


def euler_from_quaternion(quaternion):
    # ... (fungsi konversi dari respons sebelumnya) ...
    x = quaternion.x; y = quaternion.y; z = quaternion.z; w = quaternion.w
    t3 = +2.0 * (w * z + x * y); t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class SmcControllerNode(Node):
    def __init__(self):
        super().__init__('smc_trajectory_tracker')
        
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        # --- Parameter Kontrol (BISA ANDA TUNE) ---
        # self.V_CONSTANT = 1.0  # Kecepatan linear konstan (m/s)
        # self.Ky = 2.0          # Gain untuk error 'y'
        # self.K_omega = 0.5     # Gain switching untuk 'omega'
        # self.PHI = 0.5         # Boundary layer untuk fungsi sat()

        self.V_CONSTANT = 1.0  # Kecepatan linear konstan (m/s)
        self.Ky = 4.0          # Gain untuk error 'y'
        self.K_omega = 1.0     # Gain switching untuk 'omega'
        self.PHI = 0.5         # Boundary layer untuk fungsi sat()

        # --- State & Target ---
        self.initial_x = 10.0
        self.initial_y = 0.0

        # Inisialisasi posisi saat ini dengan posisi awal
        self.current_x = self.initial_x
        self.current_y = self.initial_y
        self.current_theta = 2.0

        self.trajectory_received = False
        self.final_target_x = None

        # --- Subscribers & Publisher ---
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.path_sub = self.create_subscription(Path, '/line_trajectory', self.path_callback, 10)  # Path subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Loop Kontrol ---
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz
        self.get_logger().info('SMC Controller Node has been started.')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x # <<< UBAH INI
        self.current_y = msg.pose.pose.position.y # <<< UBAH INI

    def imu_callback(self, msg):
        self.current_theta = euler_from_quaternion(msg.orientation)

    def path_callback(self, msg):
        # <<< MODIFIKASI 2: Buka gerbang saat path diterima dan simpan titik akhir >>>
        if msg.poses and not self.trajectory_received:
            # Ambil koordinat x dari pose terakhir dalam path
            self.final_target_x = msg.poses[-1].pose.position.x
            self.trajectory_received = True # Buka gerbang!
            self.get_logger().info(f"Trajectory received with {len(msg.poses)} points. Final target x: {self.final_target_x}. Starting control.")

    # def path_callback(self, msg):
    #     # Ambil titik pertama dari path
    #     if msg.poses:
    #         self.target_x = msg.poses[0].pose.position.x
    #         self.target_y = msg.poses[0].pose.position.y
    #         self.get_logger().info(f"Path received: Target x={self.target_x}, y={self.target_y}")


    def sat_function(self, s, phi):
        if s > phi: return 1.0
        elif s < -phi: return -1.0
        else: return s / phi

    def control_loop(self):
        if not self.trajectory_received:
            return
    
        # Target untuk garis lurus y=0
        y_d = 0.0
        
        # 1. Hitung error y
        y_error = self.current_y - y_d
        
        # 2. Hitung theta_d (orientasi target dinamis)
        theta_d = -math.atan(self.Ky * y_error)
        
        # 3. Hitung sliding surface 's'
        s = self.current_theta - theta_d

        # 4. Hitung omega (ω) menggunakan hukum kontrol SMC
        # Komponen pertama (equivalent control)
        u_eq = -(self.Ky * self.V_CONSTANT * math.sin(self.current_theta)) / (1 + (self.Ky * y_error)**2)
        # Komponen kedua (switching control)
        u_sw = -self.K_omega * self.sat_function(s, self.PHI)
        
        omega = u_eq + u_sw
        
        # 5. Atur kecepatan linear dan angular
        v = self.V_CONSTANT
        # Hentikan robot jika sudah mencapai ujung lintasan (misal x > 9.8)
        if self.current_x > 49.8:
            v = 0.0
            omega = 0.0
            
        # Berhenti jika robot sudah mencapai atau sedikit melewati titik x terakhir
        if self.current_x >= self.final_target_x - 0.2: # Beri buffer kecil 20 cm
            v = 0.0
            omega = 0.0

        # 6. Publikasikan perintah
        twist_msg = Twist()
        twist_msg.linear.x = v
        twist_msg.angular.z = omega
        self.cmd_pub.publish(twist_msg)
        
        # Tambahkan self.current_x dan self.current_y ke dalam log
        self.get_logger().info(
            f'Pose: (x={self.current_x:.2f}, y={self.current_y:.2f}) | '
            f'y_err: {y_error:.2f}, s: {s:.2f}, ω: {omega:.2f}, v: {v:.2f}'
        )

    def stop_robot(self):
        self.get_logger().info('Sending stop command to robot.')
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)

def main(args=None):
    # --- INI ADALAH POLA MAIN YANG SUDAH BENAR DAN STANDAR ---
    rclpy.init(args=args)
    
    smc_controller_node = SmcControllerNode()
    
    try:
        rclpy.spin(smc_controller_node)
    except KeyboardInterrupt:
        # Ini akan dieksekusi saat Anda menekan Ctrl+C
        smc_controller_node.get_logger().info('Keyboard interrupt, stopping robot...')
    finally:
        # Pastikan robot berhenti dan node dihancurkan dengan benar
        smc_controller_node.stop_robot()
        smc_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# import math
# import numpy as np

# class SMCController(Node):
#     def __init__(self):
#         super().__init__('smc_trajectory_tracker')
        
#         # If you are using this parameter later in the code:
#         use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

#         # Parameters for the Sliding Mode Control
#         self.k1 = 1.0  # Control gain for position error (x)
#         self.k2 = 1.0  # Control gain for position error (y)
#         self.k3 = 1.0  # Control gain for orientation error (theta)
#         self.target_x = 15.0  # Target x position
#         self.target_y = 0.0  # Target y position
#         self.target_theta = 0.0  # Target orientation (heading)

#         # Robot's initial position and orientation
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0

#         # Subscribe to Odometry to get the robot's position
#         self.odom_sub = self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)

#         # Subscribe to IMU data to get robot's heading (yaw)
#         self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

#         # Publisher for robot's control commands (linear velocity and angular velocity)
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Timer to periodically compute control commands
#         self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz control loop
   
#     def odom_callback(self, msg: Odometry):
#         """Callback function for Odometry data to get the robot's position."""
#         self.x = msg.pose.pose.position.x  # Update x position from Odometry
#         self.y = msg.pose.pose.position.y  # Update y position from Odometry
#         self.get_logger().info(f"Position (x, y): ({self.x}, {self.y})")

#     def imu_callback(self, msg: Imu):
#         q = msg.orientation
#         _, _, yaw = self.euler_from_quaternion(q)
#         self.theta = math.atan2(math.sin(yaw), math.cos(yaw))  # normalisasi

#     def euler_from_quaternion(self, q):
#         x, y, z, w = q.x, q.y, q.z, q.w
#         sinr_cosp = 2 * (w * x + y * z)
#         cosr_cosp = 1 - 2 * (x * x + y * y)
#         roll = np.arctan2(sinr_cosp, cosr_cosp)

#         sinp = 2 * (w * y - z * x)
#         pitch = np.arcsin(sinp)

#         siny_cosp = 2 * (w * z + x * y)
#         cosy_cosp = 1 - 2 * (y * y + z * z)
#         yaw = np.arctan2(siny_cosp, cosy_cosp)

#         return roll, pitch, yaw

#     def control_loop(self):
#         # Hitung error posisi dan orientasi
#         dx = self.target_x - self.x
#         dy = self.target_y - self.y
#         distance_to_target = math.hypot(dx, dy)
#         desired_theta = math.atan2(dy, dx)
#         e_theta = desired_theta - self.theta
#         e_theta = math.atan2(math.sin(e_theta), math.cos(e_theta))  # normalisasi
#         # # Perhitungan jarak ke target (pindah ke awal fungsi)
#         # distance_to_target = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)

#         # Cek jika sudah dekat dengan target
#         if distance_to_target < 0.1:
#             control_input = Twist()
#             control_input.linear.x = 0.0
#             control_input.angular.z = 0.0
#             self.cmd_vel_pub.publish(control_input)
#             self.get_logger().info('Robot stopped.')
#             return  # Keluar dari fungsi agar tidak lanjut ke bawah
        
#         """Main control loop for the Sliding Mode Control."""
#         # Compute errors in position and orientation
#         e_x = self.target_x - self.x
#         e_y = self.target_y - self.y
#         e_theta = self.target_theta - self.theta

#         # Ensure the error is within a valid range (e.g., keep yaw errors within -pi to pi)
#         if e_theta > math.pi:
#             e_theta -= 2 * math.pi
#         elif e_theta < -math.pi:
#             e_theta += 2 * math.pi

#         # Compute the sliding mode control input (linear velocity and angular velocity)
#         s = self.k1 * e_x + self.k2 * e_y + self.k3 * e_theta

#         # Control law to suppress chattering using a saturation function
#         control_input = Twist()

#         # For straight-line movement, we primarily use the linear velocity
#         control_input.linear.x = 0.7  # Move forward at a constant speed (adjust as needed)
#         control_input.angular.z = self.k3 * e_theta  # Rotate to correct yaw error

#         # Apply the control input
#         self.cmd_pub.publish(control_input)
#         self.get_logger().info(f"Publishing to /cmd_vel: Linear Velocity: {control_input.linear.x}, Angular Velocity: {control_input.angular.z}")

#         # Calculate the distance to the target
#         distance_to_target = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)

#         # Print position, errors, yaw (heading), and distance to target
#         # self.get_logger().info(f"Position (x, y): ({self.x}, {self.y})")
#         # self.get_logger().info(f"IMU Yaw (theta): {self.theta}")
#         # self.get_logger().info(f"Orientation error (theta): {e_theta}")
#         # self.get_logger().info(f"Distance to target: {distance_to_target}")
#         self.get_logger().info(f"Posisi       : ({self.x:.2f}, {self.y:.2f})")
#         self.get_logger().info(f"Yaw (rad)    : {self.theta:.2f} rad")
#         self.get_logger().info(f"Yaw (deg)    : {math.degrees(self.theta):.2f}°")
#         self.get_logger().info(f"Jarak ke goal: {distance_to_target:.2f} m")
#         self.get_logger().info(f"Error theta  : {e_theta:.2f} rad / {math.degrees(e_theta):.2f}°")
#         self.get_logger().info(f"Control Commands: Linear Velocity: {control_input.linear.x}, Angular Velocity: {control_input.angular.z}")

#         # Check if the robot is close enough to the target, and stop it
#         if distance_to_target < 0.1:  # If the robot is within 0.1 meters of the target
#             self.stop_robot()  # Stop the robot
#             self.get_logger().info("Robot has reached the target and is stopping.")


#     def stop_robot(self):
#         """Stops the robot by publishing zero velocities to /cmd_vel."""
#         stop_cmd = Twist()
#         stop_cmd.linear.x = 0.0
#         stop_cmd.angular.z = 0.0
#         self.cmd_pub.publish(stop_cmd)
#         self.get_logger().info('Robot stopped.')

# def main(args=None):
#     rclpy.init(args=args)
    
#     # Initialize the SMC controller node
#     smc_tracker_node = SMCController()
    
#     try:
#         # Keep the node alive and handle callbacks
#         rclpy.spin(smc_tracker_node)
#     finally:
#         # Make sure the robot stops safely when the node is destroyed
#         smc_tracker_node.stop_robot()  # This calls the stop_robot() method to stop the robot
#         smc_tracker_node.destroy_node()
        
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()








# # sudah dengan perbaikan tapi masih error


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import Twist
# import numpy as np
# import math

# class SMCPathFollower(Node):
#     def __init__(self):
#         super().__init__('smc_path_follower')

#         self.get_logger().warn("====================== PERINGATAN ======================")
#         self.get_logger().warn("Node ini mengestimasi posisi HANYA dari integrasi IMU.")
#         self.get_logger().warn("Metode ini SANGAT RENTAN TERHADAP DRIFT dan tidak akurat.")
#         self.get_logger().warn("========================================================")

#         # --- Parameter untuk Tugas Akhir ---
#         self.declare_parameter('trajectory', [20.0, 0.0, 30.0, 0.0])  # Waypoint (20, 0) -> (30, 0)
#         self.declare_parameter('waypoint_tolerance', 0.05)  # Toleransi kecil untuk waypoint

#         # --- Parameter untuk Sliding Mode Control (SMC) ---
#         self.declare_parameter('k_angular', 1.5)  # Kecepatan kontrol orientasi robot
#         self.declare_parameter('lambda_angular', 2.0)  # Kecepatan konvergensi error sudut
#         self.declare_parameter('k_linear', 0.8)  # Kecepatan linier menuju waypoint
        
#         # --- Batas Kecepatan Robot ---
#         self.declare_parameter('max_linear_speed', 0.8)  # Maksimum kecepatan linier robot
#         self.declare_parameter('max_angular_speed', 0.6)  # Maksimum kecepatan angular robot

#         # --- Ambil nilai parameter dan proses trajektori ---
#         self.trajectory = self.parse_trajectory_param()
#         self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value

#         # --- Inisialisasi Publisher dan Subscriber ---
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.imu_sub = self.create_subscription(Imu, '/imu/data', self.control_loop_callback, 10)

#         # --- Variabel State Kontrol ---
#         self.current_waypoint_index = 0
#         self.is_trajectory_done = False if self.trajectory else True
#         self.last_time = None

#         # Variabel untuk menyimpan state robot (posisi, kecepatan, orientasi)
#         self.current_position = np.array([20.0, 0.0])  # [x, y]
#         self.current_velocity = np.array([0.0, 0.0])  # [vx, vy]
#         self.current_yaw = 0.0
#         self.prev_error_angular = 0.0

#         # Variabel untuk menyimpan kecepatan sebelumnya (untuk filter)
#         self.prev_linear_speed = 0.0
#         self.prev_angular_speed = 0.0

#         if self.is_trajectory_done:
#             self.get_logger().warn('Trajektori kosong! Node tidak akan melakukan apa-apa.')
#         else:
#             self.get_logger().info(f'SMC IMU-Only Tracker dimulai. Mengikuti {len(self.trajectory)} waypoints.')
#             self.get_logger().info(f'Waypoint pertama: {self.trajectory[0]}')

#     def parse_trajectory_param(self):
#         """Mengubah parameter list flat [x1,y1,x2,y2] menjadi list of tuples [(x1,y1), (x2,y2)]."""
#         flat_list = self.get_parameter('trajectory').get_parameter_value().double_array_value
#         if len(flat_list) % 2 != 0:
#             self.get_logger().error('Parameter trajektori harus memiliki jumlah elemen genap (pasangan x,y).')
#             return []
#         return [(flat_list[i], flat_list[i+1]) for i in range(0, len(flat_list), 2)]

#     def low_pass_filter(self, new_value, prev_value, alpha=0.1):
#         """Filter low-pass untuk mengurangi noise pada pembacaan sensor."""
#         return alpha * new_value + (1 - alpha) * prev_value

#     def control_loop_callback(self, msg: Imu):
#         """
#         Callback ini adalah satu-satunya pemicu kontrol.
#         1. Mengestimasi posisi & orientasi dari IMU.
#         2. Menjalankan logika kontrol SMC.
#         """
#         if self.is_trajectory_done:
#             return

#         current_time = self.get_clock().now()
#         if self.last_time is None:
#             self.last_time = current_time
#             return
        
#         dt = (current_time - self.last_time).nanoseconds / 1e9
#         self.last_time = current_time
#         if dt <= 0: return

#         # =========================================================================
#         # LANGKAH 1: ESTIMASI STATE ROBOT HANYA DARI IMU
#         # =========================================================================
        
#         # 1a. Dapatkan Orientasi (Yaw) menggunakan fungsi baru
#         _, _, new_yaw = self.euler_from_quaternion(msg.orientation)
#         self.current_yaw = self.low_pass_filter(new_yaw, self.current_yaw, alpha=0.1)

#         # **Tambahkan pengecekan jika robot terbalik**
#         if abs(self.current_yaw) > math.pi / 2:  # Jika yaw lebih dari 90 derajat, robot mungkin terbalik
#             self.get_logger().warn("Robot terbalik, menghentikan estimasi posisi.")
#             self.stop_robot()  # Robot berhenti jika terbalik
#             return

#         # 1b. Dapatkan Percepatan Linear dari IMU (dalam frame robot)
#         accel_robot_frame = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y])

#         # 1c. Putar vektor percepatan ke frame global/dunia (World Frame)
#         cos_yaw = math.cos(self.current_yaw)
#         sin_yaw = math.sin(self.current_yaw)
#         accel_world_frame = np.array([
#             accel_robot_frame[0] * cos_yaw - accel_robot_frame[1] * sin_yaw,
#             accel_robot_frame[0] * sin_yaw + accel_robot_frame[1] * cos_yaw
#         ])

#         # Batasi percepatan agar tidak terlalu besar (mengurangi noise)
#         max_accel = 1.0  # Maksimum percepatan dalam m/s^2
#         accel_world_frame = np.clip(accel_world_frame, -max_accel, max_accel)

#         # 1d. Integrasi pertama: Percepatan -> Kecepatan
#         self.current_velocity += accel_world_frame * dt

#         # Batasi kecepatan agar tidak terlalu tinggi
#         max_velocity = 1.5  # Maksimum kecepatan robot dalam m/s
#         self.current_velocity = np.clip(self.current_velocity, -max_velocity, max_velocity)

#         # 1e. Integrasi kedua: Kecepatan -> Posisi
#         self.current_position += self.current_velocity * dt
#         self.current_position = np.clip(self.current_position, -1000, 1000)  # Batasi perubahan posisi

#         # Tambahkan pembatasan perubahan posisi untuk stabilitas
#         max_position_change = 0.5  # Maksimum perubahan posisi dalam satu langkah waktu
#         self.current_position = np.clip(self.current_position, 
#                                         self.current_position - max_position_change, 
#                                         self.current_position + max_position_change)

#         current_x = self.current_position[0]
#         current_y = self.current_position[1]

#         # =========================================================================
#         # LANGKAH 2: LOGIKA KONTROL SMC 
#         # =========================================================================
        
#         target_x, target_y = self.trajectory[self.current_waypoint_index]

#         # Hitung error jarak dan error sudut
#         error_distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
#         angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
#         error_angular = self.normalize_angle(angle_to_target - self.current_yaw)

#         # Periksa apakah robot telah mencapai waypoint
#         if error_distance < self.waypoint_tolerance:
#             self.current_waypoint_index += 1
#             self.prev_error_angular = 0.0
            
#             if self.current_waypoint_index >= len(self.trajectory):
#                 self.is_trajectory_done = True
#                 self.stop_robot()
#                 self.get_logger().info('===== Trajektori Selesai! =====')
#             else:
#                 next_target = self.trajectory[self.current_waypoint_index]
#                 self.get_logger().info(f'Waypoint tercapai! Bergerak ke waypoint berikutnya: {next_target}')
#             return

#         # --- Sliding Mode Control ---
#         k_ang = self.get_parameter('k_angular').get_parameter_value().double_value
#         lambda_ang = self.get_parameter('lambda_angular').get_parameter_value().double_value

#         error_angular_dot = (error_angular - self.prev_error_angular) / dt
#         self.prev_error_angular = error_angular

#         sliding_surface = error_angular_dot + lambda_ang * error_angular
#         angular_speed = k_ang * np.tanh(sliding_surface)

#         k_lin = self.get_parameter('k_linear').get_parameter_value().double_value
#         linear_speed = k_lin * error_distance
#         linear_speed *= math.exp(-lambda_ang * abs(error_angular))

#         # Batasi kecepatan agar tidak melebihi batas maksimum
#         max_lin = self.get_parameter('max_linear_speed').get_parameter_value().double_value
#         max_ang = self.get_parameter('max_angular_speed').get_parameter_value().double_value
#         linear_speed = np.clip(linear_speed, 0, max_lin)
#         angular_speed = np.clip(angular_speed, -max_ang, max_ang)

#         # Haluskan perubahan kecepatan dengan low-pass filter
#         linear_speed = self.low_pass_filter(linear_speed, self.prev_linear_speed)
#         angular_speed = self.low_pass_filter(angular_speed, self.prev_angular_speed)
        
#         # Simpan kecepatan terbaru
#         self.prev_linear_speed = linear_speed
#         self.prev_angular_speed = angular_speed

#         # Kirimkan perintah ke robot
#         twist_msg = Twist()
#         twist_msg.linear.x = linear_speed
#         twist_msg.angular.z = angular_speed
#         self.cmd_vel_pub.publish(twist_msg)

#         self.get_logger().info(f'Pos (est):({current_x:.2f},{current_y:.2f}) | WP:{self.current_waypoint_index} | Dist:{error_distance:.2f} | Ang:{error_angular:.2f}')



#     def stop_robot(self):
#         """Mengirim perintah berhenti total ke robot."""
#         twist_msg = Twist()
#         twist_msg.linear.x = 0.0
#         twist_msg.angular.z = 0.0
#         self.cmd_vel_pub.publish(twist_msg)

#     def euler_from_quaternion(self, quaternion):
#         """
#         Converts quaternion (w in last place) to euler roll, pitch, yaw
#         quaternion = [x, y, z, w]
#         """
#         x = quaternion.x
#         y = quaternion.y
#         z = quaternion.z
#         w = quaternion.w

#         sinr_cosp = 2 * (w * x + y * z)
#         cosr_cosp = 1 - 2 * (x * x + y * y)
#         roll = np.arctan2(sinr_cosp, cosr_cosp)

#         sinp = 2 * (w * y - z * x)
#         pitch = np.arcsin(sinp)
#         if np.abs(sinp) >= 1:
#             pitch = np.sign(sinp) * np.pi / 2.0

#         siny_cosp = 2 * (w * z + x * y)
#         cosy_cosp = 1 - 2 * (y * y + z * z)
#         yaw = np.arctan2(siny_cosp, cosy_cosp)

#         return roll, pitch, yaw

#     def normalize_angle(self, angle):
#         """Menormalkan sudut agar berada di antara -pi dan pi."""
#         while angle > math.pi: angle -= 2.0 * math.pi
#         while angle < -math.pi: angle += 2.0 * math.pi
#         return angle

# def main(args=None):
#     rclpy.init(args=args)
#     smc_tracker_node = SMCPathFollower()
#     try:
#         rclpy.spin(smc_tracker_node)
#     except KeyboardInterrupt:
#         smc_tracker_node.get_logger().info('Berhenti karena interupsi keyboard.')
#     finally:
#         smc_tracker_node.stop_robot()
#         smc_tracker_node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# sebelum perubahan

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import Twist
# import numpy as np
# import math

# class SMCPathFollower(Node):
#     def __init__(self):
#         super().__init__('smc_path_follower')
        
#         self.get_logger().warn("====================== PERINGATAN ======================")
#         self.get_logger().warn("Node ini mengestimasi posisi HANYA dari integrasi IMU.")
#         self.get_logger().warn("Metode ini SANGAT RENTAN TERHADAP DRIFT dan tidak akurat.")
#         self.get_logger().warn("========================================================")

#         # --- Parameter untuk Tugas Akhir (Bisa diubah dari command line) ---
#         self.declare_parameter('trajectory', [25.0, 0.0])
#         self.declare_parameter('waypoint_tolerance', 0.1) # Toleransi sedikit lebih besar karena drift

#         # --- Parameter untuk Sliding Mode Control (SMC) ---
#         self.declare_parameter('k_angular', 2.0)
#         self.declare_parameter('lambda_angular', 3.0)
#         self.declare_parameter('k_linear', 0.7)
        
#         # --- Batas Kecepatan Robot ---
#         self.declare_parameter('max_linear_speed', 2.0)
#         self.declare_parameter('max_angular_speed', 1.0)

#         # --- Ambil nilai parameter dan proses trajektori ---
#         self.trajectory = self.parse_trajectory_param()
#         self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        
#         # --- Inisialisasi Publisher dan Subscriber ---
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         # SATU-SATUNYA SUBSCRIBER: /imu/data
#         self.imu_sub = self.create_subscription(Imu, '/imu/data', self.control_loop_callback, 10)
        
#         # --- Variabel State Kontrol ---
#         self.current_waypoint_index = 0
#         self.is_trajectory_done = False if self.trajectory else True
#         self.last_time = None
        
#         # Variabel untuk menyimpan state robot (posisi, kecepatan, orientasi)
#         self.current_position = np.array([20.0, 0.0]) # [x, y]
#         self.current_velocity = np.array([0.0, 0.0]) # [vx, vy]
#         self.current_yaw = 0.0
#         self.prev_error_angular = 0.0

#         if self.is_trajectory_done:
#             self.get_logger().warn('Trajektori kosong! Node tidak akan melakukan apa-apa.')
#         else:
#             self.get_logger().info(f'SMC IMU-Only Tracker dimulai. Mengikuti {len(self.trajectory)} waypoints.')
#             self.get_logger().info(f'Waypoint pertama: {self.trajectory[0]}')

#     def parse_trajectory_param(self):
#         """Mengubah parameter list flat [x1,y1,x2,y2] menjadi list of tuples [(x1,y1), (x2,y2)]."""
#         flat_list = self.get_parameter('trajectory').get_parameter_value().double_array_value
#         if len(flat_list) % 2 != 0:
#             self.get_logger().error('Parameter trajektori harus memiliki jumlah elemen genap (pasangan x,y).')
#             return []
#         return [(flat_list[i], flat_list[i+1]) for i in range(0, len(flat_list), 2)]

#     def control_loop_callback(self, msg: Imu):
#         """
#         Callback ini adalah satu-satunya pemicu kontrol.
#         1. Mengestimasi posisi & orientasi dari IMU.
#         2. Menjalankan logika kontrol SMC.
#         """
#         if self.is_trajectory_done:
#             return

#         current_time = self.get_clock().now()
#         if self.last_time is None:
#             self.last_time = current_time
#             return
        
#         dt = (current_time - self.last_time).nanoseconds / 1e9
#         self.last_time = current_time
#         if dt <= 0: return

#         # =========================================================================
#         # LANGKAH 1: ESTIMASI STATE ROBOT HANYA DARI IMU
#         # =========================================================================
        
#         # 1a. Dapatkan Orientasi (Yaw) menggunakan fungsi baru
#         # Kita hanya butuh yaw, jadi roll dan pitch diabaikan dengan '_'
#         _, _, self.current_yaw = self.euler_from_quaternion(msg.orientation)

#         # 1b. Dapatkan Percepatan Linear dari IMU (dalam frame robot)
#         accel_robot_frame = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y])

#         # 1c. Putar vektor percepatan ke frame global/dunia (World Frame)
#         cos_yaw = math.cos(self.current_yaw)
#         sin_yaw = math.sin(self.current_yaw)
#         accel_world_frame = np.array([
#             accel_robot_frame[0] * cos_yaw - accel_robot_frame[1] * sin_yaw,
#             accel_robot_frame[0] * sin_yaw + accel_robot_frame[1] * cos_yaw
#         ])

#         # 1d. Integrasi pertama: Percepatan -> Kecepatan
#         self.current_velocity += accel_world_frame * dt

#         # 1e. Integrasi kedua: Kecepatan -> Posisi
#         self.current_position += self.current_velocity * dt
        
#         current_x = self.current_position[0]
#         current_y = self.current_position[1]

#         # =========================================================================
#         # LANGKAH 2: LOGIKA KONTROL SMC 
#         # =========================================================================
        
#         target_x, target_y = self.trajectory[self.current_waypoint_index]

#         error_distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
#         angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
#         error_angular = self.normalize_angle(angle_to_target - self.current_yaw)

#         if error_distance < self.waypoint_tolerance:
#             self.current_waypoint_index += 1
#             self.prev_error_angular = 0.0
            
#             if self.current_waypoint_index >= len(self.trajectory):
#                 self.is_trajectory_done = True
#                 self.stop_robot()
#                 self.get_logger().info('===== Trajektori Selesai! =====')
#             else:
#                 next_target = self.trajectory[self.current_waypoint_index]
#                 self.get_logger().info(f'Waypoint tercapai! Bergerak ke waypoint berikutnya: {next_target}')
#             return

#         k_ang = self.get_parameter('k_angular').get_parameter_value().double_value
#         lambda_ang = self.get_parameter('lambda_angular').get_parameter_value().double_value

#         error_angular_dot = (error_angular - self.prev_error_angular) / dt
#         self.prev_error_angular = error_angular

#         sliding_surface = error_angular_dot + lambda_ang * error_angular
#         angular_speed = k_ang * np.tanh(sliding_surface)

#         k_lin = self.get_parameter('k_linear').get_parameter_value().double_value
#         linear_speed = k_lin * error_distance
#         linear_speed *= math.exp(-lambda_ang * abs(error_angular))

#         max_lin = self.get_parameter('max_linear_speed').get_parameter_value().double_value
#         max_ang = self.get_parameter('max_angular_speed').get_parameter_value().double_value
#         linear_speed = np.clip(linear_speed, 0, max_lin)
#         angular_speed = np.clip(angular_speed, -max_ang, max_ang)

#         twist_msg = Twist()
#         twist_msg.linear.x = linear_speed
#         twist_msg.angular.z = angular_speed
#         self.cmd_vel_pub.publish(twist_msg)
        
#         self.get_logger().info(f'Pos (est):({current_x:.2f},{current_y:.2f}) | WP:{self.current_waypoint_index} | Dist:{error_distance:.2f} | Ang:{error_angular:.2f}')

#     def stop_robot(self):
#         """Mengirim perintah berhenti total ke robot."""
#         twist_msg = Twist()
#         twist_msg.linear.x = 0.0
#         twist_msg.angular.z = 0.0
#         self.cmd_vel_pub.publish(twist_msg)

#     def euler_from_quaternion(self, quaternion):
#         """
#         Converts quaternion (w in last place) to euler roll, pitch, yaw
#         quaternion = [x, y, z, w]
#         Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
#         """
#         x = quaternion.x
#         y = quaternion.y
#         z = quaternion.z
#         w = quaternion.w

#         sinr_cosp = 2 * (w * x + y * z)
#         cosr_cosp = 1 - 2 * (x * x + y * y)
#         roll = np.arctan2(sinr_cosp, cosr_cosp)

#         sinp = 2 * (w * y - z * x)
#         pitch = np.arcsin(sinp)
#         if np.abs(sinp) >= 1:
#             pitch = np.sign(sinp) * np.pi / 2.0 # use 90 degrees if out of range

#         siny_cosp = 2 * (w * z + x * y)
#         cosy_cosp = 1 - 2 * (y * y + z * z)
#         yaw = np.arctan2(siny_cosp, cosy_cosp)

#         return roll, pitch, yaw

#     def normalize_angle(self, angle):
#         """Menormalkan sudut agar berada di antara -pi dan pi."""
#         while angle > math.pi: angle -= 2.0 * math.pi
#         while angle < -math.pi: angle += 2.0 * math.pi
#         return angle

# def main(args=None):
#     rclpy.init(args=args)
#     smc_tracker_node = SMCPathFollower()
#     try:
#         rclpy.spin(smc_tracker_node)
#     except KeyboardInterrupt:
#         smc_tracker_node.get_logger().info('Berhenti karena interupsi keyboard.')
#     finally:
#         smc_tracker_node.stop_robot()
#         smc_tracker_node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import Twist
# import numpy as np
# import math
# from collections import deque

# class SMCPathFollower(Node):
#     def __init__(self):
#         super().__init__('smc_path_follower')
        
#         self.get_logger().warn("====================== PERINGATAN ======================")
#         self.get_logger().warn("Node ini mengestimasi posisi dari IMU dengan filter.")
#         self.get_logger().warn("Akurasi terbatas, gunakan untuk eksperimental/akademis.")
#         self.get_logger().warn("========================================================")

#         # --- Parameter untuk Tugas Akhir ---
#         self.declare_parameter('trajectory', [5.0, 0.0, 10.0, 0.0, 10.0, 5.0, 0.0, 5.0, 0.0, 0.0])
#         self.declare_parameter('waypoint_tolerance', 0.5)

#         # --- Parameter untuk Sliding Mode Control (SMC) ---
#         self.declare_parameter('k_angular', 2.0)
#         self.declare_parameter('lambda_angular', 3.0)
#         self.declare_parameter('k_linear', 0.5)
        
#         # --- Batas Kecepatan Robot ---
#         self.declare_parameter('max_linear_speed', 0.8)
#         self.declare_parameter('max_angular_speed', 1.0)
        
#         # --- Parameter Filter IMU ---
#         self.declare_parameter('accel_threshold', 0.1)  # Threshold untuk mendeteksi gerakan
#         self.declare_parameter('velocity_decay', 0.95)  # Faktor peluruhan kecepatan
#         self.declare_parameter('position_correction', 0.98)  # Koreksi posisi

#         # --- Ambil nilai parameter ---
#         self.trajectory = self.parse_trajectory_param()
#         self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        
#         # --- Inisialisasi Publisher dan Subscriber ---
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.imu_sub = self.create_subscription(Imu, '/imu/data', self.control_loop_callback, 10)
        
#         # --- Variabel State Kontrol ---
#         self.current_waypoint_index = 0
#         self.is_trajectory_done = False if self.trajectory else True
#         self.last_time = None
        
#         # --- Variabel untuk IMU-based positioning ---
#         self.current_position = np.array([0.0, 0.0])
#         self.current_velocity = np.array([0.0, 0.0])
#         self.current_yaw = 0.0
#         self.prev_error_angular = 0.0
        
#         # --- Filter dan Kompensasi IMU ---
#         self.gravity_compensation = np.array([0.0, 0.0, -9.81])  # Kompensasi gravitasi
#         self.accel_bias = np.array([0.0, 0.0, 0.0])  # Bias accelerometer
#         self.accel_buffer = deque(maxlen=10)  # Buffer untuk moving average
#         self.velocity_buffer = deque(maxlen=5)
        
#         # --- Kalibrasi IMU ---
#         self.calibration_samples = 100
#         self.calibration_count = 0
#         self.is_calibrated = False
#         self.accel_sum = np.array([20.0, 0.0, 2.0])

#         if self.is_trajectory_done:
#             self.get_logger().warn('Trajektori kosong! Node tidak akan melakukan apa-apa.')
#         else:
#             self.get_logger().info(f'SMC IMU Tracker dimulai dengan {len(self.trajectory)} waypoints.')
#             self.get_logger().info(f'Waypoint pertama: {self.trajectory[0]}')
#             self.get_logger().info('Melakukan kalibrasi IMU...')

#     def parse_trajectory_param(self):
#         """Mengubah parameter list flat [x1,y1,x2,y2] menjadi list of tuples [(x1,y1), (x2,y2)]."""
#         flat_list = self.get_parameter('trajectory').get_parameter_value().double_array_value
#         if len(flat_list) % 2 != 0:
#             self.get_logger().error('Parameter trajektori harus memiliki jumlah elemen genap (pasangan x,y).')
#             return []
#         return [(flat_list[i], flat_list[i+1]) for i in range(0, len(flat_list), 2)]

#     def calibrate_imu(self, msg: Imu):
#         """Kalibrasi bias IMU saat robot diam"""
#         if self.calibration_count < self.calibration_samples:
#             accel = np.array([
#                 msg.linear_acceleration.x,
#                 msg.linear_acceleration.y,
#                 msg.linear_acceleration.z
#             ])
#             self.accel_sum += accel
#             self.calibration_count += 1
            
#             if self.calibration_count % 20 == 0:
#                 self.get_logger().info(f'Kalibrasi IMU: {self.calibration_count}/{self.calibration_samples}')
#         else:
#             if not self.is_calibrated:
#                 self.accel_bias = self.accel_sum / self.calibration_samples
#                 # Kompensasi gravitasi pada sumbu Z
#                 self.accel_bias[2] -= 9.81
#                 self.is_calibrated = True
#                 self.get_logger().info(f'Kalibrasi selesai. Bias: {self.accel_bias}')

#     def filter_acceleration(self, raw_accel):
#         """Filter accelerometer dengan moving average dan threshold"""
#         # Kompensasi bias
#         compensated_accel = raw_accel - self.accel_bias
        
#         # Moving average filter
#         self.accel_buffer.append(compensated_accel)
#         if len(self.accel_buffer) < 3:
#             return np.array([0.0, 0.0, 0.0])
        
#         filtered_accel = np.mean(self.accel_buffer, axis=0)
        
#         # Threshold untuk mengurangi noise saat robot diam
#         threshold = self.get_parameter('accel_threshold').get_parameter_value().double_value
#         for i in range(3):
#             if abs(filtered_accel[i]) < threshold:
#                 filtered_accel[i] = 0.0
        
#         return filtered_accel

#     def estimate_position_from_imu(self, msg: Imu, dt):
#         """Estimasi posisi dari IMU dengan filter dan kompensasi"""
#         # 1. Dapatkan orientasi (yaw)
#         _, _, self.current_yaw = self.euler_from_quaternion(msg.orientation)
        
#         # 2. Dapatkan accelerasi dan filter
#         raw_accel = np.array([
#             msg.linear_acceleration.x,
#             msg.linear_acceleration.y,
#             msg.linear_acceleration.z
#         ])
        
#         filtered_accel = self.filter_acceleration(raw_accel)
        
#         # 3. Transformasi ke world frame (hanya x, y)
#         accel_robot_frame = filtered_accel[:2]  # Hanya x, y
#         cos_yaw = math.cos(self.current_yaw)
#         sin_yaw = math.sin(self.current_yaw)
        
#         accel_world_frame = np.array([
#             accel_robot_frame[0] * cos_yaw - accel_robot_frame[1] * sin_yaw,
#             accel_robot_frame[0] * sin_yaw + accel_robot_frame[1] * cos_yaw
#         ])
        
#         # 4. Integrasi dengan decay factor untuk mengurangi drift
#         velocity_decay = self.get_parameter('velocity_decay').get_parameter_value().double_value
        
#         # Update velocity dengan decay
#         self.current_velocity = self.current_velocity * velocity_decay + accel_world_frame * dt
        
#         # 5. Update posisi
#         self.current_position += self.current_velocity * dt
        
#         # 6. Koreksi posisi untuk mengurangi drift jangka panjang
#         position_correction = self.get_parameter('position_correction').get_parameter_value().double_value
#         self.current_position *= position_correction

#     def control_loop_callback(self, msg: Imu):
#         """Callback utama untuk kontrol"""
#         if self.is_trajectory_done:
#             return

#         # Kalibrasi IMU terlebih dahulu
#         if not self.is_calibrated:
#             self.calibrate_imu(msg)
#             return

#         current_time = self.get_clock().now()
#         if self.last_time is None:
#             self.last_time = current_time
#             return
        
#         dt = (current_time - self.last_time).nanoseconds / 1e9
#         self.last_time = current_time
#         if dt <= 0 or dt > 0.1:  # Skip jika dt terlalu besar
#             return

#         # Estimasi posisi dari IMU
#         self.estimate_position_from_imu(msg, dt)
        
#         current_x = self.current_position[0]
#         current_y = self.current_position[1]

#         # Logika kontrol SMC
#         target_x, target_y = self.trajectory[self.current_waypoint_index]
        
#         error_distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
#         angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
#         error_angular = self.normalize_angle(angle_to_target - self.current_yaw)

#         # Cek waypoint tercapai
#         if error_distance < self.waypoint_tolerance:
#             self.current_waypoint_index += 1
#             self.prev_error_angular = 0.0
            
#             if self.current_waypoint_index >= len(self.trajectory):
#                 self.is_trajectory_done = True
#                 self.stop_robot()
#                 self.get_logger().info('===== Trajektori Selesai! =====')
#                 return
#             else:
#                 next_target = self.trajectory[self.current_waypoint_index]
#                 self.get_logger().info(f'Waypoint {self.current_waypoint_index-1} tercapai! Menuju: {next_target}')

#         # SMC Control
#         k_ang = self.get_parameter('k_angular').get_parameter_value().double_value
#         lambda_ang = self.get_parameter('lambda_angular').get_parameter_value().double_value
#         k_lin = self.get_parameter('k_linear').get_parameter_value().double_value

#         # Angular control
#         error_angular_dot = (error_angular - self.prev_error_angular) / dt
#         self.prev_error_angular = error_angular
        
#         sliding_surface = error_angular_dot + lambda_ang * error_angular
#         angular_speed = k_ang * np.tanh(sliding_surface)

#         # Linear control dengan pengurangan saat error angular besar
#         linear_speed = k_lin * error_distance
#         linear_speed *= math.exp(-2.0 * abs(error_angular))  # Kurangi kecepatan saat belok

#         # Batasi kecepatan
#         max_lin = self.get_parameter('max_linear_speed').get_parameter_value().double_value
#         max_ang = self.get_parameter('max_angular_speed').get_parameter_value().double_value
#         linear_speed = np.clip(linear_speed, 0, max_lin)
#         angular_speed = np.clip(angular_speed, -max_ang, max_ang)

#         # Kirim perintah
#         twist_msg = Twist()
#         twist_msg.linear.x = linear_speed
#         twist_msg.angular.z = angular_speed
#         self.cmd_vel_pub.publish(twist_msg)
        
#         # Log setiap 10 callback untuk mengurangi spam
#         if self.calibration_count % 10 == 0:
#             self.get_logger().info(
#                 f'Pos:({current_x:.2f},{current_y:.2f}) | '
#                 f'Target:({target_x:.2f},{target_y:.2f}) | '
#                 f'WP:{self.current_waypoint_index}/{len(self.trajectory)} | '
#                 f'Dist:{error_distance:.2f} | '
#                 f'Yaw:{math.degrees(self.current_yaw):.1f}° | '
#                 f'AngErr:{math.degrees(error_angular):.1f}°'
#             )

#     def stop_robot(self):
#         """Mengirim perintah berhenti total ke robot."""
#         twist_msg = Twist()
#         twist_msg.linear.x = 0.0
#         twist_msg.angular.z = 0.0
#         self.cmd_vel_pub.publish(twist_msg)

#     def euler_from_quaternion(self, quaternion):
#         """Converts quaternion to euler roll, pitch, yaw"""
#         x = quaternion.x
#         y = quaternion.y
#         z = quaternion.z
#         w = quaternion.w

#         sinr_cosp = 2 * (w * x + y * z)
#         cosr_cosp = 1 - 2 * (x * x + y * y)
#         roll = np.arctan2(sinr_cosp, cosr_cosp)

#         sinp = 2 * (w * y - z * x)
#         pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
#         yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))



# # ini pake imu dan odom tapi ga jalan


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# import numpy as np
# import math

# class SMCPathFollower(Node):
#     def __init__(self):
#         super().__init__('smc_path_follower')

#         self.waiting_for_data_warning_sent = False
#         self.get_logger().info("====================== SMC CONTROLLER ======================")
#         self.get_logger().info("Menggunakan IMU untuk orientasi (yaw) dan Odometry untuk posisi")
#         self.get_logger().info("===========================================================")

#         # --- Parameter untuk Tugas Akhir ---
#         self.declare_parameter('trajectory', [30.0, 0.0])
#         self.declare_parameter('waypoint_tolerance', 0.1)

#         # --- Parameter untuk Sliding Mode Control (SMC) ---
#         self.declare_parameter('k_angular', 2.0)
#         self.declare_parameter('lambda_angular', 3.0)
#         self.declare_parameter('k_linear', 0.7)
        
#         # --- Batas Kecepatan Robot ---
#         self.declare_parameter('max_linear_speed', 0.5)
#         self.declare_parameter('max_angular_speed', 1.0)

#         # --- Ambil nilai parameter dan proses trajektori ---
#         self.trajectory = self.parse_trajectory_param()
#         self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        
#         # --- Inisialisasi Publisher dan Subscriber ---
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
#         # Subscriber untuk IMU (orientasi/yaw)
#         self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
#         # Subscriber untuk Odometry (posisi)
#         self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
#         # --- Variabel State Robot ---
#         self.current_position_x = 0.0  # Dari Odometry
#         self.current_position_y = 0.0  # Dari Odometry
#         self.current_yaw = 0.0         # Dari IMU
#         self.position_updated = False  # Flag untuk sinkronisasi data
#         self.orientation_updated = False
        
#         # --- Variabel State Kontrol ---
#         self.current_waypoint_index = 0
#         self.is_trajectory_done = False if self.trajectory else True
#         self.prev_error_angular = 0.0
#         self.last_time = None

#         # Timer untuk kontrol loop (menjalankan kontrol pada frekuensi tetap)
#         self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

#         if self.is_trajectory_done:
#             self.get_logger().warn('Trajektori kosong! Node tidak akan melakukan apa-apa.')
#         else:
#             self.get_logger().info(f'SMC Controller dimulai dengan {len(self.trajectory)} waypoints.')
#             self.get_logger().info(f'Waypoint pertama: {self.trajectory[0]}')

#     def parse_trajectory_param(self):
#         """Mengubah parameter list flat [x1,y1,x2,y2] menjadi list of tuples [(x1,y1), (x2,y2)]."""
#         flat_list = self.get_parameter('trajectory').get_parameter_value().double_array_value
#         if len(flat_list) % 2 != 0:
#             self.get_logger().error('Parameter trajektori harus memiliki jumlah elemen genap (pasangan x,y).')
#             return []
#         return [(flat_list[i], flat_list[i+1]) for i in range(0, len(flat_list), 2)]

#     def imu_callback(self, msg: Imu):
#         """
#         Callback untuk data IMU - hanya mengambil orientasi (yaw)
#         """
#         # Konversi quaternion ke euler untuk mendapatkan yaw
#         _, _, self.current_yaw = self.euler_from_quaternion(msg.orientation)
#         self.orientation_updated = True
        
#         # Debug log (bisa dimatikan untuk performa)
#         # self.get_logger().debug(f'IMU Yaw: {math.degrees(self.current_yaw):.1f}°')

#     def odom_callback(self, msg: Odometry):
#         """
#         Callback untuk data Odometry - hanya mengambil posisi (x, y)
#         """
#         self.current_position_x = msg.pose.pose.position.x
#         self.current_position_y = msg.pose.pose.position.y
#         self.position_updated = True
        
#         # Debug log (bisa dimatikan untuk performa)
#         # self.get_logger().debug(f'Odom Pos: ({self.current_position_x:.2f}, {self.current_position_y:.2f})')

#     def control_loop(self):
#         """
#         Loop kontrol utama yang berjalan pada timer
#         Menggabungkan data dari IMU (yaw) dan Odometry (posisi)
#         """
#         if self.is_trajectory_done:
#             return
            
#         # Pastikan kita sudah menerima data dari kedua sensor
#         if not (self.position_updated and self.orientation_updated):
#             self.get_logger().warn('Menunggu data dari IMU dan Odometry...')
#             self.waiting_for_data_warning_sent = True
#             return

#         current_time = self.get_clock().now()
#         if self.last_time is None:
#             self.last_time = current_time
#             return
        
#         dt = (current_time - self.last_time).nanoseconds / 1e9
#         self.last_time = current_time
#         if dt <= 0: 
#             return

#         if not (self.position_updated and self.orientation_updated):
#             if not self.waiting_for_data_warning_sent:
#                 self.get_logger().warn('Menunggu data dari IMU dan Odometry...')
#                 self.waiting_for_data_warning_sent = True # Set flag agar pesan tidak muncul lagi
#             return
#         # =========================================================================
#         # LANGKAH 1: AMBIL STATE ROBOT
#         # Posisi dari Odometry, Orientasi dari IMU
#         # =========================================================================
        
#         current_x = self.current_position_x  # Dari Odometry
#         current_y = self.current_position_y  # Dari Odometry
#         current_yaw = self.current_yaw       # Dari IMU

#         # =========================================================================
#         # LANGKAH 2: LOGIKA KONTROL SMC 
#         # =========================================================================
        
#         target_x, target_y = self.trajectory[self.current_waypoint_index]

#         # Hitung error jarak dan sudut
#         error_distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
#         angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
#         error_angular = self.normalize_angle(angle_to_target - current_yaw)

#         # Cek apakah waypoint sudah tercapai
#         if error_distance < self.waypoint_tolerance:
#             self.current_waypoint_index += 1
#             self.prev_error_angular = 0.0
            
#             if self.current_waypoint_index >= len(self.trajectory):
#                 self.is_trajectory_done = True
#                 self.stop_robot()
#                 self.get_logger().info('===== Trajektori Selesai! =====')
#             else:
#                 next_target = self.trajectory[self.current_waypoint_index]
#                 self.get_logger().info(f'Waypoint tercapai! Bergerak ke waypoint berikutnya: {next_target}')
#             return

#         # Sliding Mode Control untuk kecepatan angular
#         k_ang = self.get_parameter('k_angular').get_parameter_value().double_value
#         lambda_ang = self.get_parameter('lambda_angular').get_parameter_value().double_value

#         error_angular_dot = (error_angular - self.prev_error_angular) / dt
#         self.prev_error_angular = error_angular

#         sliding_surface = error_angular_dot + lambda_ang * error_angular
#         angular_speed = k_ang * np.tanh(sliding_surface)

#         # Kontrol kecepatan linear
#         k_lin = self.get_parameter('k_linear').get_parameter_value().double_value
#         linear_speed = k_lin * error_distance
        
#         # Kurangi kecepatan linear jika error angular besar
#         linear_speed *= math.exp(-lambda_ang * abs(error_angular))

#         # Batasi kecepatan
#         max_lin = self.get_parameter('max_linear_speed').get_parameter_value().double_value
#         max_ang = self.get_parameter('max_angular_speed').get_parameter_value().double_value
#         linear_speed = np.clip(linear_speed, 0, max_lin)
#         angular_speed = np.clip(angular_speed, -max_ang, max_ang)

#         # Kirim perintah kecepatan
#         twist_msg = Twist()
#         twist_msg.linear.x = linear_speed
#         twist_msg.angular.z = angular_speed
#         self.cmd_vel_pub.publish(twist_msg)
        
#         # Log informasi
#         self.get_logger().info(
#             f'Pos(Odom):({current_x:.2f},{current_y:.2f}) | '
#             f'Yaw(IMU):{math.degrees(current_yaw):.1f}° | '
#             f'WP:{self.current_waypoint_index} | '
#             f'Dist:{error_distance:.2f} | '
#             f'AngErr:{math.degrees(error_angular):.1f}° | '
#             f'LinSpd:{linear_speed:.2f} | '
#             f'AngSpd:{angular_speed:.2f}'
#         )

#     def stop_robot(self):
#         """Mengirim perintah berhenti total ke robot."""
#         twist_msg = Twist()
#         twist_msg.linear.x = 0.0
#         twist_msg.angular.z = 0.0
#         self.cmd_vel_pub.publish(twist_msg)

#     def euler_from_quaternion(self, quaternion):
#         """
#         Converts quaternion (w in last place) to euler roll, pitch, yaw
#         quaternion = [x, y, z, w]
#         """
#         x = quaternion.x
#         y = quaternion.y
#         z = quaternion.z
#         w = quaternion.w

#         sinr_cosp = 2 * (w * x + y * z)
#         cosr_cosp = 1 - 2 * (x * x + y * y)
#         roll = np.arctan2(sinr_cosp, cosr_cosp)

#         sinp = 2 * (w * y - z * x)
#         pitch = np.arcsin(sinp)
#         if np.abs(sinp) >= 1:
#             pitch = np.sign(sinp) * np.pi / 2.0

#         siny_cosp = 2 * (w * z + x * y)
#         cosy_cosp = 1 - 2 * (y * y + z * z)
#         yaw = np.arctan2(siny_cosp, cosy_cosp)

#         return roll, pitch, yaw

#     def normalize_angle(self, angle):
#         """Menormalkan sudut agar berada di antara -pi dan pi."""
#         while angle > math.pi: 
#             angle -= 2.0 * math.pi
#         while angle < -math.pi: 
#             angle += 2.0 * math.pi
#         return angle

# def main(args=None):
#     rclpy.init(args=args)
#     smc_tracker_node = SMCPathFollower()
#     try:
#         rclpy.spin(smc_tracker_node)
#     except KeyboardInterrupt:
#         smc_tracker_node.get_logger().info('Berhenti karena interrupsi keyboard.')
#     finally:
#         smc_tracker_node.stop_robot()
#         smc_tracker_node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()
