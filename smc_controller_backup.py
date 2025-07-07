
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
#         self.accel_sum = np.array([0.0, 0.0, 0.0])

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

#         sinp = 2 * (w *
# y - z * x)
        


# setup.py bawaan



# from setuptools import find_packages, setup

# package_name = 'smc_controller'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='satyo03',
#     maintainer_email='satyosecond03@gmail.com',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'smc_controller = smc_controller.smc_controller:main',
#         ],
#     },
# )