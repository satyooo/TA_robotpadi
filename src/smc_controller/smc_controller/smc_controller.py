#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
import math

class SMCPathFollower(Node):
    def __init__(self):
        super().__init__('smc_path_follower')
        
        self.get_logger().warn("====================== PERINGATAN ======================")
        self.get_logger().warn("Node ini mengestimasi posisi HANYA dari integrasi IMU.")
        self.get_logger().warn("Metode ini SANGAT RENTAN TERHADAP DRIFT dan tidak akurat.")
        self.get_logger().warn("Gunakan hanya untuk tujuan eksperimental/akademis.")
        self.get_logger().warn("========================================================")

        # --- Parameter untuk Tugas Akhir (Bisa diubah dari command line) ---
        self.declare_parameter('trajectory', [1.0, 0.0])
        self.declare_parameter('waypoint_tolerance', 0.1) # Toleransi sedikit lebih besar karena drift

        # --- Parameter untuk Sliding Mode Control (SMC) ---
        self.declare_parameter('k_angular', 2.0)
        self.declare_parameter('lambda_angular', 3.0)
        self.declare_parameter('k_linear', 0.7)
        
        # --- Batas Kecepatan Robot ---
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.2)

        # --- Ambil nilai parameter dan proses trajektori ---
        self.trajectory = self.parse_trajectory_param()
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        
        # --- Inisialisasi Publisher dan Subscriber ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # SATU-SATUNYA SUBSCRIBER: /imu/data
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.control_loop_callback, 10)
        
        # --- Variabel State Kontrol ---
        self.current_waypoint_index = 0
        self.is_trajectory_done = False if self.trajectory else True
        self.last_time = None
        
        # Variabel untuk menyimpan state robot (posisi, kecepatan, orientasi)
        self.current_position = np.array([0.0, 0.0]) # [x, y]
        self.current_velocity = np.array([0.0, 0.0]) # [vx, vy]
        self.current_yaw = 0.0
        self.prev_error_angular = 0.0

        if self.is_trajectory_done:
            self.get_logger().warn('Trajektori kosong! Node tidak akan melakukan apa-apa.')
        else:
            self.get_logger().info(f'SMC IMU-Only Tracker dimulai. Mengikuti {len(self.trajectory)} waypoints.')
            self.get_logger().info(f'Waypoint pertama: {self.trajectory[0]}')

    def parse_trajectory_param(self):
        """Mengubah parameter list flat [x1,y1,x2,y2] menjadi list of tuples [(x1,y1), (x2,y2)]."""
        flat_list = self.get_parameter('trajectory').get_parameter_value().double_array_value
        if len(flat_list) % 2 != 0:
            self.get_logger().error('Parameter trajektori harus memiliki jumlah elemen genap (pasangan x,y).')
            return []
        return [(flat_list[i], flat_list[i+1]) for i in range(0, len(flat_list), 2)]

    def control_loop_callback(self, msg: Imu):
        """
        Callback ini adalah satu-satunya pemicu kontrol.
        1. Mengestimasi posisi & orientasi dari IMU.
        2. Menjalankan logika kontrol SMC.
        """
        if self.is_trajectory_done:
            return

        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return
        
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0: return

        # =========================================================================
        # LANGKAH 1: ESTIMASI STATE ROBOT HANYA DARI IMU
        # =========================================================================
        
        # 1a. Dapatkan Orientasi (Yaw) menggunakan fungsi baru
        # Kita hanya butuh yaw, jadi roll dan pitch diabaikan dengan '_'
        _, _, self.current_yaw = self.euler_from_quaternion(msg.orientation)

        # 1b. Dapatkan Percepatan Linear dari IMU (dalam frame robot)
        accel_robot_frame = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y])

        # 1c. Putar vektor percepatan ke frame global/dunia (World Frame)
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        accel_world_frame = np.array([
            accel_robot_frame[0] * cos_yaw - accel_robot_frame[1] * sin_yaw,
            accel_robot_frame[0] * sin_yaw + accel_robot_frame[1] * cos_yaw
        ])

        # 1d. Integrasi pertama: Percepatan -> Kecepatan
        self.current_velocity += accel_world_frame * dt

        # 1e. Integrasi kedua: Kecepatan -> Posisi
        self.current_position += self.current_velocity * dt
        
        current_x = self.current_position[0]
        current_y = self.current_position[1]

        # =========================================================================
        # LANGKAH 2: LOGIKA KONTROL SMC 
        # =========================================================================
        
        target_x, target_y = self.trajectory[self.current_waypoint_index]

        error_distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        error_angular = self.normalize_angle(angle_to_target - self.current_yaw)

        if error_distance < self.waypoint_tolerance:
            self.current_waypoint_index += 1
            self.prev_error_angular = 0.0
            
            if self.current_waypoint_index >= len(self.trajectory):
                self.is_trajectory_done = True
                self.stop_robot()
                self.get_logger().info('===== Trajektori Selesai! =====')
            else:
                next_target = self.trajectory[self.current_waypoint_index]
                self.get_logger().info(f'Waypoint tercapai! Bergerak ke waypoint berikutnya: {next_target}')
            return

        k_ang = self.get_parameter('k_angular').get_parameter_value().double_value
        lambda_ang = self.get_parameter('lambda_angular').get_parameter_value().double_value

        error_angular_dot = (error_angular - self.prev_error_angular) / dt
        self.prev_error_angular = error_angular

        sliding_surface = error_angular_dot + lambda_ang * error_angular
        angular_speed = k_ang * np.tanh(sliding_surface)

        k_lin = self.get_parameter('k_linear').get_parameter_value().double_value
        linear_speed = k_lin * error_distance
        linear_speed *= math.exp(-lambda_ang * abs(error_angular))

        max_lin = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        max_ang = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        linear_speed = np.clip(linear_speed, 0, max_lin)
        angular_speed = np.clip(angular_speed, -max_ang, max_ang)

        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist_msg)
        
        self.get_logger().info(f'Pos (est):({current_x:.2f},{current_y:.2f}) | WP:{self.current_waypoint_index} | Dist:{error_distance:.2f} | Ang:{error_angular:.2f}')

    def stop_robot(self):
        """Mengirim perintah berhenti total ke robot."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)
        if np.abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2.0 # use 90 degrees if out of range

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def normalize_angle(self, angle):
        """Menormalkan sudut agar berada di antara -pi dan pi."""
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    smc_tracker_node = SMCPathFollower()
    try:
        rclpy.spin(smc_tracker_node)
    except KeyboardInterrupt:
        smc_tracker_node.get_logger().info('Berhenti karena interupsi keyboard.')
    finally:
        smc_tracker_node.stop_robot()
        smc_tracker_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


