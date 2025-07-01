import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np

def euler_from_quaternion(quaternion):
    x = quaternion.x; y = quaternion.y; z = quaternion.z; w = quaternion.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class SMCController(Node):
    def __init__(self):
        super().__init__('smc_controller_path_follower')

        # --- Deklarasi Parameter ---
        self.declare_parameter('lambda_psi', 1.5)
        self.declare_parameter('lambda_cte', 0.5)
        self.declare_parameter('k_val', 0.8)
        self.declare_parameter('phi_val', 0.2)
        self.declare_parameter('forward_velocity', 0.8)
        self.declare_parameter('goal_tolerance', 0.3)

        # --- Dapatkan nilai parameter ---
        self.lambda_psi_ = self.get_parameter('lambda_psi').get_parameter_value().double_value
        self.lambda_cte_ = self.get_parameter('lambda_cte').get_parameter_value().double_value
        self.k_ = self.get_parameter('k_val').get_parameter_value().double_value
        self.phi_ = self.get_parameter('phi_val').get_parameter_value().double_value
        self.v_ = self.get_parameter('forward_velocity').get_parameter_value().double_value
        self.goal_tol_ = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        # --- [PATH] Path lurus yang efisien hanya butuh titik akhir ---
        self.path = [(10.0, 0.0)] 
        self.waypoint_idx_ = 0
        self.path_completed = False
        self.start_waypoint_ = (0.0, 0.0)

        # --- Inisialisasi State ---
        self.robot_x = 0.0; self.robot_y = 0.0; self.robot_yaw = 0.0
        self.odom_received = False

        # --- Subscriber & Publisher ---
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_smc', 10)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('SMC Path Follower Node (CTE Enhanced) has been started.')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = euler_from_quaternion(msg.pose.pose.orientation)
        if not self.odom_received:
            self.start_waypoint_ = (self.robot_x, self.robot_y)
        self.odom_received = True

    def control_loop(self):
        if not self.odom_received or self.path_completed:
            self.stop_robot(); return

        target_waypoint = self.path[self.waypoint_idx_]
        
        dist_to_goal = math.sqrt((target_waypoint[0] - self.robot_x)**2 + (target_waypoint[1] - self.robot_y)**2)
        if dist_to_goal < self.goal_tol_:
            if not self.path_completed: # Cek agar pesan tidak berulang
                self.get_logger().info('Goal Reached!')
                self.path_completed = True
            return

        # --- Perhitungan Error (Orientasi + Jarak ke Garis) ---
        desired_yaw = math.atan2(target_waypoint[1] - self.robot_y, target_waypoint[0] - self.robot_x)
        e_psi = desired_yaw - self.robot_yaw
        if e_psi > math.pi: e_psi -= 2 * math.pi
        if e_psi < -math.pi: e_psi += 2 * math.pi

        y2 = target_waypoint[1]; y1 = self.start_waypoint_[1]
        x2 = target_waypoint[0]; x1 = self.start_waypoint_[0]
        e_cte = ((y2 - y1) * self.robot_x - (x2 - x1) * self.robot_y + x2 * y1 - y2 * x1) / (math.sqrt((y2-y1)**2 + (x2-x1)**2) + 1e-9)

        sigma = self.lambda_cte_ * e_cte + e_psi
        omega = self.lambda_psi_ * e_psi + self.k_ * np.tanh(sigma / self.phi_)

        cmd_msg = Twist(); cmd_msg.linear.x = self.v_; cmd_msg.angular.z = omega
        self.cmd_vel_publisher.publish(cmd_msg)

    def stop_robot(self):
        cmd_msg = Twist(); cmd_msg.linear.x = 0.0; cmd_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    smc_node = SMCController()
    rclpy.spin(smc_node)
    smc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()