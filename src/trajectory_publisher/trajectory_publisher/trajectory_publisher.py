#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time

class LurusTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('lurus_trajectory_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/trajectory', 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory)  # Publikasi setiap 1 detik

        # Titik awal robot (x, y)
        self.start_position = (20.0, 0.0)  # posisi mulai di x=0, y=0
        self.distance = 20.0  # Trajectory sepanjang 20 meter ke arah sumbu X
        self.step_size = 0.5  # Langkah per titik (setiap 0.5 meter)
        
        # Menghasilkan posisi titik di sepanjang sumbu x
        self.trajectory = [(self.start_position[0] + i * self.step_size, self.start_position[1] + i * self.step_size, 0.0) 
                           for i in range(int(self.distance / self.step_size) + 1)]
        
        self.current_index = 0  # Untuk melacak titik saat ini di trajectory
        self.last_point = self.trajectory[-1]

    def publish_trajectory(self):
        if self.current_index < len(self.trajectory):
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'odom'  # Sesuaikan dengan frame robot
            # pose.pose.position.x = self.trajectory[self.current_index][0]
            # pose.pose.position.y = self.trajectory[self.current_index][1]
            # pose.pose.position.z = self.trajectory[self.current_index][2]
            pose.pose.position.x = self.last_point[0]
            pose.pose.position.y = self.last_point[1]
            pose.pose.position.z = self.last_point[2]

            self.publisher.publish(pose)
            self.get_logger().info(f'Publishing trajectory point: {pose.pose.position.x}, {pose.pose.position.y}')
            self.current_index += 1
        else:
            self.current_index = 0  # Kembali ke awal jika sudah selesai


def main(args=None):
    rclpy.init(args=args)
    node = LurusTrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
