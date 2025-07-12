#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('coordinate_publisher')
        self.publisher_ = self.create_publisher(Point, '/coordinates', 10)
        
        # --- PERUBAHAN DIMULAI ---
        # Siapkan data yang akan dipublikasikan
        self.coordinates = [[float(x), 0.0] for x in range(20, 41)]
        self.current_index = 0
        
        # Buat timer yang memanggil fungsi callback secara berkala
        timer_period = 0.2  # detik (misalnya, publikasikan 5 koordinat per detik)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('CoordinatePublisher node started. Publishing coordinates periodically.')
        # --- PERUBAHAN SELESAI ---

    # --- FUNGSI BARU ---
    def timer_callback(self):
        # Periksa apakah semua koordinat sudah dipublikasikan
        if self.current_index < len(self.coordinates):
            x, y = self.coordinates[self.current_index]
            
            msg = Point()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = 0.0

            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: x = {msg.x}, y = {msg.y}, z = {msg.z}')
            
            self.current_index += 1
        else:
            # Jika semua sudah terkirim, hentikan timer agar tidak berjalan terus
            self.get_logger().info('All coordinates have been published. Timer stopped.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point

# class CoordinatePublisher(Node):
#     def __init__(self):
#         super().__init__('coordinate_publisher')  # Nama node
#         self.publisher_ = self.create_publisher(Point, '/coordinates', 10)  # Nama topik dan queue size
#         self.publish_coordinates()  # Memanggil fungsi untuk langsung mempublikasikan koordinat

#     def publish_coordinates(self):
#         coordinates = [[float(x), 0.0] for x in range(20, 41)]  # Mengubah koordinat menjadi float
        
#         # Publikasikan semua koordinat sekaligus
#         for x, y in coordinates:
#             msg = Point()
#             msg.x = float(x)  # Pastikan x adalah float
#             msg.y = float(y)  # Pastikan y adalah float
#             msg.z = 0.0       # Tetap di sumbu Z = 0.0

#             # Publikasikan pesan
#             self.publisher_.publish(msg)

#             # Tampilkan di terminal
#             self.get_logger().info(f'Publishing: x = {msg.x}, y = {msg.y}, z = {msg.z}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = CoordinatePublisher()
#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
