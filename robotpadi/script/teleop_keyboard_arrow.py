#!/usr/bin/env python3
import sys, select, termios, tty
import rclpy
from geometry_msgs.msg import Twist

# Mapping tombol panah (escape sequence)
moveBindings = {
    '\x1b[A': (1, 0),   # ↑ maju
    '\x1b[B': (-1, 0),  # ↓ mundur
    '\x1b[C': (0, -1),  # → kanan
    '\x1b[D': (0, 1),   # ← kiri
}

speed = 0.5
turn = 1.0

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        ch1 = sys.stdin.read(1)
        if ch1 == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                return ch1 + ch2 + ch3
        return ch1
    return ''

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('teleop_arrow')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    try:
        print("Gunakan arrow keys untuk kontrol robot")
        print("Tekan Ctrl+C untuk keluar")

        while True:
            key = getKey()
            twist = Twist()

            if key in moveBindings:
                x, th = moveBindings[key]
                twist.linear.x = x * speed
                twist.angular.z = th * turn
            elif key == '\x03':  # Ctrl+C
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
