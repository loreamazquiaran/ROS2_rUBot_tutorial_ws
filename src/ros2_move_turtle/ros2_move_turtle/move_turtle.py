import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import curses
from threading import Thread

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.limit = 7.0
        self.linear = 0.0
        self.angular = 0.0

    def pose_callback(self, msg):
        twist = Twist()
        if msg.x > self.limit or msg.y > self.limit:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('¡Límite alcanzado! La tortuga se detiene.')
        else:
            twist.linear.x = self.linear
            twist.angular.z = self.angular
        self.publisher.publish(twist)

    def set_direction(self, key):
        if key == curses.KEY_UP:
            self.linear = 2.0
            self.angular = 0.0
        elif key == curses.KEY_DOWN:
            self.linear = -2.0
            self.angular = 0.0
        elif key == curses.KEY_LEFT:
            self.linear = 0.0
            self.angular = 1.5
        elif key == curses.KEY_RIGHT:
            self.linear = 0.0
            self.angular = -1.5
        elif key == ord('x'):
            self.linear = 0.0
            self.angular = 0.0

def keyboard_loop(node):
    def curses_loop(stdscr):
        stdscr.nodelay(True)
        stdscr.clear()
        stdscr.addstr(0, 0, "Usa las flechas para mover, x para detener, q para salir.")
        while True:
            key = stdscr.getch()
            if key == ord('q'):
                break
            elif key != -1:
                node.set_direction(key)
    curses.wrapper(curses_loop)

def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtle()
    Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    keyboard_loop(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
