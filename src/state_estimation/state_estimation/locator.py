import rclpy
import numpy as np
from rclpy.node import Node

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped


class LocatorNode(Node):

    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.position = np.zeros((3,))
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('locator node started')
        
    def range_cb(self, msg):
        self.anchor_ranges.append(msg)
        self.anchor_ranges = self.anchor_ranges[-10:]
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')

    def timer_cb(self):
        if not self.initialized:
            self.get_logger().info('no range received yet')
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.get_logger().info(f'publishing position: {msg}')
        self.position_pub.publish(msg)
    
    def calculate_position(self):
        if not len(self.anchor_ranges):
            return 0.0, 0.0, 0.0
        x_old = self.position

        for _ in range(100):
            matrix = np.array([self.anchor_ranges[i].range
                               - np.linalg.norm(np.array([
                                   self.anchor_ranges[i].anchor.x,
                                   self.anchor_ranges[i].anchor.y,
                                   self.anchor_ranges[i].anchor.z]) - x_old)
                                   for i in range(len(self.anchor_ranges))])
            # partial derivatives for x, y, z
            derivative_matrix = -1 * np.array([(x_old[j] - [self.anchor_ranges[i].anchor.x,
                                                            self.anchor_ranges[i].anchor.y,
                                                            self.anchor_ranges[i].anchor.z][j])
                                               / np.linalg.norm(x_old - np.array([
                                                   self.anchor_ranges[i].anchor.x,
                                                   self.anchor_ranges[i].anchor.y,
                                                   self.anchor_ranges[i].anchor.z]))
                                                   for i in range(len(self.anchor_ranges))
                                                   for j in range(3)]).reshape(len(self.anchor_ranges), 3)
            # insert the drake memes here
            x_new = x_old - np.matmul(np.linalg.pinv(derivative_matrix), matrix)
            if np.linalg.norm(x_new - x_old) < 0.01:
                break
            x_old = x_new
        self.position = x_old
        self.get_logger().info(f"I am at {x_old}")
        return tuple(x_old)


def main(args=None):
    rclpy.init(args=args)

    node = LocatorNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()