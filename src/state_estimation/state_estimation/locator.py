import rclpy
import numpy as np
from rclpy.node import Node
from scipy.optimize import minimize

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped


class LocatorNode(Node):

    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges: list[Range] = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
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
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)
    
    def calculate_position(self):
        if not len(self.anchor_ranges):
            return 0.0, 0.0, 0.0
        
        # YOUR CODE GOES HERE:
        x = np.mean([r.range for r in self.anchor_ranges]) - 0.5

        if len(self.anchor_ranges) < 2:
            return 0.0, 0.0, 0.0

        position_estimate = self.multilateration()

        self.get_logger().info(f"I am at {position_estimate}")

        # assume 2d (height will remain constant)
        return position_estimate[0], position_estimate[1], 0.0
    
    # https://github.com/glucee/Multilateration/blob/master/Python/example.py
    def multilateration(self):
        def error(x, c, r):
            return sum([(np.linalg.norm(x - c[i]) - r[i]) ** 2 for i in range(len(c))])
        
        anchor_coords = np.array([[range.anchor.x, range.anchor.y] for range in self.anchor_ranges])
        anchor_ranges = np.array([range.range for range in self.anchor_ranges])

        l = len(anchor_coords)
        S = sum(anchor_ranges)
	    # compute weight vector for initial guess
        W = [((l - 1) * S) / (S - w) for w in anchor_ranges]
	    # get initial guess of point location
        x0 = sum([W[i] * anchor_coords[i] for i in range(l)])
	    # optimize distance from signal origin to border of spheres
        return minimize(error, x0, args=(anchor_coords, anchor_ranges), method='Nelder-Mead').x

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
