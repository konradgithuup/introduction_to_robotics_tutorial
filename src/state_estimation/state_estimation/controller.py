import rclpy
from rclpy.node import Node

import numpy as np

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.goal = None
        self.position_old = [0,0]
        self.turn_angle = 0.0
        self.position = None
        self.turning = False
        self.turning_cd = 0
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        
    def timer_cb(self):
        msg = Twist()
        x = self.forward_distance - 0.3
        x = x if x < 0.1 else 0.1
        x = x if x >= 0 else 0.0

        if (self.turning):
            msg.angular.z = np.pi/4
            self.turn_angle -= np.pi/40
            if (self.turn_angle <= 0):
                self.turning = False
        elif (self.turning_cd > 0):
            msg.linear.x = x
            self.turning_cd -= 1
        else:
            v = self.determine_trajectory()
            ang = self.determine_angle(v)
            self.turn_angle = ang
            self.turning = True
            self.turning_cd = 10

        self.publisher.publish(msg)
    
    def goal_cb(self, msg):
        goal = msg.pose.position.x, msg.pose.position.y
        if self.goal != goal:
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal
    
    def laser_cb(self, msg):
        self.forward_distance = msg.ranges[0]
        
    def position_cb(self, msg):
        self.position_old = self.position
        self.position = msg.point.x, msg.point.y
    
    def determine_angle(self, v: list[float]) -> float:
        if (self.position == None or self.position_old == None):
            return 0.0
        current = [self.position[0] - self.position_old[0], self.position[1] - self.position_old[1]]
        v_adjusted = [v[0] - current[0], v[1] - current[1]]
        a_degrees = np.arccos(np.dot(v_adjusted, current) / (np.linalg.norm(v_adjusted) * np.linalg.norm(current)))

        return (np.pi / 180) * a_degrees

    def determine_trajectory(self):
        if (self.position == None or self.position_old == None):
            self.get_logger().warn("2 positions required to calculate trajectory")
            return [0.1, 0.0]

        delta_x = abs(self.position_old[0] - self.position[0])
        delta_y = abs(self.position_old[1] - self.position[1])

        v_x = self.partial(delta_x, "x")
        v_y = self.partial(delta_y, "y")

        return [v_x, v_y]

    def partial(self, d: float, partial: str) -> float:
        if (d == 0):
            d = 0.01

        x = self.position[0]
        y = self.position[1]
        p = [0.0, 0.0]
        if partial == "x":
            p = [x+d, y]
        elif partial == "y":
            p = [x, y+d]

        f_part = (self.evaluate(p[0], p[1]) - self.evaluate(x, y)) / d
        return f_part

    def evaluate(self, x, y) -> float:
        
        return np.linalg.norm([(x - self.goal[0]) ** 2,
                                   (y - self.goal[1]) ** 2])

def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
