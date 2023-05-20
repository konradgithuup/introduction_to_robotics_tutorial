import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class VelocityController(Node):

    __max_speed = 0.1
    __track_width = 0.25

    def __init__(self):
        super().__init__('velocity_controller')
        self.__spin = False
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        
    def timer_cb(self):
        msg = Twist()
        x = self.forward_distance - 0.3
        x = x if x < VelocityController.__max_speed else VelocityController.__max_speed
        x = x if x >= 0 else 0.0
        msg.linear.x = x
        
        if self.__spin:
            msg.angular.z = 0.5
        
        self.publisher.publish(msg)
    
    def laser_cb(self, msg):
        self.forward_distance = msg.ranges[0]
        for range in msg.ranges:
            if range - 0.3 < VelocityController.__max_speed:
                self.__spin = True
                return
        self.__spin = False

        



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
