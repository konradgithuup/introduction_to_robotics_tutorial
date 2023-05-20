import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class VelocityController(Node):

    __max_speed = 0.1

    def __init__(self):
        super().__init__('velocity_controller')
        self.__move_timer = 0
        self.__turn_timer = 0
        self.__turn_counter = 0
        self.__spin = 0.5
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        
    def timer_cb(self):
        msg = Twist()
        
        if self.__turn_timer > 0:
            msg.angular.z = self.__spin
            self.__turn_timer -= 1
        else:
            x = self.forward_distance - 0.3
            x = x if x < VelocityController.__max_speed else VelocityController.__max_speed
            x = x if x >= 0 else 0.0
            msg.linear.x = x
            self.__move_timer -= 1
        
        self.publisher.publish(msg)
    
    def laser_cb(self, msg):
        self.forward_distance = msg.ranges[0]
        if self.__turn_timer > 0:
            return
        
        n_ranges = len(msg.ranges)
        left_range = msg.ranges[n_ranges//2]
        right_range = msg.ranges[n_ranges//2 - 1]

        incomplete_turn = (self.__turn_counter % 2 == 1) and (self.__move_timer <= 0)

        if self.forward_distance - 0.3 < VelocityController.__max_speed or incomplete_turn:
            # switch turn
            if (self.__turn_counter % 2) == 0:
                self.__spin *= -1

            self.__turn_timer = 32
            self.__move_timer = 40
            self.__turn_counter += 1
            self.get_logger().info(f"turn {self.__turn_counter}: turning in {'cw' if self.__spin < 0 else 'ccw'}")
            return

        



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
