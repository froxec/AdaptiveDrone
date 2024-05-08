import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import SetMode
class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.client_ = self.create_client(SetMode, "/mavros/set_mode")
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = SetMode.Request()
        msg.custom_mode = "LAND"
        self.client_.call(msg)
        self.get_logger().info('Publishing: "%s"' % msg.custom_mode)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    controller_node = Controller()

    rclpy.spin(controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
