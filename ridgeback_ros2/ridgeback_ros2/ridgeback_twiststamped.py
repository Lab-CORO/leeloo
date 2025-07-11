import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class RidgebackTwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('ridgeback_twist_to_twiststamped')

        # Create publisher
        self.twist_stamped_publisher = self.create_publisher(TwistStamped,'/r100_0597/cmd_vel',10)

        # Create subscriber
        self.twist_subscriber = self.create_subscription(Twist,'/cmd_vel',self.twist_callback,10)

    def twist_callback(self, msg: Twist):

        # Create a new TwistStamped message
        twist_stamped_msg = TwistStamped()

        # Add curent time of the node and sets the reference frame -> might need to change it 
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = "base_link" 

        # Copy the velocity date from the twist message
        twist_stamped_msg.twist = msg

        # Publisher the new message to the /r100_0597/cmd_vel
        self.twist_stamped_publisher.publish(twist_stamped_msg)

        # Write the action in terminal
        self.get_logger().info('Published TwistStamped message')


def main(args=None):
    rclpy.init(args=args)
    node = RidgebackTwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
