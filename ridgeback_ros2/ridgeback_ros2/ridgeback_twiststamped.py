import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

from std_msgs.msg import Header

# This node converts incoming geometry_msgs/Twist messages from the Unity interface
# into geometry_msgs/TwistStamped messages.
# It listens to the /unity/cmd_vel topic from Unity and republishes the stamped messages
# to /r100_0597/cmd_vel for use with the Ridgeback robot.
# Source : https://github.com/joshnewans/twist_stamper/blob/main/twist_stamper/twist_stamper.py
class RidgebackTwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('ridgeback_twist_to_twiststamped')

        # Create publisher
        self.twist_stamped_publisher = self.create_publisher(TwistStamped,'/r100_0597/cmd_vel',10)

        # Create subscriber
        self.twist_subscriber = self.create_subscription(Twist,'/unity/cmd_vel',self.twist_callback,10)

    def twist_callback(self, msg: Twist):

        # Create a new TwistStamped message
        twist_stamped_msg = TwistStamped()

        # Add curent time of the node and set the reference frame
        twist_stamped_msg.header = Header()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = ''

        # Add the velocity rate from the twist message 
        twist_stamped_msg.twist = msg

        # Publish the new message to the /r100_0597/cmd_vel topic
        self.twist_stamped_publisher.publish(twist_stamped_msg)

        # Write the action in terminal
        self.get_logger().info('TwistStamped message published')


def main(args=None):
    rclpy.init(args=args)
    node = RidgebackTwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
