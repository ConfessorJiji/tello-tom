from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy
class TB3_teleop(Node):

    def __init__(self):
        super().__init__('TB3_publisher')
        self.mypub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(
            Joy, 'joy', self.mysubcallback, 10)

        self.create_timer(0.1, self.mytimercallback)
        self.controllerInput = Joy()

    def mytimercallback(self):
        mymsg = Twist()
        mymsg.linear.x = self.controllerInput.axes[1]
        mymsg.angular.z = self.controllerInput.axes[0]
        self.mypub.publish(mymsg)

    def mysubcallback(self, msg):
         self.controllerInput = msg
        


def main():
    rclpy.init()
    node_handle = TB3_teleop()
    try:
        rclpy.spin(node_handle)
    except KeyboardInterrupt:
        pass
    node_handle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
