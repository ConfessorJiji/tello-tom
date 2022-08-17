from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tello_msgs.srv import TelloAction
import rclpy
class tello_teleop(Node):

    def __init__(self):
        super().__init__('tello_publisher')
        self.mypub = self.create_publisher(Twist, '/drone1/cmd_vel', 1)
        self.subscription = self.create_subscription(
            Joy, '/drone1/joy', self.mysubcallback, 10)
        self.create_timer(0.1, self.mytimercallback)
        self.controllerInput = Joy()
        #self.srv = self.create_client(TelloAction, '/tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')


    def mytimercallback(self):
        mymsg = Twist()
        mymsg.linear.x = self.controllerInput.axes[1]
        mymsg.angular.z = self.controllerInput.axes[0]
        mymsg.linear.z = self.controllerInput.axes[4]
        self.mypub.publish(mymsg)
'''     
        if self.controllerInput.buttons[2]==1:
            rqs = TelloAction.Request()
            rqs.cmd = 'takeoff'
            self.srv.call_async(TelloAction)
        if self.controllerInput.buttons[3]==1:
            rqs = TelloAction.Request()
            rqs.cmd = 'land'
            self.srv.call_async(TelloAction)
'''      

def mysubcallback(self, msg):
         self.controllerInput = msg




def main():
    rclpy.init()
    node_handle = tello_teleop()
    try:
        rclpy.spin(node_handle)
    except KeyboardInterrupt:
        pass
    node_handle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
