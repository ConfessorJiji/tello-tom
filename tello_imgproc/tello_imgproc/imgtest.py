#!/usr/bin/env python3

import cv_bridge
from geometry_msgs import msg
from geometry_msgs.msg import Twist
import rclpy
from std_msgs.msg import UInt32
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from tello_msgs.srv import TelloAction
import cv2 as cv
from cv_bridge import CvBridge


rclpy.init()
node_handle = rclpy.create_node('imgtest')
mypub = node_handle.create_publisher(Image, '/processImg', 1)
global imgCount
imgCount = 0
print("I reset the image count")

def main():


    node_handle.create_subscription(Image, 'image_raw', mysubcallback, 10)

    try:
        rclpy.spin(node_handle)
    except KeyboardInterrupt:
        pass
    node_handle.destroy_node()
    rclpy.shutdown()

def mysubcallback(msg):
    global imgCount
    imgCount = imgCount + 1
    print('Image Number: ', imgCount )
    cv_bridge = CvBridge()
    cv_frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    blur = cv.blur(cv_frame,(10,10))
    ros_frame = cv_bridge.cv2_to_imgmsg( blur)
    ros_frame.header.frame_id = msg.header.frame_id
    
    

    mypub.publish(ros_frame)



if __name__ == '__main__':
    main()  