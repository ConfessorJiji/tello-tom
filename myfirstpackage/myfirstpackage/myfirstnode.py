#!/usr/bin/env python3

import rclpy

def main():
    rclpy.init()
    myfirstnode = rclpy.create_node('myfirstnode')
    try:
        rclpy.spin(myfirstnode)
    except KeyboardInterrupt:
        pass  
    myfirstnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()