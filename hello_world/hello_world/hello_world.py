import rclpy
from multiprocessing import Process
import time

def doWork():
    i = 1
    while True:
        time1 = 1
        str1 = "Hello ROS2 World!"
        p = str(i)
        print (p + str1)
        time.sleep(time1)
        i +=1 

def main():
    rclpy.init()
    hello_world = rclpy.create_node('hello_world')
    try:
        p = Process(target=doWork)
        p.start()
         
    except KeyboardInterrupt:
        pass
    hello_world.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

    

    