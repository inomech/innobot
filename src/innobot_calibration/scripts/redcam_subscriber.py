#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Int32MultiArray
from rclpy.qos import ReliabilityPolicy, QoSProfile

class RedCamSubscNode(Node):

    def __init__(self):
        super().__init__('redcam_subscriber')

        self.t1_counter = 0
        self.t2_counter = 0
        self.t3_counter = 0

        self.t1_data = []
        self.t2_data = []
        self.t3_data = []

        self.redcam_subscriber_t1 = self.create_subscription(
            Int32MultiArray,
            '/RedCamT1',
            self.redcam_t1_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) 

        self.redcam_subscriber_t2 = self.create_subscription(
            Int32MultiArray,
            '/RedCamT2',
            self.redcam_t2_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) 

        self.redcam_subscriber_t3 = self.create_subscription(
            Int32MultiArray,
            '/RedCamT3',
            self.redcam_t3_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) 

        self.srv = self.create_service(Empty, 'redcam_logging', self.logging_callback)
        
        # to prevent unused var warning
        self.redcam_subscriber_t1

        self.redcam_subscriber_t2

        self.redcam_subscriber_t3
    
    def logging_callback(self,request,response):

        self.get_logger().info("Logging RedCam Data")

        np.savetxt("RedCamT1.csv",np.array(self.t1_data),delimiter=',')
        np.savetxt("RedCamT2.csv",np.array(self.t2_data),delimiter=',')
        np.savetxt("RedCamT3.csv",np.array(self.t3_data),delimiter=',')

        self.t1_data.clear()
        self.t2_data.clear()
        self.t3_data.clear()

        return response



    def redcam_t1_callback(self, msg):
        
        self.phix1 = msg.data[1]
        self.phiz1 = msg.data[2]
        self.x1 = msg.data[3]
        
        self.t1_msg = np.array([
            self.x1,
            self.phix1,
            self.phiz1])

        self.t1_data.append(self.t1_msg)


    def redcam_t2_callback(self, msg):
        
        self.phix2 = msg.data[1]
        self.phiz2 = msg.data[2]
        self.x2 = msg.data[3]
        
        self.t2_msg = np.array([
            self.x2,
            self.phix2,
            self.phiz2])

        self.t2_data.append(self.t2_msg)


    def redcam_t3_callback(self, msg):
        
        self.phix3 = msg.data[1]
        self.phiz3 = msg.data[2]
        self.x3 = msg.data[3]
        
        self.t3_msg = np.array([
            self.x3,
            self.phix3,
            self.phiz3])

        self.t3_data.append(self.t3_msg)

            
def main(args=None):

    rclpy.init(args=args)

    simple_subscriber = RedCamSubscNode()

    rclpy.spin(simple_subscriber)

    simple_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
