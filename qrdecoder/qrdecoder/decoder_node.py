import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from origincar_msg.msg import Sign
from cv_bridge import CvBridge
import cv2
import numpy as np
import re
from pyzbar import pyzbar
from geometry_msgs.msg import Twist

class DecoderNode(Node):

    def __init__(self):
        super().__init__('decoder_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.foxstate_publisher = self.create_publisher(Int32, 'sign_foxglove', 10)
        self.qrstate_publisher = self.create_publisher(Sign, 'sign_switch', 10) # Add this line
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        
        #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image =  self.bridge.compressed_imgmsg_to_cv2(msg,"bgr8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
        ret, th = cv2.threshold(cv_image, 0, 255, cv2.THRESH_OTSU)
        res = str(pyzbar.decode(th))
        
        for barcode in decode(th):
            print (barcode.rect)
        match = re.search(r"data=b'([^']+)'", res)
        fox_state = Int32()
        
        if match:
            data_value = match.group(1)
            state = Sign()           
            if data_value == "ClockWise" :
                state.sign_data = 3
                
                self.qrstate_publisher.publish(state)
                
                # self.get_logger().info("Publish Successfully")
                
            elif data_value == "AntiClockWise" :
                state.sign_data = 4
                
                self.qrstate_publisher.publish(state)
                
                # self.get_logger().info("Publish Successfully")
                
            fox_state.data = 2
            self.foxstate_publisher.publish(fox_state)
            
            # self.get_logger().info(f"接收到{data_value},发布了{state.sign_data}")
            # self.get_logger().info(f"到达目的地，发布了{fox_state.data}")
            self.destroy_node()
        else: 
            fox_state.data = 1
            self.foxstate_publisher.publish(fox_state)
            self.get_logger().info("No Match")
            
            
            
def main(args=None):
    rclpy.init(args=args)
    decoder_node = DecoderNode()
    rclpy.spin(decoder_node)
    decoder_node.destroy_node()
    rclpy.shutdown()    

        
        