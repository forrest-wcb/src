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
# from pyzbar.pyzbar import decode
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
        self.foxstate_publisher = self.create_publisher(
            Sign, 
            '/sign_foxglove', 
            10)
        
        self.qrstate_publisher = self.create_publisher(
            Sign, 
            '/sign_switch',
            10)
        
        # self.cmd_vel_pub = self.create_publisher(
        #     Twist, 
        #     'cmd_vel', 
        #     10)

    def send_foxglove_signal(self, signal_value): #上位机信号
        msg = Sign()
        msg.sign_data = signal_value
        self.foxstate_publisher.publish(msg)
        self.get_logger().info(f"上位机信号发布了{msg.sign_data}")
        
    def send_car_signal(self, signal_value): #小车信号
        msg = Sign()
        msg.sign_data = signal_value
        self.qrstate_publisher.publish(msg)
        self.get_logger().info(f"小车信号发布了{msg.sign_data}")
        
    def listener_callback(self, msg):
        
        #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image =  self.bridge.compressed_imgmsg_to_cv2(msg,"bgr8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
        ret, th = cv2.threshold(cv_image, 0, 255, cv2.THRESH_OTSU)
        res = str(pyzbar.decode(th))
        
        # for barcode in decode(th):
        #     print (barcode.rect)
        match = re.search(r"data=b'([^']+)'", res)
        
        # fox_state = Int32()  #上位机信号
        # state = Sign()  #小车信号
        
        if match:
            data_value = match.group(1)
                      
            if data_value == "ClockWise" :
                # state.sign_data = 3
                # self.qrstate_publisher.publish(state)
                self.send_car_signal(3)
                self.get_logger().info("ClockWise")
                
            elif data_value == "AntiClockWise" :
                # state.sign_data = 4
                # self.qrstate_publisher.publish(state)
                self.send_car_signal(4)
                self.get_logger().info("AntiClockWise")
                
            # fox_state.data = 2
            # self.foxstate_publisher.publish(fox_state)
            self.send_foxglove_signal(2)
            
            # self.get_logger().info(f"接收到{data_value},发布了{state.sign_data}")
           
            self.destroy_node()
            
        else: 
            self.send_foxglove_signal(1)
            # fox_state.data = 1
            # self.foxstate_publisher.publish(fox_state)
            # self.get_logger().info("No Match")
            
            
            
def main(args=None):
    rclpy.init(args=args)
    decoder_node = DecoderNode()
    rclpy.spin(decoder_node)
    decoder_node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()   
        