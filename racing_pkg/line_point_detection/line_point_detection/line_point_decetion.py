import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import CompressedImage
from origincar_msg.msg import Point
from hobot_dnn import pyeasy_dnn as dnn
import numpy as np
import cv2 , cv_bridge

import time

# 加载模型
models = dnn.load('/root/dev_ws/src/racing_pkg/line_point_detection/resnet18_224x224_nv12.bin')

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_point_detection')
        
        self.bridge = cv_bridge.CvBridge()
        
        self.img_sub = self.create_subscription(CompressedImage, '/image', self.img_callback, 10)
        
        self.line_point_pub = self.create_publisher(Point, '/line_point', 10)        
        
        self.get_logger().info("initialize successfully")
        
    
    
    def img_callback(self,msg):
        # self.get_logger().info("Following")
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        
        x, y, p = process_frame(cv_image, models, 640, 480)
           
        
        point = Point()
        point.line_point_x = x
        point.line_point_y
        point.confidence = p
        
        self.line_point_pub.publish(point)
        self.get_logger().info("line point is published")
            
        # self.get_logger().info("No Point") 

def bgr2nv12_opencv(image):
    height, width = image.shape[0], image.shape[1]
    area = height * width
    yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
    y = yuv420p[:area]
    uv_planar = yuv420p[area:].reshape((2, area // 4))
    uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))
    nv12 = np.zeros_like(yuv420p)
    nv12[:height * width] = y
    nv12[height * width:] = uv_packed
    return nv12

def print_properties(pro):
    print("tensor type:", pro.tensor_type)
    print("data type:", pro.dtype)
    print("layout:", pro.layout)
    print("shape:", pro.shape)

def process_frame(cv_image , models , ori_width , ori_height):
    
    cv_image_resized = cv2.resize(cv_image, (224, 224), interpolation=cv2.INTER_NEAREST) #最近邻插值方法进行缩放
    
    nv12_image = bgr2nv12_opencv(cv_image_resized)
    #nv12_image = convert_bgr_to_nv12(cv_image_resized)
    
    #outputs = models[0].forward(nv12_image)
    outputs = models[0].forward(np.frombuffer(nv12_image, dtype=np.uint8))
    outputs = outputs[0].buffer
    
    x_ratio, y_ratio, confidence = outputs[0][0][0][0], outputs[0][1][0][0], outputs[0][2][0][0]
    
    x_pixel = float((x_ratio * 224.0/2 + 224.0/2) * ori_width/224.0)
    y_pixel = float(224.0 - 224.0/2 * (y_ratio + 1) +256 )
    confidence = float(confidence)
    # print("load model time:",time_mid-time_begin)
    #print("process_frame time:",time_end-time_mid)#打印推理时间
    
    return x_pixel, y_pixel, confidence
    
def main(args=None):
       
    # # 打印模型输入输出信息
    # print("=" * 10, "inputs[0] properties", "=" * 10)
    # print_properties(models[0].inputs[0].properties)
    # print("inputs[0] name is:", models[0].inputs[0].name)

    # print("=" * 10, "outputs[0] properties", "=" * 10)
    # print_properties(models[0].outputs[0].properties)
    # print("outputs[0] name is:", models[0].outputs[0].name)

    
    rclpy.init(args=args)   
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
    
   