#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy, cv2, cv_bridge, numpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import time


class Shooter(Node):
    def __init__(self):
        super().__init__('image_shooter')
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(CompressedImage, '/image', self.image_callback, 10)
        self.timer = self.create_timer(0.3,self.timer_callback)
        self.image = numpy.matrix(0)
        self._last_shot_time = time.time()
        self.get_logger().info("Image shooter init finished.")

    def timer_callback(self):
        current_time = time.time()
        write_file_path = "/root/dev_ws/img/{}.jpg".format(current_time)
        try:
            cv2.imwrite(write_file_path,self.image)
            self.get_logger().info("Shot at {0}, save to {1}.".format(current_time,write_file_path))
        except Exception as e:
            print(e)
    def image_callback(self, msg):
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg,"bgr8")
        #self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")

  
  


def main(args=None):
    rclpy.init(args=args)
    shooter = Shooter()
    rclpy.spin(shooter)
    shooter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()