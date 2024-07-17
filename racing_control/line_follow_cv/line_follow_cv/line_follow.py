import rclpy, cv2, cv_bridge, numpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
# from origincar_msg.msg import Int32
from origincar_msg.msg import Sign

class Follower(Node):
    def __init__(self):
        super().__init__('line_follower')
        # self.get_logger().info("Start line follower.")

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = self.create_subscription(CompressedImage, '/image', self.image_callback, 10)
        self.get_logger().info("Subscribed to /image")

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Published to /cmd_vel")

        self.pub = self.create_publisher(CompressedImage, '/camera/process_image', 10)
        self.get_logger().info("Published to /camera/process_image")
        
        self.foxglove_sub = self.create_subscription(Sign, "/sign_foxglove", self.pause_callback, 10)

        self.twist = Twist()
        # 
        self.paused = False

        # 添加暂停订阅器
        # self.pause_sub = self.create_subscription(Int32, 'sign_foxglove', self.pause_callback, 10)
        
    # def pause_callback(self, msg):
    #     if msg.data == 1:  # Example condition to pause
    #         self.paused = False
    #         self.get_logger().info("Node running.")
    #     elif msg.data == 6:  # Example condition to resume
    #         self.paused = False
    #         self.get_logger().info("Node running.")
    #     else:
    #         self.paused = True
    #         self.get_logger().info("Node paused.")
        
        def pause_callback(self, msg : Sign):
            if msg.sign_data == 5:  # Example condition to pause
                self.paused = True
                self.get_logger().info("Paused")
                self.get_logger().info("Node is paused, skipping image callback.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
            else:
                self.paused = False
                self.get_logger().info("Unpaused")
                self.get_logger().info("Following...")
        
    def image_callback(self, msg):
        #image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # if self.paused:
        #     self.get_logger().info("Node is paused, skipping image callback.")
        #     self.twist.linear.x = 0.0
    
        #     self.cmd_vel_pub.publish(self.twist)
        #     return
        
        # image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8') #compressed_imgmsg_to_cv2
        if self.paused:

            pass

        else:
            image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8') #compressed_imgmsg_to_cv2

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        self.get_logger().info("Following...")
        lower = numpy.array([ 0,  0, 0])
        upper = numpy.array([180, 255, 46])
        mask = cv2.inRange(hsv, lower, upper)

        h, w, d = image.shape

        search_top = int(h/2)
        search_bot = int(h/2 + 20)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00']) 
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            err = cx - w/2
            self.twist.linear.x = 0.18
            self.twist.angular.z = -float(err) / 400
            self.cmd_vel_pub.publish(self.twist)
            
        #self.pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))
        self.pub.publish(self.bridge.cv2_to_compressed_imgmsg(image))   #cv2_to_compressed_imgmsg

def main(args=None):
    rclpy.init(args=args)    
    follower = Follower()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()