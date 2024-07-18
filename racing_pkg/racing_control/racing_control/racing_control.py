import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets
from geometry_msgs.msg import Twist
from origincar_msg.msg import Point

# 参数设定
y_bottom_threshold = 380.0  # 避障阈值
confidence_threshold = 0.5  # 置信度阈值

follow_angular_ratio = -1.0  # 巡线速度变化系数
avoid_angular_ratio = 1.0  # 避障角速度变化系数

follow_linear_speed = 0.7  # 循线速度
avoid_linear_speed = 0.4  # 避障线速度

restore_angular_speed = 0.5 #回正幅度

class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('racing_control')
        self.subscription_perception = self.create_subscription(
            PerceptionTargets,
            'hobot_dnn_detection',
            self.perception_callback,
            10)

        self.subscription_point = self.create_subscription(
            Point,
            'line_point_detection',
            self.line_follow_callback,
            10)

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        
        self.twist = Twist()
        self.avoid_obstacle = False
        self.line_dropped = False
        self.last_turn_direction = None # 1表示向左, -1表示向右
        
        self.get_logger().info("Node Initialized successfully")
        
    def turn(self, linear_speed, angular_speed):
        
        self.twist.linear.x = linear_speed
        self.twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(self.twist)
        self.last_turn_direction = 1 if angular_speed > 0 else -1  #记录转向

    def perception_callback(self, msg: PerceptionTargets):
        self.avoid_obstacle = self.judge(msg)
        
        if self.avoid_obstacle:
            self.obstacle_avoidance(msg)
            self.get_logger().info("Avoiding")
            

    def line_follow_callback(self, msg):
       
        if not self.avoid_obstacle:
            self.line_follow(msg)
            self.get_logger().info("Following")

    def line_follow(self, msg: Point):
    
        offset = msg.line_point_x - 320
        confidence = msg.confidence
        y = msg.line_point_y
        # 添加死区特性
        if -20 < offset < 20:
            offset = 0
        

        if confidence > confidence_threshold:
            # angular_speed_follow = follow_angular_ratio * offset / 400
            angular_speed_follow = follow_angular_ratio * offset / 150 * y / 480
            self.turn(follow_linear_speed, angular_speed_follow)
            
            self.line_dropped = False
        else:
            self.line_dropped = True

            self.turn(follow_linear_speed, self.last_turn_direction * restore_angular_speed )
            self.get_logger().warn("Line lost, attempting recovery")
    # 判断循线或是避障(True避False循)
    def judge(self, msg: PerceptionTargets) -> bool:
        if not msg.targets:
            
            return False

        # max_area = 0
        # max_area_num = 0

        # for num, target in enumerate(msg.targets):
        #     area = target.rois[0].rect.height * target.rois[0].rect.width

        #     if area > max_area:
        #         max_area = area
        #         max_area_num = num
        
        target = max(msg.targets, key=lambda t: t.rois[0].rect.height * t.rois[0].rect.width) #
        
        roi = target.rois[0]
        y_bottom = float(roi.rect.y_offset + roi.rect.height)
        confidence = roi.confidence

        return y_bottom > y_bottom_threshold and confidence > confidence_threshold


    def obstacle_avoidance(self, msg):
        
        target = max(msg.targets, key=lambda t: t.rois[0].rect.height * t.rois[0].rect.width)
        roi = target.rois[0]
        x_center = float(roi.rect.x_offset + roi.rect.width / 2)
        
        # if self.judge(msg):
        temp = x_center - 320.0

        if 0 <= temp < 20:
            temp = 20
        elif -20 < temp < 0:
            temp = -20

        angular_speed_avoid = avoid_angular_ratio * 700 / temp
        
        self.turn(avoid_linear_speed, angular_speed_avoid)



def main(args=None):
    rclpy.init(args=args)
    racing_control = TargetSubscriber()

    try:
        rclpy.spin(racing_control)
    except KeyboardInterrupt:
        racing_control.get_logger().warn('Avoidance exit.')
    finally:
        racing_control.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
