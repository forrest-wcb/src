import rclpy 
from rclpy.node import Node 
from ai_msgs.msg import PerceptionTargets 
from geometry_msgs.msg import Twist
from origincar_msg.msg import Point
 

#参数设定
y_bottom_threshold = 350.0 #避障阈值
confidence_threshold = 0.5 #置信度阈值

follow_angular_ratio = -1.0 #巡线速度变化系数
avoid_angular_ratio = 1.0 #避障角速度变化系数

follow_linear_speed = 0.5 #循线速度
avoid_linear_speed = 0.5 #避障线速度



class TargetSubscriber(Node): 
    def __init__(self): 
        super().__init__('racing_control') 
        self.subscription = self.create_subscription( 
            PerceptionTargets, 
            'hobot_dnn_detection',  
            self.obstacle_avoidance, 
            10) 
        
        self.subscription = self.create_subscription( 
            Point, 
            'line_point_detection',  
            self.line_follow, 
            10)
         
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10)
        self.twist = Twist()
        self.get_logger().info("Node Initialized successfully")

    def line_follow(self,msg):
        
        offset = msg.line_point_x - 320
        p = msg.confidence
        
        #添加死区特性
        if -20 < offset < 20:
            offset = 0

        
        self.twist.linear.x = follow_linear_speed
        self.twist.angular.z = follow_angular_ratio * offset / 150 * msg.line_point_y / 224
               
        self.cmd_vel_pub.publish(self.twist)    
            

    #判断循线或是避障(True避False循)
    def judge(self, y_bottom, y_bottom_threshold, confidence, confidence_threshold):
        
        if y_bottom > y_bottom_threshold and confidence> confidence_threshold:
            return True
        else:
            self.get_logger().info("Not avoiding")
            return False
    
    def obstacle_avoidance(self, msg): 
                
        # print(f"\n \033[31m---\033[0m This Frame: FPS = {msg.fps}  \033[31m---\033[0m")
        # for num, target in enumerate(msg.targets):
        #     print(f"Target \033[0;32;40m {num} \033[0m: ", end="")
        #     print("Type: %s, x_offset=%d, y_offset=%d, height=%d, width=%d, conf=%.2f"%(target.rois[0].type, 
        #     target.rois[0].rect.x_offset,
        #     target.rois[0].rect.y_offset,
        #     target.rois[0].rect.height,
        #     target.rois[0].rect.width,
        #     target.rois[0].confidence))
    
        #判断是否识别到
        if not msg.targets:
            self.get_logger().info("No Obstacle")
            return
        
        #选取最近的目标(面积优先)
        max_area = 0
        max_area_num = 0

        
        for num, target in enumerate(msg.targets):
        
            area = target.rois[0].rect.height * target.rois[0].rect.width
        
            if area > max_area:
                
                max_area = area
                max_area_num = num
        # 返回最大面积对应的target和num
          
           
        #获取中心点x坐标
        x_center = float(msg.targets[max_area_num].rois[0].rect.x_offset + msg.targets[max_area_num].rois[0].rect.width/2)
        
        #获取底部y坐标
        y_bottom = float(msg.targets[max_area_num].rois[0].rect.y_offset + msg.targets[max_area_num].rois[0].rect.height)
        
        #获取置信度
        confidence = msg.targets[max_area_num].rois[0].confidence
        
        #判断是否进行避障
        
        if self.judge(y_bottom, y_bottom_threshold, confidence, confidence_threshold):
        
            #计算误差
            temp = x_center - 320.0
            
            # 添加饱和特性
            if temp >= 0 and temp < 20:
                temp = 20
            elif temp < 0 and temp > -20:
                temp = -20
                
            #计算角速度
            self.twist.angular.z = avoid_angular_ratio * 700 / temp
            self.twist.linear.x = avoid_linear_speed
            
            self.cmd_vel_pub.publish(self.twist)        
            
            self.get_logger().info("Avoiding")
        
        
            
 
 
def main(args=None): 
    rclpy.init(args=args) 
    racing_control = TargetSubscriber() 
    
    try:
        rclpy.spin(racing_control)
    except KeyboardInterrupt:
        racing_control.get_logger().warn('Avoidence exit.')
    finally:
        racing_control.destroy_node()
    
    rclpy.shutdown() 


if __name__ == '__main__': 
    main()
    
    


