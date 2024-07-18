import rclpy 
from rclpy.node import Node 
from ai_msgs.msg import PerceptionTargets 
 
class MinimalSubscriber(Node): 
    def __init__(self): 
        super().__init__('minimal_subscriber') 
        self.subscription = self.create_subscription( 
            PerceptionTargets, 
            'hobot_dnn_detection',  
            self.listener_callback, 
            10) 
        self.subscription  # prevent unused variable warning 
        self.count = 0

    def listener_callback(self, msg): 
        # 回调函数，每帧推理结果产生后就会运行这个函数
        # 这里给出解析的方法
        print("\n \033[31m---\033[0m This Frame: FPS = %d  \033[31m---\033[0m"%msg.fps)
        for num, target in enumerate(msg.targets):
            print("Traget \033[0;32;40m%d\033[0m: "%num, end="")
            print("Type: %s, x_offset=%d, y_offset=%d, height=%d, width=%d, conf=%.2f"%(target.rois[0].type, 
            target.rois[0].rect.x_offset,
            target.rois[0].rect.y_offset,
            target.rois[0].rect.height,
            target.rois[0].rect.width,
            target.rois[0].confidence))
 
 
def main(args=None): 
    rclpy.init(args=args) 
    minimal_subscriber = MinimalSubscriber() 
    rclpy.spin(minimal_subscriber) 
    minimal_subscriber.destroy_node() 
    rclpy.shutdown() 

# 给main函数一个入口，省得colcon build编译
if __name__ == '__main__': 
    main()