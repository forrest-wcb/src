import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('ArtagTrack')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)#msg:原始坐标信息或者坐标中心点
        self.subscription  # prevent unused variable warning
        self.last_pose_error = 0

    def listener_callback(self, msg):
        pass
#    #****************处理msg获取中心坐标点***************#
#         Middle_point_x = 
#         Middle_point_y = 
#    #****************根据图像尺寸大小选择图像中心点坐标***************#
#         Targer_point_x =
#         Target_point_y = 
#         pos_error = Targer_point_x - Middle_point_x
#         Kp = 2.5
#         Ki = 0
#         Kd = 0
#         turn_anger = Kp * pos_error + Kd *(pos_error-self.last_pose_error)#计算转角
#         self.last_pose_error = pos_error
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
