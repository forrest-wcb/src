import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # 使用标准消息类型
from origincar_msg.msg import Sign
class ListenerNode(Node):

    def __init__(self):
        super().__init__('racing_state')
        self.subscription = self.create_subscription(
            Int32,  # 消息类型
            '/sign4return',  # 订阅的话题名
            self.listener_callback,
            10  # 队列大小
        )
        #self.state_publisher = self.create_publisher(Int32, 'sign_foxglove', 10)
        self.foxglove_state_publisher = self.create_publisher(Sign, '/sign_foxglove', 10)
        self.car_state_publisher = self.create_publisher(Sign, "/sign_switch", 10)

    def listener_callback(self, msg:Int32):
        
        state = Sign()
        
        state.sign_data = msg.data
        self.foxglove_state_publisher.publish(state)
        # if msg.data == 5:
        #     #state.data = 5
        #     self.get_logger().info('开始遥测')
        #     self.state_publisher.publish(msg)
        # elif msg.data == 6:
        #     self.get_logger().info('结束遥测')
        #     self.state_publisher.publish(msg)
            
        self.get_logger().info(f'上位机发布了 "{msg.data}"')
        self.get_logger().info(f'发布到信号: {state.sign_data}')
        


def main(args=None):
    rclpy.init(args=args)
    listener_node = ListenerNode()
    rclpy.spin(listener_node)

    # 显式销毁节点
    listener_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
