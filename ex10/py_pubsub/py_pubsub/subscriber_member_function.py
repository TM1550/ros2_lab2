import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class TextToCmdVel(Node):

    def __init__(self):
        super().__init__('text_to_cmd_vel')
        
        # Подписка на текстовые команды
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.command_callback,
            10
        )
        
        # Публикация команд скорости для черепахи
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.get_logger().info('Text to cmd_vel converter ready')
        self.get_logger().info('Waiting for commands: "turn_right", "turn_left", "move_forward", "move_backward"')

    def command_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received command: "{command}"')
        
        twist_msg = Twist()
        
        if command == 'move_forward':
            twist_msg.linear.x = 1.0  # 1 м/с вперед
        elif command == 'move_backward':
            twist_msg.linear.x = -1.0  # 1 м/с назад
        elif command == 'turn_right':
            twist_msg.angular.z = -1.5  # 1.5 рад/с по часовой стрелке
        elif command == 'turn_left':
            twist_msg.angular.z = 1.5  # 1.5 рад/с против часовой стрелки
        else:
            self.get_logger().warn(f'Invalid command: "{command}"')
            return
        
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Published Twist command: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    text_to_cmd_vel = TextToCmdVel()
    
    try:
        rclpy.spin(text_to_cmd_vel)
    except KeyboardInterrupt:
        pass
    finally:
        text_to_cmd_vel.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()