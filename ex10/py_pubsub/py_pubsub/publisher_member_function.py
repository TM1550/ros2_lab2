import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys


class KeyboardMovementPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_movement_publisher')
        self.publisher_ = self.create_publisher(String, 'cmd_text', 10)
        
        self.get_logger().info('Keyboard movement publisher ready')
        self.get_logger().info('Available commands: "turn_right", "turn_left", "move_forward", "move_backward"')
        self.get_logger().info('Type commands and press Enter (or type "exit" to quit)')
        
        # Запускаем поток для чтения ввода с клавиатуры
        self.keyboard_thread = threading.Thread(target=self.read_keyboard_input)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def read_keyboard_input(self):
        while True:
            try:
                command = input().strip().lower()
                
                if command == 'exit':
                    rclpy.shutdown()
                    sys.exit(0)
                
                valid_commands = ['turn_right', 'turn_left', 'move_forward', 'move_backward']
                
                if command in valid_commands:
                    msg = String()
                    msg.data = command
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing command: "{command}"')
                else:
                    self.get_logger().warn(f'Invalid command: "{command}". Valid commands: {valid_commands}')
                    
            except EOFError:
                break
            except Exception as e:
                self.get_logger().error(f'Error reading input: {e}')


def main(args=None):
    rclpy.init(args=args)

    keyboard_publisher = KeyboardMovementPublisher()

    try:
        rclpy.spin(keyboard_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()