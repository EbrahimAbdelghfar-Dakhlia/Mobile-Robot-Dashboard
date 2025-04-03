import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime

class LogPublisher(Node):
    def __init__(self):
        super().__init__('log_publisher')
        self.publisher = self.create_publisher(String, 'robot/logs', 10)
        self.timer = self.create_timer(2.0, self.publish_log)  # Publish every 2 seconds

    def publish_log(self):
        log_entry = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "severity": "Info",  # Change to "Warning" or "Error" as needed
            "message": "This is a test log message from the ROS 2 publisher."
        }
        msg = String()
        msg.data = json.dumps(log_entry)
        print(f"Publishing log: {msg.data}")
        self.publisher.publish(msg)
        self.get_logger().info(f"Published log: {log_entry}")

def main(args=None):
    rclpy.init(args=args)
    node = LogPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
