import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraStreamPublisher(Node):
    def __init__(self):
        super().__init__('camera_stream_publisher')
        self.publisher = self.create_publisher(Image, 'robot/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.01, self.publish_camera_frame)  # Publish at 10 Hz

        # Open the default camera (index 0)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open the camera.")
            raise RuntimeError("Failed to open the camera.")

    def publish_camera_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from the camera.")
            return

        # Convert the frame to a ROS2 Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info("Published a camera frame.")

    def destroy_node(self):
        # Release the camera resource when shutting down
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
