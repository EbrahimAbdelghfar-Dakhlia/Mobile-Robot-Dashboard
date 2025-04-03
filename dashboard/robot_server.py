import asyncio
import json
import datetime
import websockets
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32, Int32, Bool
from std_msgs.msg import Float32MultiArray, ByteMultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import base64
import cv2
import threading
import sys
import signal

# Robot data generator
class RobotDataGenerator:
    def __init__(self):
        self.robot_id = "ROB-00221"
        self.robot_name = "Mobile Explorer Bot"
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.battery_percentage = 0
        self.battery_voltage = 0.0
        self.battery_charging = False
        self.battery_temperature = 0.0
        self.network_status = 0  # bars (1-5)
        self.ip_address = "0.0.0.0"  # Default IP address
        self.connection_type = "Unknown"
        self.network_latency = 0  # ms
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.logs = []
        self.latitude = 0.0  # Default latitude
        self.longitude = 0.0  # Default longitude
        self.camera_feed_base64 = None
        self.sensors = {
            "lidar": "Unknown",
            "camera": "Unknown",
            "encoder": "Unknown",
            "imu": "Unknown"
        }

    def generate_data(self):
        # Use the current values for all fields (no random modifications)
        send_logs = self.logs[-5:] if self.logs else []
        
        data = {
            "linear_velocity": self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "battery": {
                "percentage": self.battery_percentage,
                "voltage": self.battery_voltage,
                "charging": self.battery_charging,
                "temperature": self.battery_temperature
            },
            "logs": send_logs,
            "network_status": self.network_status,
            "network_info": {
                "name": self.robot_name,
                "ip": self.ip_address,
                "type": self.connection_type,
                "latency": f"{self.network_latency}ms"
            },
            "robot_id": self.robot_id,
            "imu": {"pitch": self.pitch, "roll": self.roll, "yaw": self.yaw},
            "sensors": self.sensors,
            "location": {
                "latitude": self.latitude,
                "longitude": self.longitude
            }
        }
        
        # Add camera feed if available
        if self.camera_feed_base64:
            data["camera_feed"] = f"data:image/jpeg;base64,{self.camera_feed_base64}"
        else:
            data["camera_feed"] = None  # No default random image
        
        return data

# ROS2 Node for subscribing to topics
class RobotDataSubscriber(Node):
    def __init__(self, robot_data):
        super().__init__('robot_dashboard_subscriber')
        self.robot_data = robot_data
        self.bridge = CvBridge()

        # Subscriptions for robot data
        self.create_subscription(String, 'robot/id', self.update_robot_id, 10)
        self.create_subscription(String, 'robot/name', self.update_robot_name, 10)
        self.create_subscription(Float32, 'robot/linear_velocity', self.update_linear_velocity, 10)
        self.create_subscription(Float32, 'robot/angular_velocity', self.update_angular_velocity, 10)
        self.create_subscription(Int32, 'robot/battery/percentage', self.update_battery_percentage, 10)
        self.create_subscription(Float32, 'robot/battery/voltage', self.update_battery_voltage, 10)
        self.create_subscription(Bool, 'robot/battery/charging', self.update_battery_charging, 10)
        self.create_subscription(Float32, 'robot/battery/temperature', self.update_battery_temperature, 10)
        self.create_subscription(Int32, 'robot/network/status', self.update_network_status, 10)
        self.create_subscription(String, 'robot/network/ip_address', self.update_ip_address, 10)
        self.create_subscription(String, 'robot/network/connection_type', self.update_connection_type, 10)
        self.create_subscription(Int32, 'robot/network/latency', self.update_network_latency, 10)
        self.create_subscription(Float32, 'robot/imu/pitch', self.update_pitch, 10)
        self.create_subscription(Float32, 'robot/imu/roll', self.update_roll, 10)
        self.create_subscription(Float32, 'robot/imu/yaw', self.update_yaw, 10)
        self.create_subscription(Float32, 'robot/location/latitude', self.update_latitude, 10)
        self.create_subscription(Float32, 'robot/location/longitude', self.update_longitude, 10)
        self.create_subscription(Image, 'robot/camera/image_raw', self.update_camera_feed, 10)
        self.create_subscription(String, 'robot/logs', self.update_logs, 10)
        self.create_subscription(String, 'robot/sensors/lidar', self.update_lidar_status, 10)
        self.create_subscription(String, 'robot/sensors/camera', self.update_camera_status, 10)
        self.create_subscription(String, 'robot/sensors/encoder', self.update_encoder_status, 10)
        self.create_subscription(String, 'robot/sensors/imu', self.update_imu_status, 10)

        self.get_logger().info('ROS2 subscribers initialized for robot dashboard')

    # Callback methods for each topic
    def update_robot_id(self, msg):
        self.robot_data.robot_id = msg.data
        self.get_logger().info(f"Received robot_id: {msg.data}")

    def update_robot_name(self, msg):
        self.robot_data.robot_name = msg.data
        self.get_logger().info(f"Received robot_name: {msg.data}")

    def update_linear_velocity(self, msg):
        self.robot_data.linear_velocity = msg.data
        self.get_logger().info(f"Received linear_velocity: {msg.data}")

    def update_angular_velocity(self, msg):
        self.robot_data.angular_velocity = msg.data
        self.get_logger().info(f"Received angular_velocity: {msg.data}")

    def update_battery_percentage(self, msg):
        self.robot_data.battery_percentage = msg.data
        self.get_logger().info(f"Received battery_percentage: {msg.data}")

    def update_battery_voltage(self, msg):
        self.robot_data.battery_voltage = msg.data
        self.get_logger().info(f"Received battery_voltage: {msg.data}")

    def update_battery_charging(self, msg):
        self.robot_data.battery_charging = msg.data
        self.get_logger().info(f"Received battery_charging: {msg.data}")

    def update_battery_temperature(self, msg):
        self.robot_data.battery_temperature = msg.data
        self.get_logger().info(f"Received battery_temperature: {msg.data}")

    def update_network_status(self, msg):
        self.robot_data.network_status = msg.data
        self.get_logger().info(f"Received network_status: {msg.data}")

    def update_ip_address(self, msg):
        self.robot_data.ip_address = msg.data
        self.get_logger().info(f"Received ip_address: {msg.data}")

    def update_connection_type(self, msg):
        self.robot_data.connection_type = msg.data
        self.get_logger().info(f"Received connection_type: {msg.data}")

    def update_network_latency(self, msg):
        self.robot_data.network_latency = msg.data
        self.get_logger().info(f"Received network_latency: {msg.data}")

    def update_pitch(self, msg):
        self.robot_data.pitch = msg.data
        self.get_logger().info(f"Received pitch: {msg.data}")

    def update_roll(self, msg):
        self.robot_data.roll = msg.data
        self.get_logger().info(f"Received roll: {msg.data}")

    def update_yaw(self, msg):
        self.robot_data.yaw = msg.data
        self.get_logger().info(f"Received yaw: {msg.data}")

    def update_latitude(self, msg):
        self.robot_data.latitude = msg.data
        self.get_logger().info(f"Received latitude: {msg.data}")

    def update_longitude(self, msg):
        self.robot_data.longitude = msg.data
        self.get_logger().info(f"Received longitude: {msg.data}")

    def update_camera_feed(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image)
            self.robot_data.camera_feed_base64 = base64.b64encode(buffer).decode('utf-8')
            self.get_logger().info("Received camera feed")
        except Exception as e:
            self.get_logger().error(f"Error processing camera feed: {str(e)}")

    def update_logs(self, msg):
        try:
            log_entry = json.loads(msg.data)
            if isinstance(log_entry, dict) and 'timestamp' in log_entry and 'severity' in log_entry and 'message' in log_entry:
                self.robot_data.logs.append(log_entry)
                if len(self.robot_data.logs) > 100:
                    self.robot_data.logs = self.robot_data.logs[-100:]
                self.get_logger().info(f"Received log entry: {log_entry}")
        except Exception as e:
            self.get_logger().error(f"Error processing log entry: {str(e)}")
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.robot_data.logs.append({
                "timestamp": timestamp,
                "severity": "Info",
                "message": msg.data
            })

    def update_lidar_status(self, msg):
        self.robot_data.sensors["lidar"] = msg.data
        self.get_logger().info(f"Received lidar status: {msg.data}")

    def update_camera_status(self, msg):
        self.robot_data.sensors["camera"] = msg.data
        self.get_logger().info(f"Received camera status: {msg.data}")

    def update_encoder_status(self, msg):
        self.robot_data.sensors["encoder"] = msg.data
        self.get_logger().info(f"Received encoder status: {msg.data}")

    def update_imu_status(self, msg):
        self.robot_data.sensors["imu"] = msg.data
        self.get_logger().info(f"Received imu status: {msg.data}")

# WebSocket handler
async def robot_data_handler(websocket, robot_data):
    try:
        while True:
            data = robot_data.generate_data()
            await websocket.send(json.dumps(data))
            await asyncio.sleep(0.01)
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")

# Create a class that manages both the ROS2 node and WebSocket server
class RobotDashboardServer:
    def __init__(self):
        # Initialize ROS2
        rclpy.init(args=None)
        
        # Create robot data object
        self.robot_data = RobotDataGenerator()
        
        # Create ROS2 node
        self.ros_node = RobotDataSubscriber(self.robot_data)
        
        # Create executor for ROS2
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.ros_node)
        
        # Flags for graceful shutdown
        self.running = True
        self.ws_server = None
        self.ros_thread = None
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        print(f"\nReceived signal {sig}, shutting down...")
        self.running = False
        if self.ws_server:
            self.ws_server.close()
        sys.exit(0)
    
    def run_ros2(self):
        """Run ROS2 node in a separate thread"""
        try:
            print("ROS2 node started")
            while self.running and rclpy.ok():
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            print(f"Error in ROS2 thread: {e}")
        finally:
            print("ROS2 thread ending")
    
    async def run_websocket_server(self):
        """Run the WebSocket server"""
        self.ws_server = await websockets.serve(
            lambda ws, path: robot_data_handler(ws, self.robot_data),
            "localhost", 
            8765
        )
        print("WebSocket server started at ws://localhost:8765")
        
        # Keep the server running until shutdown
        while self.running:
            await asyncio.sleep(0.1)
    
    def start(self):
        """Start the entire system"""
        try:
            # Start ROS2 thread
            self.ros_thread = threading.Thread(target=self.run_ros2)
            self.ros_thread.daemon = True
            self.ros_thread.start()
            
            # Start WebSocket server in the main thread
            print("Starting WebSocket server...")
            asyncio.run(self.run_websocket_server())
        except KeyboardInterrupt:
            print("\nShutdown requested by KeyboardInterrupt")
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Shutdown everything gracefully"""
        self.running = False
        
        # Shutdown ROS2
        if hasattr(self, 'ros_node'):
            self.ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
        # Wait for ROS thread if needed
        if self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0)
        
        print("Robot Dashboard Server shutdown complete")

# Main entry point
def main():
    print("Starting Robot Dashboard Server with ROS2 integration...")
    server = RobotDashboardServer()
    server.start()

if __name__ == "__main__":
    main()
