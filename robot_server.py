import asyncio
import json
import random
import datetime
import websockets

# Sample robot data generator
class RobotDataGenerator:
    def __init__(self):
        self.robot_id = "ROB-00221"
        self.robot_name = "Mobile Explorer Bot"
        self.linear_velocity = 15.0
        self.angular_velocity = 45.0
        self.battery_percentage = 75
        self.battery_voltage = 12.3
        self.battery_charging = True
        self.battery_temperature = 28.5
        self.network_status = 4  # bars (1-5)
        # New network information
        self.ip_address = "192.168.43." + str(random.randint(100, 250))
        self.connection_types = ["WiFi", "Ethernet", "Cellular", "Bluetooth"]
        self.connection_type = "WiFi"
        self.network_latency = 15  # ms
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.logs = []
        self.latitude = 37.7749  # Default latitude (San Francisco)
        self.longitude = -122.4194  # Default longitude (San Francisco)
    
    def generate_data(self):
        # Simulate changes in robot data
        self.linear_velocity = max(0, min(30, self.linear_velocity + (random.random() - 0.5) * 2))
        self.angular_velocity = max(-90, min(90, self.angular_velocity + (random.random() - 0.5) * 5))
        
        # Battery discharge simulation (very slow)
        if not self.battery_charging and random.random() > 0.95:
            self.battery_percentage = max(1, self.battery_percentage - 1)
        elif self.battery_charging and random.random() > 0.8:
            self.battery_percentage = min(100, self.battery_percentage + 1)
        
        # Randomly toggle charging state (rarely)
        if random.random() > 0.97:
            self.battery_charging = not self.battery_charging
            
        self.battery_voltage = 10 + (self.battery_percentage / 100 * 4) + (random.random() * 0.2 - 0.1)
        self.battery_temperature = max(20, min(40, self.battery_temperature + (random.random() - 0.5)))
        
        # Network status sometimes changes
        if random.random() > 0.9:
            self.network_status = max(1, min(5, self.network_status + (1 if random.random() > 0.5 else -1)))
            
            # Network latency changes based on network status
            base_latency = 10 * (6 - self.network_status)  # Better signal (higher status) = lower latency
            jitter = random.randint(-5, 15)
            self.network_latency = max(2, base_latency + jitter)
        
        # Occasionally change connection type (very rarely)
        if random.random() > 0.98:
            self.connection_type = random.choice(self.connection_types)
            # If connection type changes, IP might change too
            if self.connection_type == "WiFi":
                self.ip_address = "192.168.43." + str(random.randint(100, 250))
            elif self.connection_type == "Ethernet":
                self.ip_address = "10.0.0." + str(random.randint(10, 250))
            elif self.connection_type == "Cellular":
                self.ip_address = "172.20." + str(random.randint(10, 250)) + "." + str(random.randint(2, 250))
            else:  # Bluetooth
                self.ip_address = "192.168.44." + str(random.randint(10, 250))
        
        # Orientation changes
        self.pitch = max(-30, min(30, self.pitch + (random.random() - 0.5) * 5))
        self.roll = max(-30, min(30, self.roll + (random.random() - 0.5) * 5))
        self.yaw = (self.yaw + random.random() * 3) % 360
        
        # Occasionally generate log messages
        if random.random() > 0.85:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            severity = random.choice(["Info", "Warning", "Error"])
            messages = {
                "Info": [
                    "System check successful",
                    "Navigation path updated",
                    "Sensor calibration complete",
                    "Scheduled maintenance due in 7 days",
                    "Camera feed active"
                ],
                "Warning": [
                    "Battery level below 30%",
                    "CPU temperature high",
                    "Network signal weak",
                    "Obstacle detected in path",
                    "Route deviation detected"
                ],
                "Error": [
                    "Sensor failure detected",
                    "Motor encoder error",
                    "Communication timeout",
                    "Navigation system failure",
                    "Hardware fault detected"
                ]
            }
            message = random.choice(messages[severity])
            self.logs.append({"timestamp": timestamp, "severity": severity, "message": message})
            
        # Only keep the last 5 logs to send (to prevent the message from getting too large)
        send_logs = self.logs[-5:] if self.logs else []
        
        # New part: simulate sensor statuses
        sensor_statuses = ["OK", "Warning", "Fail"]
        sensors = {
            "lidar": random.choice(sensor_statuses),
            "camera": random.choice(sensor_statuses),
            "encoder": random.choice(sensor_statuses),
            "imu": random.choice(sensor_statuses)
        }
        
        # Simulate location changes
        self.latitude += (random.random() - 0.5) * 0.0001  # Small random movement
        self.longitude += (random.random() - 0.5) * 0.0001  # Small random movement

        # Construct the data packet
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
            "camera_feed": "https://picsum.photos/800/450?random=" + str(random.randint(1, 1000)),
            "sensors": sensors,
            "location": {
                "latitude": self.latitude,
                "longitude": self.longitude
            }
        }
        
        return data

# WebSocket handler
async def robot_data_handler(websocket):
    robot = RobotDataGenerator()
    
    try:
        while True:
            data = robot.generate_data()
            await websocket.send(json.dumps(data))
            await asyncio.sleep(0.5)  # Send data every 1 second
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")

# Start the WebSocket server
async def main():
    server = await websockets.serve(
        robot_data_handler,
        "localhost",
        8765
    )
    
    print("WebSocket server started at ws://localhost:8765")
    await server.wait_closed()

if __name__ == "__main__":
    print("Starting robot data simulation server...")
    print("Press Ctrl+C to stop")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped")
