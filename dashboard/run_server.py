import rclpy
from rclpy.node import Node
import http.server
import socketserver
import os
import threading
import webbrowser  # Import the webbrowser module

# Define the port and directory to serve
PORT = 8000
DIRECTORY = os.path.dirname(os.path.abspath(__file__))

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.server_thread = None

    def start_server(self):
        os.chdir(DIRECTORY)
        handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(*args, directory=DIRECTORY, **kwargs)
        self.httpd = socketserver.TCPServer(("", PORT), handler)
        self.get_logger().info(f"Serving at http://localhost:{PORT}")
        webbrowser.open(f"http://localhost:{PORT}/robot_dashboard.html")  # Open the dashboard in the browser
        self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.server_thread.start()

    def stop_server(self):
        if self.httpd:
            self.httpd.shutdown()
            self.httpd.server_close()
            self.get_logger().info("Web server stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        node.start_server()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_server()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
