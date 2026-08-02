"""Serve the dashboard static files over HTTP.

The dashboard talks to ROS through rosbridge (port 9090) and shows the
camera streams served by web_video_server (port 8080). This node only
serves the static HTML/JS/CSS on port 8000.
"""

import functools
import http.server
import os
import socketserver
import threading

import rclpy
from rclpy.node import Node

WEB_DIR = os.path.join(os.path.dirname(__file__), "..", "web")


class DashboardServer(Node):

    def __init__(self):
        super().__init__("serve_dashboard")
        self.declare_parameter("port", 8000)
        port = self.get_parameter("port").value
        web_dir = os.path.realpath(
            os.path.join(os.path.dirname(__file__), "..", "web"))
        handler = functools.partial(
            http.server.SimpleHTTPRequestHandler, directory=web_dir)
        socketserver.ThreadingTCPServer.allow_reuse_address = True
        self._httpd = socketserver.ThreadingTCPServer(("0.0.0.0", port),
                                                      handler)
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"dashboard on http://localhost:{port}/")

    def __del__(self):
        try:
            self._httpd.shutdown()
        except Exception:
            pass


def main():
    rclpy.init()
    node = DashboardServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
