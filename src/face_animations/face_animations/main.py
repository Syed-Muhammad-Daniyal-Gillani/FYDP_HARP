import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import websockets
import threading
import webbrowser
import os

package_path = os.path.expanduser("~/ROS_FYDP/src/face_animations/face_animations")


class FaceAnimationNode(Node):
    def __init__(self):
        super().__init__('face_animation_node')
        self.subscription = self.create_subscription(
            String,
            '/user_emotions',
            self.emotion_callback,
            10)
        self.subscription
        self.current_emotion = "neutral"
        self.client_sockets = set()  # Renamed from self.clients

        # Start WebSocket Server in Background
        threading.Thread(target=self.start_websocket_server, daemon=True).start()

        # Open the browser with local HTML file
        webbrowser.open(f"file://{package_path}/index.html")

    def emotion_callback(self, msg):
        emotion = msg.data.lower().strip()
        valid_emotions = ["happy", "sad", "angry", "focused", "confused"]

        if emotion in valid_emotions and emotion != self.current_emotion:
            self.current_emotion = emotion
            self.send_emotion_to_clients(emotion)

    def send_emotion_to_clients(self, emotion):
        for client in self.client_sockets:  # Updated variable name
            asyncio.run(client.send(emotion))

    async def websocket_handler(self, websocket, path):
        self.client_sockets.add(websocket)  # Updated variable name
        try:
            async for message in websocket:
                pass  # No messages expected from the browser
        finally:
            self.client_sockets.remove(websocket)  # Updated variable name

    def start_websocket_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        server = websockets.serve(self.websocket_handler, "localhost", 8765)
        loop.run_until_complete(server)
        loop.run_forever()

def main(args=None):
    rclpy.init(args=args)
    node = FaceAnimationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
