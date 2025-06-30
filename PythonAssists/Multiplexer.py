import asyncio
import json
import websockets
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import BatteryState, JointState, CompressedImage
from geometry_msgs.msg import TwistStamped
from rosidl_runtime_py import message_to_ordereddict
import subprocess
import os
import signal
import pathlib
import time
from PIL import Image
import numpy as np
import base64
import io
from datetime import datetime
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose



active_clients = {}
latest_map = {"msg": None}
slam_proc = None
nav2_proc = None
explorer_proc = None
EXPLORER_SCRIPT = str(pathlib.Path(_file_).parent / "frontier_explorer.py")
param_path = os.path.abspath("mynav2.yaml")

def wait_for_nav2_readiness(timeout_sec=30.0):
    from rclpy.executors import SingleThreadedExecutor

    nav2_node = rclpy.create_node("wait_for_nav2_node")
    client = ActionClient(nav2_node, NavigateToPose, "navigate_to_pose")

    print("⏳ Waiting for Nav2 action server (navigate_to_pose)...")
    start_time = time.time()

    while time.time() - start_time < timeout_sec:
        rclpy.spin_once(nav2_node, timeout_sec=1.0)
        if client.wait_for_server(timeout_sec=1.0):
            print("✅ Nav2 action server ready")
            nav2_node.destroy_node()
            return True
        print("…still waiting")

    print("❌ Timeout: Nav2 action server not ready")
    nav2_node.destroy_node()
    return False

def start_auto_map():
    global slam_proc, nav2_proc, explorer_proc
    try:
        print("\U0001F680 Starting Autonomous Mapping Workflow")
        if not slam_proc or slam_proc.poll() is not None:
            slam_proc = subprocess.Popen([
                "ros2", "launch", "turtlebot4_navigation", "slam.launch.py"
            ])
            print("✅ SLAM launched")

        time.sleep(3)  # Delay for SLAM to initialize

        if not nav2_proc or nav2_proc.poll() is not None:
            nav2_proc = subprocess.Popen([
                "ros2", "launch", "turtlebot4_navigation", "nav2.launch.py", f"params_file:={param_path}"
            ])
            print("✅ Navigation stack launched")

        if wait_for_nav2_readiness():
            if not explorer_proc or explorer_proc.poll() is not None:
                explorer_proc = subprocess.Popen(["python3", EXPLORER_SCRIPT])
                print("✅ Explorer launched")
        else:
            print("❌ Skipping Explorer launch: Nav2 not ready")

    except Exception as e:
        print(f"❌ Failed to start auto mapping processes: {e}")


def make_auto_map_handler():
    async def handler(websocket):
        global slam_proc, nav2_proc, explorer_proc
        print("\U0001F7E2 Auto mapping client connected")
        try:
            async for message in websocket:
                if message == "Auto_Map":
                    start_auto_map()
                    await websocket.send(json.dumps({"type": "auto_map_ack", "status": "started"}))
                elif message == "stop_auto_map":
                    print("\U0001F6D1 Stopping autonomous mapping processes...")
                    for proc, name in [(slam_proc, "SLAM"), (nav2_proc, "Nav2"), (explorer_proc, "Explorer")]:
                        if proc and proc.poll() is None:
                            proc.send_signal(signal.SIGINT)
                            proc.wait()
                            print(f"\U0001F6D1 {name} stopped.")
                    slam_proc = nav2_proc = explorer_proc = None
                    await websocket.send(json.dumps({"type": "auto_map_ack", "status": "stopped"}))
        except Exception as e:
            print(f"❌ Auto mapping WebSocket error: {e}")
        finally:
            print("🔌 Auto mapping client disconnected")
    return handler

def make_camera_handler():
    async def handler(websocket):
        active_clients.setdefault('/camera', set()).add(websocket)
        print("📸 Camera client connected")
        try:
            await websocket.wait_closed()
        finally:
            active_clients['/camera'].remove(websocket)
            print("🔌 Camera client disconnected")
    return handler


def build_ros_message(msg_type, data: dict):
    msg = msg_type()
    for key, value in data.items():
        setattr(msg, key, value)
    return msg



async def safe_broadcast(clients, message):
    dead = set()
    for ws in clients:
        try:
            await ws.send(message)
        except Exception as e:
            print(f"❌ WebSocket send failed: {e}")
            dead.add(ws)
    for ws in dead:
        clients.remove(ws)


class MultiplexerNode(Node):
    def __init__(self, loop):
        super().__init__('ros2_multiplexer')
        print("🧐 MultiplexerNode constructed")
        self.loop = loop

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.create_subscription(OccupancyGrid, '/map', self.on_map_update, 10)
        self.create_subscription(Odometry, '/odom', self.make_callback('/odom'), 10)
        self.create_subscription(BatteryState, '/battery_state', self.make_callback('/battery_state'), 10)
        self.create_subscription(JointState, '/joint_states', self.make_callback('/joint_states'), 10)
        self.create_subscription(CompressedImage, '/oakd/rgb/preview/image_raw/compressed',self.on_camera_image, 10)

    def on_camera_image(self, msg):
        clients = active_clients.get('/camera', set())
        if not clients:
            return

        try:
            img_bytes = msg.data  # raw JPEG
            b64_image = base64.b64encode(img_bytes).decode('utf-8')

            payload = json.dumps({
                "type": "camera",
                "image": b64_image
            })

            asyncio.run_coroutine_threadsafe(
                safe_broadcast(clients, payload),
                self.loop
            )
        except Exception as e:
            self.get_logger().error(f"❌ Error in on_camera_image: {e}")

    def on_map_update(self, msg):
        latest_map["msg"] = msg
        clients = active_clients.get('/map', set())
        if not clients:
            return

        self.get_logger().info("🚁 Callback triggered on /map")
        try:
            msg_dict = message_to_ordereddict(msg)
            message = json.dumps({
                "type": "map",
                "info": msg_dict["info"],
                "image": render_map_image(msg)
            })
            self.get_logger().info(f"📤 Queued broadcast to {len(clients)} clients")
            asyncio.run_coroutine_threadsafe(
                safe_broadcast(clients, message),
                self.loop
            )
        except Exception as e:
            self.get_logger().error(f"❌ Error in /map callback: {e}")

    def make_callback(self, topic_name):
        def callback(msg):
            clients = active_clients.get(topic_name, set())
            if not clients:
                return

            self.get_logger().info(f"🚁 Callback triggered on {topic_name}")
            try:
                msg_dict = message_to_ordereddict(msg)
                message = json.dumps(msg_dict)
                self.get_logger().info(f"📤 Queued broadcast to {len(clients)} clients")
                asyncio.run_coroutine_threadsafe(
                    safe_broadcast(clients, message),
                    self.loop
                )
            except Exception as e:
                self.get_logger().error(f"❌ Error in {topic_name} callback: {e}")
        return callback

    def publish_cmd_vel(self, data):
        try:
            linear = float(data.get("linear", 0.0))
            angular = float(data.get("angular", 0.0))
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = linear
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = angular

            self.cmd_vel_pub.publish(msg)
            self.get_logger().info(f"🚀 Published /cmd_vel: linear={linear}, angular={angular}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to publish TwistStamped: {e}")


def render_map_image(map_msg):
    width = map_msg.info.width
    height = map_msg.info.height
    data = map_msg.data

    arr = np.zeros((height, width), dtype=np.uint8)
    for i, val in enumerate(data):
        if val == -1:
            arr.flat[i] = 128
        elif val == 0:
            arr.flat[i] = 255
        elif val == 100:
            arr.flat[i] = 0

    img = Image.fromarray(arr, mode='L')
    buf = io.BytesIO()
    img.save(buf, format='PNG')
    return base64.b64encode(buf.getvalue()).decode()


def handle_map_save(map_name):
    maps_dir = os.path.expanduser("~/saved_maps")
    os.makedirs(maps_dir, exist_ok=True)

    save_result = subprocess.run([
        "ros2", "service", "call", "/slam_toolbox/save_map", "slam_toolbox/srv/SaveMap",
        f"name:\n  data: '{map_name}'"
    ], capture_output=True, text=True)

    if save_result.returncode != 0:
        print("❌ Map save failed:", save_result.stderr)
        return False, None

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = map_name

    base_path = os.path.expanduser("~")
    files = [
        f"{prefix}.pgm", f"{prefix}.yaml"
    ]
    for f in files:
        src = os.path.join(base_path, f)
        dst = os.path.join(maps_dir, f"{map_name}{timestamp}_orig{f}")
        if os.path.exists(src):
            os.rename(src, dst)
            
    # ✅ Save PNG version of the current map
    if latest_map["msg"]:
        png_filename = os.path.join(maps_dir, f"{map_name}_{timestamp}.png")
        img_data = render_map_image(latest_map["msg"])
        with open(png_filename, "wb") as f:
            f.write(base64.b64decode(img_data))
            
    return True, maps_dir
    


def make_broadcast_handler(topic_name):
    async def handler(websocket):
        active_clients.setdefault(topic_name, set()).add(websocket)
        print(f"🟢 Client connected to {topic_name}")
        try:
            await websocket.wait_closed()
        finally:
            active_clients[topic_name].remove(websocket)
            print(f"🔴 Client disconnected from {topic_name}")
    return handler


def make_cmd_vel_handler(node):
    async def handler(websocket):
        async for message in websocket:
            try:
                data = json.loads(message)
                node.publish_cmd_vel(data)
            except Exception as e:
                node.get_logger().error(f"❌ WebSocket /cmd_vel error: {e}")
    return handler


def make_slam_control_handler():
    slam_process = None

    async def handler(websocket):
        nonlocal slam_process  # 👈 This fixes the scope error
        print("🟢 SLAM control client connected")
        try:
            async for message in websocket:
                print(f"⚙ SLAM command received: {message}")
                if message == "launch_slam":
                    if slam_process is None:
                        slam_process = subprocess.Popen([
                            "ros2", "launch", "turtlebot4_navigation", "slam.launch.py"
                        ])
                        print("🚀 SLAM launched.")
                    else:
                        print("⚠ SLAM already running.")
                elif message == "stop_slam":
                    if slam_process and slam_process.poll() is None:
                        slam_process.send_signal(signal.SIGINT)
                        slam_process.wait()
                        print("🛑 SLAM stopped gracefully.")
                        slam_process = None
                    else:
                        print("ℹ No active SLAM process to stop.")
                elif message.startswith("save_map:"):
                    map_name = message.split("save_map:")[1].strip()
                    success, path = handle_map_save(map_name)
                    await websocket.send(json.dumps({
                        "type": "save_map_result",
                        "success": success,
                        "path": path
                    }))
        except Exception as e:
            print(f"❌ SLAM WebSocket error: {e}")
        finally:
            print("🔴 SLAM control client disconnected")

    return handler



def make_image_broadcast_handler():
    async def handler(websocket):
        print("🖼 Image client connected")
        try:
            while True:
                if latest_map["msg"]:
                    img_b64 = render_map_image(latest_map["msg"])
                    await websocket.send(img_b64)
                await asyncio.sleep(1)
        except Exception as e:
            print("❌ Image WebSocket error:", e)
        finally:
            print("🔌 Image client disconnected")
    return handler
    
def make_saved_maps_handler():
    async def handler(websocket):
        print("📂 Saved maps client connected")
        try:
            maps_dir = os.path.expanduser("~/saved_maps")
            if not os.path.exists(maps_dir):
                await websocket.send(json.dumps([]))
                return

            images = []
            for fname in os.listdir(maps_dir):
                if fname.endswith(".png"):
                    path = os.path.join(maps_dir, fname)
                    with open(path, "rb") as f:
                        b64 = base64.b64encode(f.read()).decode()
                        images.append({
                            "filename": fname,
                            "base64": b64
                        })

            images.sort(reverse=True, key=lambda x: x["filename"])
            await websocket.send(json.dumps(images))
        except Exception as e:
            print(f"❌ Saved maps handler error: {e}")
            await websocket.send(json.dumps([]))
        finally:
            print("🔌 Saved maps client disconnected")
    return handler


async def start_websocket_servers(node):
    print("✅ Starting WebSocket servers...")
    await websockets.serve(make_broadcast_handler('/map'), '0.0.0.0', 9091)
    await websockets.serve(make_cmd_vel_handler(node), '0.0.0.0', 9092)
    await websockets.serve(make_broadcast_handler('/odom'), '0.0.0.0', 9093)
    await websockets.serve(make_broadcast_handler('/battery_state'), '0.0.0.0', 9094)
    await websockets.serve(make_broadcast_handler('/joint_states'), '0.0.0.0', 9095)
    await websockets.serve(make_slam_control_handler(), '0.0.0.0', 9099)
    await websockets.serve(make_image_broadcast_handler(), '0.0.0.0', 9096)
    await websockets.serve(make_saved_maps_handler(), '0.0.0.0', 9097)
    await websockets.serve(make_auto_map_handler(), '0.0.0.0', 9098)
    await websockets.serve(make_camera_handler(), '0.0.0.0', 9100)




async def main():
    print("✅ ROS 2 (rclpy) multiplexer bridge running...")

    loop = asyncio.get_running_loop()
    node = MultiplexerNode(loop)

    import threading
    ros_spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_spin_thread.start()

    await start_websocket_servers(node)
    await asyncio.Future()


if __name__ == '__main__':
    rclpy.init()
    asyncio.run(main())