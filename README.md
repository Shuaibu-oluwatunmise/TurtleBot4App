
# 📱 TurtleBot4 Mobile Companion App

This is a **React Native app** built to serve as a **remote companion UI** for the TurtleBot4 running ROS 2 Jazzy. The app enables live control, monitoring, and feedback from your robot using modern mobile UX and WebSocket-based ROS communication.

---

## 🚀 Features

### 🎮 Teleoperation
- **Joystick Mode** – smooth analog-style control with real-time feedback
- **Button Pad Mode** – simple directional buttons for easy navigation
- Adjustable **speed slider** for precise control
- Live **velocity telemetry** from the robot

### 🔋 Battery Monitoring
- Current battery percentage shown live
- Historical battery status display
- Intuitive and beautiful UI

### 🌐 WebSocket Connectivity
- Real-time ROS communication using `rosbridge_suite`
- Modular structure ready for more ROS topics (e.g., camera feed, SLAM)

---

## 🛠️ Setup Instructions

### 📦 Prerequisites

- ROS 2 Jazzy running on the TurtleBot4 or compatible VM
- `rosbridge_suite` installed and running:
```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

- Mobile device (or emulator) with internet access to same network as robot
- React Native environment set up (Expo or Bare Workflow)

---

## 🔧 Installation & Running

1. Clone this repo:
```bash
git clone <your-app-repo-url>
cd turtlebot4-mobile-app
```

2. Install dependencies:
```bash
npm install
```

3. Start the app:
```bash
npx expo start
```

4. Enter your TurtleBot IP and port (e.g., `ws://192.168.1.100:9090`) in the connection screen.

> 📝 **NOTE:** The robot must be running `rosbridge_websocket` and be accessible over the same network.

---

## 🧪 Under Development

The app is still under heavy development. Current roadmap:

- 🗺️ **Live Map Creation**  
  Real-time drawing of map using `/map` topic and React Canvas  
  Python backend planned for receiving and parsing occupancy grids

- 📷 **Camera Integration**  
  Stream image topic into the app for visual feedback

- 🧠 **Voice Trigger & SLAM Management**  
  Trigger voice nav and monitor SLAM status remotely

---

## ⚠️ Developer Notes

To automate socket setup, consider editing the robot’s startup scripts to auto-launch rosbridge on boot:

```bash
echo "ros2 launch rosbridge_server rosbridge_websocket_launch.xml" >> ~/.bashrc
```

---

## 🌟 Future Enhancements

- Tap-to-Navigate on map
- Web Dashboard trigger sync
- Natural Language processing for complex commands
- Deep linking to ROS tools (e.g., launch SLAM remotely)

---

## 📬 Feedback

Pull requests and contributions are welcome!  
Please open an issue if you'd like to suggest a feature or report a bug.

---

**Made with ❤️ for Mobile Robotics & ROS.**
