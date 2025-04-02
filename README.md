# Gesture-Based Manipulator Control in ROS with TinyML 🤖✋

**ROS2 Jazzy Package for Kinova Gen3 Manipulator Control Using TinyML-Powered Hand Gestures**  
*Part of Undergraduate Thesis: "A Bimanual Gesture Interface for ROS-Based Mobile Manipulators Using Edge-AI and Sensor Fusion"*

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-%23C5221F)](https://docs.ros.org/en/jazzy/)
[![MoveIt2](https://img.shields.io/badge/MoveIt2-Project-%23C5221F)](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)
[![Edge Impulse](https://img.shields.io/badge/Edge_Impulse-Project-FF6F00)](https://studio.edgeimpulse.com/public/646751/live)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Control a 7-DOF Kinova Gen3 manipulator in ROS2 using natural hand gestures recognized by an Arduino Nano 33 BLE Sense board with more than 99% accuracy. Integrates TinyML, and MoveIt! for intuitive human-robot interaction.

## 📋 Project Overview
- **TinyML Gesture Recognition**: 6-class CNN model trained via Edge Impulse
- **ROS2-MoveIt! Integration**: Predefined trajectories for industrial tasks
- **Multi-Sensor Fusion**: APDS-9960 + IMU + environmental sensors
- **Bimanual Coordination**: Designed to pair with [Mobile Base Control](https://github.com/NajeebAhmedBhuiyan/Gesture-Based-Mobile-Robot-Control-in-ROS)

## 🛠️ Prerequisites
- **OS**: Ubuntu 24.04 LTS
- **ROS2**: Jazzy Jalisco ([Installation Guide](https://docs.ros.org/en/jazzy/Installation.html))
- **MoveIt2**: MoveIt2 ([Installation Guide](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)
- **Edge Impulse**: [Know about this from here](https://edgeimpulse.com) & [My Impulse](https://studio.edgeimpulse.com/public/646751/live)
- **Arduino IDE** ([Installation Guide](https://www.arduino.cc/en/software))
- **Python 3.10+**:
  ```bash
  sudo apt install python3-pip python3-venv
  pip install pyserial pygame
  ```

## 🛠️ Hardware Setup 
Just 1 Arduino Nano 33 BLE Sense board with it's USB cable, thats it!

## ⚙️ Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/NajeebAhmedBhuiyan/Gesture-Based-Manipulator-Control-in-ROS-with-TinyML.git
   ```
2. Build the ROS2 workspace:
   Put the `kinova_control` folder into the `src` file of your `ws_moveit` workspace after installing [MoveIt2](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html) and build it.
   ```bash
   cd ws_moveit/
   colcon build --packages-select kinova_control
   source install/setup.bash
   ```

## 🚀 Usage
1. Flash the Arduino code via the arduino IDE into the Arduino Nano 33 BLE Sense board.
2. Firstly, launch the Kinova Gen3 in RViz:
   ```bash
   ros2 launch moveit2_tutorials demo.launch.py
   ```
3. Then, launch the `kinova_control` package:
   ```bash
   ros2 launch kinova_control kinova_control.launch.py
   ```
4. Control modes:
   Using the Arduino Nano 33 BLE Sense board, draw any of the following gestures:
   1. Up-Down
   2. Forward-Backward
   3. Left-Right
   4. Circle
   5. Rectangle
   6. Flat Rectangle

  Each gesture is connected to a particular movement of the Kinova Gen3 Manipulator and upon detecting the particular gestures by the board, it executes that movement.  

## 🧩 System Architecture
```mermaid
graph TD
  A[Arduino Nano 33 BLE Sense] -->|Serial| B[Gesture Controller Node]
  B -->|Service Call| C[MoveIt Service Node]
  C -->|Trajectory Planning| D[Kinova Gen3 Manipulator]
  D -->|Joint States| E[RViz/Gazebo]
  C -->|Diagnostics| F[ROS2 Bag]
  B -->|GUI Feedback| G[Pygame Interface]
  
  style A fill:#4CAF50,stroke:#388E3C
  style B fill:#2196F3,stroke:#1976D2
  style C fill:#9C27B0,stroke:#7B1FA2
  style D fill:#FF5722,stroke:#E64A19

## 🚨 Troubleshooting
1. **Serial Port Permissions**:
   ```bash
   ls /dev/ttyACM*
   sudo chmod 666 /dev/ttyACM0
   ```
2. **Missing Python Packages**:
   ```bash
   sudo apt install python3-pip python3-venv
   pip install pyserial pygame
   ```

## 📜 License
Apache License 2.0 - See [LICENSE](LICENSE) file

## 🙌 Contribution
Contributions welcome! Please follow:
1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Open a pull request



