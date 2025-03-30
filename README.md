# Gesture-Based Manipulator Control in ROS 🤖✋

**ROS2 Jazzy Package for Kinova Gen3 Manipulator Control Using TinyML-Powered Hand Gestures**  
*Part of Undergraduate Thesis: "A Bimanual Gesture Interface for ROS-Based Mobile Manipulators Using Edge-AI and Sensor Fusion"*

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-%23C5221F)](https://docs.ros.org/en/jazzy/)
[![Edge Impulse](https://img.shields.io/badge/Edge_Impulse-Project-FF6F00)](https://studio.edgeimpulse.com/public/646751/live)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Control a 7-DOF Kinova Gen3 manipulator in ROS2 using natural hand gestures recognized by an Arduino Nano 33 BLE Sense board with 98.6% accuracy. Integrates TinyML, MoveIt!, and Gazebo for intuitive human-robot interaction.

https://github.com/NajeebAhmedBhuiyan/Gesture-Based-Manipulator-Control-in-ROS/assets/58342357/abc12345-67d8-90ef-ghij-klmnopqrstuv (Insert demo GIF)

## 📋 Project Overview
- **TinyML Gesture Recognition**: 6-class CNN model trained via Edge Impulse
- **ROS2-MoveIt! Integration**: Predefined trajectories for industrial tasks
- **Multi-Sensor Fusion**: APDS-9960 + IMU + environmental sensors
- **Bimanual Coordination**: Designed to pair with [Mobile Base Control](https://github.com/NajeebAhmedBhuiyan/Gesture-Based-Mobile-Robot-Control-in-ROS)

## 🛠️ Hardware Requirements
| Component               | Quantity | Purpose                     |
|-------------------------|----------|-----------------------------|
| Arduino Nano 33 BLE Sense | 1       | Gesture recognition         |
| APDS-9960 Sensor        | 1        | Proximity/gesture detection |
| USB-C Cable             | 1        | Power/communication         |
| Kinova Gen3 (Sim/Real)  | 1        | Manipulation platform       |

![Circuit Diagram](arduino/circuit_diagram.png)

## 💻 Software Prerequisites
- **OS**: Ubuntu 24.04 LTS
- **ROS2**: Jazzy Jalisco ([Install Guide](https://docs.ros.org/en/jazzy/Installation.html))
- **Arduino CLI** ([Install Guide](https://arduino.github.io/arduino-cli/))
- **Python 3.10+**:
  ```bash
  sudo apt install python3-pip python3-venv
  pip install pyserial pygame
