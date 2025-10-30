# 🤖 Robot-Control  
**Integration of Motor Controllers with ROS 2 via a Teensy 4.1 Microcontroller**  
*© 2025 Santiago Cuartas Palacio*

---

## 🧭 Table of Contents  
1. [Overview](#overview)  
2. [Architecture](#architecture)  
3. [Repository Structure](#repository-structure)  
4. [Getting Started](#getting-started)  
5. [Usage](#usage)  
6. [Configuration](#configuration)  
7. [Relay & Serial Library Details](#relay--serial-library-details)  
8. [Troubleshooting & Tips](#troubleshooting--tips)  
9. [Contributing](#contributing)  
10. [License](#license)

---

## 🧩 Overview  
**Robot-Control** is a ROS 2–integrated system that bridges a host computer and motor controllers through a **Teensy 4.1 microcontroller**.  
It manages real-time motion commands, relay control, and serial communication for a mobile robotic platform.

### Core Functionality
- The **ROS 2 node** subscribes to `/cmd_vel` messages (velocity commands).  
- A **custom Serial Communication Library** handles reliable data exchange between ROS 2 and the Teensy.  
- The **Teensy** interprets serial packets, controls the motor controllers, and toggles relays for auxiliary functions.

This modular setup decouples high-level motion logic (ROS 2) from low-level actuator control (Teensy + Motor Controllers), making the system scalable, debuggable, and flexible.

---

## 🧠 Architecture  

```text
+----------------------------+
| ROS 2 Host Computer        |
|  ├── ROS 2 Node (C++)      |
|  │     ├── Subscribes to /cmd_vel
|  │     ├── Controls relays via server
|  │     └── Uses SerialCom library
+----------------------------+
              │
              ▼
+----------------------------+
| Serial Communication Library|
|  - Handles serial I/O       |
|  - Provides read/write API  |
|  - Ensures packet integrity |
+----------------------------+
              │
              ▼
+----------------------------+
| Teensy 4.1 Microcontroller |
|  ├── Receives serial commands
|  ├── Controls 2 motor drivers
|  ├── Drives 2 relays
|  └── Sends feedback/status  |
+----------------------------+
```

---

## 📁 Repository Structure  

```
Robot-Control/
├── ArduinoControl/          # Microcontroller firmware for Teensy
│   ├── main.ino
│   └── ...                  
│
├── SerialCom/               # Serial communication library (C++)
│   └── Source/
│       ├── serial.cpp
│       ├── serial.hpp
│       └── ...
│
├── src/robot_control/       # ROS 2 node
│   ├── robot_control_node.cpp
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── ...
│
└── README.md
```

**Summary**
- `ArduinoControl/`: Teensy firmware handling serial + motor + relay logic.  
- `SerialCom/`: C++ library for host–microcontroller serial communication.  
- `src/robot_control/`: ROS 2 node subscribing to `/cmd_vel` and managing relays.  

---

## ⚙️ Getting Started  

### Prerequisites  
- ROS 2 Humble (or later)  
- Teensy 4.1 with motor controllers and relays connected  
- A serial/USB connection between PC and Teensy  
- C++17 compiler and CMake for building ROS packages  

---

### 🔧 Build Instructions  

```bash
# Clone this repository
git clone https://github.com/scuar003/Robot-Control.git
cd Robot-Control

# Add to your ROS 2 workspace
cd ~/ros2_ws/src
ln -s /path/to/Robot-Control/src/robot_control .

# Build
cd ~/ros2_ws
colcon build --packages-select robot_control
```

### 🧠 Flash the Microcontroller  
Upload the firmware from `ArduinoControl/` to your Teensy 4.1 using the Arduino IDE or PlatformIO.

---

## 🚀 Usage  

### 1️⃣ Start the ROS 2 Nodes 
- "CONTROLS" Starts the ROS 2 node for communication with microcontroller
- "JOY_TO_CMD" Remaps jjoy sensor msgs to geometry msgs for topic cmd_vel
```bash
ros2 launch robot_control r_control.launch.py
```

### 2️⃣ Send Velocity Commands  
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist   "{ linear: { x: 0.5, y: 0.0, z: 0.0 }, angular: { z: 0.3 } }"
```

### 3️⃣ Control Relays  
```bash
ros2 service call /relay_control robot_control/srv/RelayCmd   "{ relay_id: 1, state: true }"
```
### Use remote controler
- Make sure that remote is connected to a computer in the same ROS2 network
- Start the Joy Node 
```bash
ros2 run joy joy_node
``` 

*(Relay IDs and topic names may vary based on your configuration.)*

---

## 🔌 Relay & Serial Library Details  

### SerialCom Library  
- Handles low-level serial communication between host and microcontroller.  
- Offers robust read/write, configurable baud rate, and packet framing.  
- Abstracts OS-specific serial APIs for Linux and Windows.  


### Relay Server  
- Exposed as a ROS 2 service or TCP endpoint.  
- Allows remote toggling of two relays connected to the Teensy.  
- Useful for peripherals (lights, pumps, air valves, etc).  

---


### ✨ Author  
**Santiago Cuartas Palacio**  
> “With God and the right mentality, anything is possible.”  
> *Te Deum Laudamus*  
[GitHub @scuar003](https://github.com/scuar003)
