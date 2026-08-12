# ROS2 Humble Quadrotor Controller (Voice & Manual Control)

This package contains a fully containerized, ROS2 Humble-based manual and offline voice controller for the X3 UAV in Gazebo (Ignition). It implements continuous key-holding/press-and-hold controls, offline voice command duration processing using **Vosk**, and runs 100% offline.

---

## Features
1. **Fully Dockerized & ROS2 Humble-controlled**: The container includes ROS2 Humble, Ignition Gazebo Sim, `ros_gz_bridge`, and PyQt5 dependencies.
2. **Offline-Ready Simulation**: All online Ignition Fuel models (e.g. X3 UAV) are cached during Docker build. Bloated visual models (e.g. cloud visual mesh containing over 100k lines of vertices and all hoops) are removed, leaving a super-optimized world.
3. **Continuous Press-and-Hold Manual Controls**: Manual controls only move the drone *while* the button is pressed. Releasing the button immediately sends a stop/hover command.
4. **Offline High-Accuracy Voice Control**: Voice recognition uses local `Vosk` models, executing instructions for set periods before auto-hovering:
   - **Forward, Backward, Ascend, Descend**: 1.0 second movement.
   - **Slide Left, Slide Right, Rotate Left, Rotate Right**: 0.5 second movement.
   - **Stop / Hover**: Immediate stop.

---

## Instructions to Run the Setup

To build and launch the environment on your host machine, perform the following steps:

### 1. Allow GUI/X11 access on the Host
Since Gazebo and the PyQt5 GUI run inside Docker, you must allow the container to access your host's X11 server:
```bash
xhost +local:root
```

### 2. Launch using Docker Compose
Run the following command to build the image and launch the entire stack:
```bash
docker compose up --build
```

This single command will:
1. Build the ROS2 Humble Docker container.
2. Download and cache the local Vosk english speech model and X3 UAV Gazebo fuel files.
3. Source ROS2 and build our custom `quadrotor_control` package.
4. Launch the optimized `world.sdf` simulation.
5. Initialize the `ros_gz_bridge` to bridge `/X3/cmd_vel`.
6. Start the PyQt5 Voice & Manual control GUI.

---

## Voice Commands Synonym Guide
Click **"Start Listening"** on the GUI, allow microphone access, and speak clearly:
- **Forward**: `"forward"`, `"front"`, `"go"`
- **Backward**: `"backward"`, `"back"`
- **Ascend**: `"up"`, `"ascend"`, `"climb"`
- **Descend**: `"down"`, `"descend"`, `"land"`
- **Slide Left**: `"left"`
- **Slide Right**: `"right"`
- **Rotate Left**: `"turn left"`, `"rotate left"`
- **Rotate Right**: `"turn right"`, `"rotate right"`
- **Stop**: `"stop"`, `"hold"`, `"hover"`

