# Autonomous Vehicle Platform (AV-CPP2025)
Senior Design Project – Cal Poly Pomona (EGR 4820 / 4830)

This repository contains the software for an autonomous ground vehicle platform developed at California State Polytechnic University, Pomona. While the system retains legacy remote-piloting functionality, the primary focus is now on autonomous perception, sensing, and vehicle control.

The platform integrates ROS2, LiDAR-based object detection, a camera with YOLO-based computer vision, onboard sensor fusion, and Arduino-based actuator subsystems. A Windows client application is included for system monitoring and optional manual override.

---

## Project Overview

The autonomous vehicle platform consists of:

- ROS2 nodes for LiDAR processing, camera inference, localization, and decision-making  
- A redesigned TCP protocol supporting bidirectional communication  
- Arduino microcontroller subsystems for steering, throttle, lighting, and telemetry  
- An RTSP video pipeline from the onboard camera  
- A Windows GUI client for viewing vehicle status and optionally controlling the vehicle  
- Documentation describing setup, system behavior, and workflows (in `Documentation/2025/`)

This repository forms a modular foundation for continued development of autonomous navigation capabilities.

---

## Repository Structure

```text
AV-CPP2025/
│
├── Vehicle/                 # ROS2 workspace for autonomous perception & control
│   ├── src/
│   │   ├── vehicle_core/    # Core vehicle node (TCP, mode/state, drive arbiter)
│   │   ├── object_detection/ # LiDAR clustering and safety zone logic
│   │   └── computer_vision/ # Camera input, YOLO inference, RTSP streamer
│   └── build/ install/      # ROS2 build outputs
│
├── TCPClient/               # Windows GUI client
│   ├── src/
│   ├── Shared/              # Shared networking definitions (Keys.h)
│   ├── Dependencies/        # FFmpeg DLLs for RTSP support
│   └── build/               # Compiled TCPClient.exe
│
├── Subsystems/              # Arduino firmware for hardware subsystems
│   ├── DriveSubsystem/
│   └── LightingSubsystem/
│
├── Shared/                  # Shared headers used by both vehicle and client code
│
└── Documentation/
    └── 2025/                # Setup Guide, User Guide, and Final Report

## Software Components
### Vehicle Software (ROS2)

The Orange Pi 5 runs the ROS2 system, which includes:
- LiDAR object detection and classification
- Camera acquisition and YOLOv8-Nano inference
- RTSP video streaming
- EKF-based localization (GPS, IMU, odometry)
- Drive arbitration and safety-zone braking logic
- Support for manual override through TCP messages
These components collectively form the autonomous perception and control stack.

### TCP Client (Windows)

A GUI application that provides:
- Real-time vehicle status monitoring
- Optional manual teleoperation
- Integrated RTSP video viewing
- Local or remote (FRP-based) connectivity
The client is intended primarily for testing, debugging, and validation.

Microcontroller Subsystems

Arduino boards handle deterministic, low-level hardware operations:
- Steering, throttle, odomtery and brakes
- Lighting control
Telemetry reporting (IMU, battery, speed)

## Setup ##

This repository depends on the existence of other repositories (as packages in ROS) within the ROS workspace in the directory `Vehicle/`.
To automatically load these repositories, set the working directory to `Vehicle/` and use `vcs` to import them like so:

```bash
cd Vehicle/
vcs import < av_cpp2026.repos
```

## Documentation

Detailed instructions for setup, development workflow, and runtime operation are located in:
'Documentation/2025/'
This includes:
- Setup Guide
- User Guide
- Final Report (architecture, design rationale, testing results)
- Wiring diagram (.drawio file)

## Credits

Carson Quesada
Justin Cortes
Justin Ortiz
Jielsen Travis Cangco
Advisor: Dr. Tamer Omar

This project is intended for academic and research use within EGR 4820/4830.