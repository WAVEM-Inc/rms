# ktp_interface
- KTP 3rd Party Agent 통신(ROS - TCP) 인터페이스 패키지

## Document
- [ktp_interface](#ktp_interface)
  - [Document](#document)
  - [Environment](#1-environment)
  - [SetUp Installation](#2-setup-installation)
    - [Prerequisites](#2-1-prerequisites)
      - [Install rosbridge_library](#2-1-1-install-rosbridge-library)
      - [Install pip requirements](#2-1-2-install-pip-requirements)
  - [Clone & Build Project](#3-clone--build-project)
    - [Clone Project](#3-1-clone-project)
    - [Build Project](#3-2-build-project)
  - [Launch Check](#4-build-check)


## 1. Environment
* <img src="https://img.shields.io/badge/ROS2 humble-22314E?style=for-the-badge&logo=ros&logoColor=white">
* <img src="https://img.shields.io/badge/ubuntu 22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white">
* <img src="https://img.shields.io/badge/python 3.10.12-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54">
* <img src="https://img.shields.io/badge/CMake-064F8C.svg?style=for-the-badge&logo=cmake&logoColor=white">

## 2. SetUp Installation

### 2-1. Prerequisites

Before installing, please ensure the following software is installed and configured on your system:

- [ubuntu](https://ubuntu.com/) version required 22.04 - **INSTALL [ubuntu 22.04](https://ubuntu.com/)**

- [ROS2](https://index.ros.org/doc/ros2/Installation/) version required humble-hawksbill -
  **INSTALL [ROS2 humble-hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)**

- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) version required humble-hawksbill -
  **INSTALLL [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)**

- [python3](https://www.python.org/downloads/) version required over than 3.0 - 
  **INSTALL [python3](https://www.python.org/downloads/)**

### 2-1-1. Install rosbridge-library
```bash
sudo apt-get install ros-humble-rosbridge-library
```

### 2-1-2. Install pip requirements
```bash
cd ~/kec_ws/src/rms/ktp_dummy_interface/
pip install -r pip_requirement.txt
```

## 3. Clone & Build Project

### 3-1. Clone Project
```bash
cd ~/kec_ws/src/rms/
git clone https://github.com/WAVEM-Inc/rms.git
```

### 3-2. Build Project
```bash
cd ~/kec_ws/src/rms/
colcon build --packages-select ktp_interface
```

## 4. Launch Check
```bash
source ~/kec_ws/src/rms/install/setup.bash
ros2 launch ktp_interface ktp_interface.launch.py
```