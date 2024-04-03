# ktp_task_controller
- KTP 임무 / 제어 관리 패키지

## Document
- [ktp_task_controller](#ktp_task_controller)
  - [Document](#document)
  - [Environment](#1-environment)
  - [SetUp Installation](#2-setup-installation)
    - [Prerequisites](#2-1-prerequisites)
  - [Clone & Build Project](#2-clone--build-project)
    - [Clone Project](#2-1-clone-project)
    - [Build Project](#2-2-build-project)
  - [Launch Check](#3-build-check)


## 1. Environment
* <img src="https://img.shields.io/badge/ROS2 humble-22314E?style=for-the-badge&logo=ros&logoColor=white">
* <img src="https://img.shields.io/badge/ubuntu 22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white">
* <img src="https://img.shields.io/badge/python 3.10.12-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54">
* <img src="https://img.shields.io/badge/CMake-064F8C.svg?style=for-the-badge&logo=cmake&logoColor=white">

## 2. SetUp Installation

### 2-1. Prerequisites

Before installing, please ensure the following software is installed and configured on your system:

- [ubuntu](https://ubuntu.com/) version required 20.04 - **INSTALL [ubuntu 22.04](https://ubuntu.com/)**

- [ROS2](https://index.ros.org/doc/ros2/Installation/) version required humble-hawksbill -
  **INSTALL [ROS2 humble-hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)**

## 2. Clone & Build Project

### 2-1. Clone Project
```bash
cd ~/kec_ws/src/rms/
git clone https://github.com/WAVEM-Inc/rms.git
```

### 2-2. Build Project
```bash
cd ~/kec_ws/src/rms/
colcon build --packages-select ktp_task_controller
```

## 3. Launch Check
```bash
source ~/kec_ws/src/rms/install/setup.bash
ros2 launch ktp_task_controller ktp_task_controller.launch.py
```