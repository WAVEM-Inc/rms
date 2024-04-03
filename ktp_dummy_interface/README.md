# ktp_dummy_interface
- 내부 주행 테스트 용 ROS - MQTT 중계 서버

## Document
- [ktp_dummy_interface](#ktp_dummy_interface)
  - [Document](#document)
  - [Environment](#1-environment)
  - [SetUp Installation](#2-setup-installation)
    - [Prerequisites](#2-1-prerequisites)
      - [Install mosquitto 2.0.11](#2-1-1-install-mosquitto-2011)
      - [Set Up Mosquitto Configuration](#2-1-2-set-up-mosquitto-configuration)
        - [Write Configuration File](#2-1-2-1-write-configuration-file)
        - [Write Script File](#2-1-2-2-write-script-file)
        - [Run mosquitto](#2-1-2-3-run-mosquitto)
  - [Clone & Build Project](#2-clone--build-project)
    - [Clone Project](#2-1-clone-project)
    - [Build Project](#2-2-build-project)
  - [Launch Check](#3-build-check)


## 1. Environment
* <img src="https://img.shields.io/badge/ROS2 humble-22314E?style=for-the-badge&logo=ros&logoColor=white">
* <img src="https://img.shields.io/badge/ubuntu 22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white">
* <img src="https://img.shields.io/badge/python 3.10.12-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54">
* <img src="https://img.shields.io/badge/CMake-064F8C.svg?style=for-the-badge&logo=cmake&logoColor=white">
* <img src="https://img.shields.io/badge/mqtt-660066?style=for-the-badge&logo=mqtt&logoColor=white">

## 2. SetUp Installation

### 2-1. Prerequisites

Before installing, please ensure the following software is installed and configured on your system:

- [ubuntu](https://ubuntu.com/) version required 20.04 - **INSTALL [ubuntu 22.04](https://ubuntu.com/)**

- [mosquitto](https://mosquitto.org/) version required 2.0.11 - **INSTALL [mosquitto 1.6.9](https://mosquitto.org/)**

- [ROS2](https://index.ros.org/doc/ros2/Installation/) version required humble-hawksbill -
  **INSTALL [ROS2 humble-hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)**

### 2-1-1. Install mosquitto 2.0.11
```bash
sudo apt-get install mosquitto
sudo apt-get install mosquitto-dev
sudo apt-get install mosquitto-clients
```

### 2-1-2. Set Up mosquitto Configuration
### 2-1-2-1. Write Configuration File
```bash
sudo systemctl service stop mosquitto.service

cd /etc/mosquitto/conf.d/
// mosquitto.conf
listener 1883 ${IPv4 Address}
protocol mqtt
allow_anonymous true

listener 8883 ${IPv4 Address}
protocol websockets
allow_anonymous true
```
### 2-1-2-2. Write Script File
```bash
cd ~/

// mosquitto.sh
mosquitto --verbose --config-file /etc/mosquitto/conf.d/mosquitto.conf
```
### 2-1-2-3. Run mosquitto
```bash
sudo chmod +x ./mosquitto.sh
./mosquitto.sh
```

## 2. Clone & Build Project

### 2-1. Clone Project
```bash
cd ~/kec_ws/src/rms/
git clone https://github.com/WAVEM-Inc/rms.git
```

### 2-2. Build Project
```bash
cd ~/kec_ws/src/rms/
colcon build --packages-select ktp_dummy_interface
```

## 3. Launch Check
```bash
source ~/kec_ws/src/rms/install/setup.bash
ros2 launch ktp_dummy_interface ktp_dummy_interface.launch.py
```