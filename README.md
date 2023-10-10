# uvc_manufacture_server

## Document
- [uvc_manufacture_server - For MQTT data manufacturing](#uvcmanufactureserver)
    - [Document](#document)
    - [Environment](#1-environment)
    - [SetUp Installation](#2-setup-installation)
        - [Prerequisites](#2-1-prerequisites)
        - [Install openjdk17](#2-2-install-openjdk17)
          - [IF your jdk version is already installed by another version, change openjdk version into 17](#2-2-1-if-your-jdk-version-is-already-installed-by-another-version-change-openjdk-version-into-17)
        - [Install gradle 8.2.1](#2-3-install-gradle-821)
          - [Install wget](#2-3-1-install-wget)
          - [Install unzip](#2-3-2-install-unzip)
          - [Download & Unzip gradle.zip file](#2-3-3-download--unzip-gradlezip-file)
          - [Write & Apply script file for export installed gradle path](#2-3-4-write--apply-script-file-for-export-installed-gradle-path)
          - [Check your gradle version](#2-3-5-check-your-gradle-version)
        - [Install mosquitto 1.6.9](#2-4-install-mosquitto-169)
    - [Clone & Build Project](#3-clone--build-project)
      - [Clone Project](#3-1-clone-project)
      - [Build Project](#3-2-build-project)
    - [Usage Example](#4-usage-example)

## 1. Environment
* <img src="https://img.shields.io/badge/openjdk17-437291?style=for-the-badge&logo=openjdk&logoColor=white">
* <img src="https://img.shields.io/badge/kotlin 1.9-7F52FF?style=for-the-badge&logo=kotlin&logoColor=white">
* <img src="https://img.shields.io/badge/spring boot 3.1.3-6DB33F?style=for-the-badge&logo=springboot&logoColor=white">
* <img src="https://img.shields.io/badge/mqtt-660066?style=for-the-badge&logo=mqtt&logoColor=white">
  <img src="https://img.shields.io/badge/Intellij IDEA-000000?style=for-the-badge&logo=intellijidea&logoColor=white">
* <img src="https://img.shields.io/badge/ubuntu 20.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white">

## 2. SetUp Installation

### 2-1. Prerequisites

Before installing, please ensure the following software is installed and configured on your system:

- [ubuntu](https://ubuntu.com/) version required 20.04 - **INSTALL [ubuntu 20.04](https://ubuntu.com/)**

- [openjdk](https://openjdk.org/) version required 17 - **INSTALL [openjdk17](https://openjdk.org/projects/jdk/17/)** 

- [mosquitto](https://mosquitto.org/) version required 1.6.9 - **INSTALL [mosquitto 1.6.9](https://mosquitto.org/)**

### 2-2. Install openjdk17
```bash
sudo apt-get update
sudo apt-get install openjdk-17-jdk

# If you installed openjdk17 successfully, then check your java version
java --version
#  openjdk 17.0.8.1 ${installed date}
# OpenJDK Runtime Environment (build 17.0.8.1+1-Ubuntu-0ubuntu120.04)
# OpenJDK 64-Bit Server VM (build 17.0.8.1+1-Ubuntu-0ubuntu120.04, mixed mode, sharing)
```

### 2-2-1. IF your jdk version is already installed by another version, change openjdk version into 17
```bash
sudo apt-get update
sudo apt-get install openjdk-17-jdk

sudo update-alternatives --config java

# There are 3 choices for the alternative java (providing /usr/bin/java).
#
#  Selection    Path                                            Priority   Status
# ------------------------------------------------------------
# * 0            /usr/lib/jvm/java-17-openjdk-amd64/bin/java      1711      auto mode
#  1            /usr/lib/jvm/java-11-openjdk-amd64/bin/java      1111      manual mode
#  2            /usr/lib/jvm/java-17-openjdk-amd64/bin/java      1711      manual mode
#  3            /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java   1081      manual mode
#
# Press <enter> to keep the current choice[*], or type selection number: 0


sudo update-alternatives --config javac

# There are 3 choices for the alternative javac (providing /usr/bin/javac).
#
#  Selection    Path                                          Priority   Status
# ------------------------------------------------------------
# * 0            /usr/lib/jvm/java-17-openjdk-amd64/bin/javac   1711      auto mode
#  1            /usr/lib/jvm/java-11-openjdk-amd64/bin/javac   1111      manual mode
#  2            /usr/lib/jvm/java-17-openjdk-amd64/bin/javac   1711      manual mode
#  3            /usr/lib/jvm/java-8-openjdk-amd64/bin/javac    1081      manual mode
#
# Press <enter> to keep the current choice[*], or type selection number: 0
```

### 2-3. Install gradle 8.2.1

### 2-3-1. Install wget
```bash
sudo apt-get update
sudo apt-get install wget
```

### 2-3-2. Install unzip
```bash
sudo apt-get update
sudo apt-get install unzip
```

### 2-3-3. Download & Unzip gradle.zip file
```bash
wget https://services.gradle.org/distributions/gradle-8.2.1-bin.zip -P /tmp

sudo unzip -d /opt/gradle /tmp/gradle-8.2.1-bin.zip

ls /opt/gradle/gradle-8.2.1
# /opt/gradle/gradle-8.2.1
# bin  init.d  lib  LICENSE  NOTICE  README
```

### 2-3-4. Write & Apply script file for export installed gradle path
```bash
sudo vim /etc/profile.d/gradle.sh

# /etc/profile.d/gradle.sh
# export GRADLE_HOME=/opt/gradle/gradle-8.2.1
# export PATH=${GRADLE_HOME}/bin:${PATH}

sudo chmod +x /etc/profile.d/gradle.sh
source /etc/profile.d/gradle.sh
```

### 2-3-5. Check your gradle version
```bash
gradle -v

# ------------------------------------------------------------
# Gradle 8.2.1
# ------------------------------------------------------------

# Build time:   2023-07-10 12:12:35 UTC
# Revision:     a38ec64d3c4612da9083cc506a1ccb212afeecaa
# 
# Kotlin:       1.8.20
# Groovy:       3.0.17
# Ant:          Apache Ant(TM) version 1.10.13 compiled on January 4 2023
# JVM:          17.0.8.1 (Private Build 17.0.8.1+1-Ubuntu-0ubuntu120.04)
# OS:           Linux 5.15.0-83-generic amd64
```

### 2-4. Install mosquitto 1.6.9
```bash
sudo apt-get update
sudo apt-get install mosquitto-dev
```

## 3. Clone & Build Project

### 3-1. Clone Project
```bash
cd ${your workspace}/src
git clone https://github.com/reidlo5135/uvc_manufacture_server.git
```

## 3-2. Build Project
```bash
cd ${your workspace}/src/uvc_manufacture_server

./gradlew clean build
# Starting a Gradle Daemon (subsequent builds will be faster)
# 
# > Task :compileKotlin
# 
# BUILD SUCCESSFUL in 18s
# 6 actionable tasks: 6 executed
```

## 4. Usage Example
```bash
cd ${your workspace}/src/uvc_manufacture_server

./gradlew bootRun
# > Task :bootRun

#   .   ____          _            __ _ _
 # /\\ / ___'_ __ _ _(_)_ __  __ _ \ \ \ \
# ( ( )\___ | '_ | '_| | '_ \/ _` | \ \ \ \
 # \\/  ___)| |_)| | | | | || (_| |  ) ) ) )
  # '  |____| .__|_| |_|_| |_\__, | / / / /
 # =========|_|==============|___/=/_/_/_/
 # :: Spring Boot ::                (v3.1.3)

# 2023-09-13T10:09:50.551+09:00  INFO 12466 --- [  restartedMain] n.w.u.UvcManufactureServerApplicationKt  : Starting UvcManufactureServerApplicationKt using Java 17.0.8.1 with PID 12466 (/home/wavem/server_ws/src/uvc_manufacture_server/build/classes/kotlin/main started by wavem in /home/wavem/server_ws/src/uvc_manufacture_server)
# 2023-09-13T10:09:50.552+09:00 DEBUG 12466 --- [  restartedMain] n.w.u.UvcManufactureServerApplicationKt  : Running with Spring Boot v3.1.3, Spring v6.0.11
# 2023-09-13T10:09:50.554+09:00  INFO 12466 --- [  restartedMain] n.w.u.UvcManufactureServerApplicationKt  : The following 3 profiles are active: "private", "local", "mqtt"
# 2023-09-13T10:09:52.145+09:00  WARN 12466 --- [  restartedMain] ion$DefaultTemplateResolverConfiguration : Cannot find template location: classpath:/templates/ (please add some templates, check your Thymeleaf configuration, or set spring.thymeleaf.check-template-location=false)
```