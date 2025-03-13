# Gazebo_ROS_Drone

![default_gzclient_camera(1)-2025-03-13T08_29_01 094318](https://github.com/user-attachments/assets/706a9b8c-8aad-498c-a4f3-dbc334e04373)

## By following through you will install and setup:
- ROS Noetic
- PX4-Autopilot
- MAVROS
- MAVProxy
- MAVSDK
- Catkin Workspace
- Ardupilot
- Gazebo 11

## Before continuing make sure you have
- [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)

## ROS İnstallation:
- Update and upgrade system:
```sh
sudo apt update
sudo apt upgrade
```

- İnstall curl:
```sh
sudo apt install curl
```

- Add ROS repository:
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

- Add ROS apt key & update
```sh
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade
```

- Install full ROS desktop
```sh
sudo apt install ros-noetic-desktop-full
```

- Install additional ROS tools:
```sh
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

- Initialize rosdep:
```sh
sudo rosdep init
rosdep update
```

- Install roslaunch & update environment:
```sh
sudo apt install python3-roslaunch
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## MAVROS Setup:

- Install Python dependencies:
```sh
sudo apt-get install python3-dev python3-pip
```

- Install ROS package manager:
```sh
sudo pip3 install rospkg
```

- Install MAVProxy:
```sh
pip install MAVProxy
```

- Prepare for ROS packages:
```sh
curl -sSL http://repo.ros2.org/repos.key | sudo apt-key add -
sudo apt-get update
```

- Install MAVROS packages:
```sh
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
```

- Download & install GeographicLib datasets:
```sh
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

## Catkin Workspace Setup:
- Install Git & configure URLs:
```sh
sudo apt-get install git gitk git-gui
git config --global url."https://github.com/".insteadOf git@github.com:
git config --global url."https://".insteadOf git://
```

- Install catkin tools:
```sh
sudo apt-get install python3-catkin-tools
```

- Create workspace & clone repository:
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/PX4/PX4-SITL_gazebo-classic.git
```

- Initialize submodules & build:
```sh
cd PX4-SITL_gazebo-classic
git submodule update --init --recursive
cd ~/catkin_ws
catkin build
```

- Update environment:
```sh
cd ~
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Ardupilot Setup:

- Update and upgrade system:
```sh
sudo apt update
sudo apt upgrade
```

- Clone & setup ArduPilot:
```sh
cd && git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot/ && git submodule update --init --recursive
```

- Install test resources & prerequisites:
```sh
sudo apt-get install python3-testresources
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

- Setup ArduPilot simulation for Gazebo:
```sh
curl -s -L https://raw.githubusercontent.com/LazyTurtle711/Gazebo_ROS_Drone/db97a9739e7e8c05211b46900188f9bec4447361/main/setup_ardu_sim.sh | /usr/bin/bash
. ~/.profile
```

## Copter Simulation:

- Launch ArduCopter simulation:
```sh
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -w
```
- After opening terminate with CTRL+C

## Gazebo Installation:

- 
```sh

```

- 
```sh

```

- 
```sh

```
