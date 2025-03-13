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

## Before continuing make sure installed
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

- 
```sh

```
