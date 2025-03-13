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

## Prerequisite Installation:
- [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)

- Update and upgrade system:
```sh
sudo apt update
sudo apt upgrade
```

- İnstall curl:
```sh
sudo apt install curl
```

- Install Git & configure URLs:
```sh
sudo apt-get install git gitk git-gui
git config --global url."https://github.com/".insteadOf git@github.com:
git config --global url."https://".insteadOf git://
```

- Install Python dependencies:
```sh
sudo apt-get install python3-dev python3-pip
```

## ROS İnstallation:

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

- Add Gazebo repository:
```sh
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \
```

- Add repository key & update:
```sh
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
```

- Install Gazebo 11:
```sh
sudo apt-get install gazebo11 libgazebo11-dev
```

## Gazebo Plugin Setup:

- Clone and build the plugin:
```sh
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

- Update environment for Gazebo:
```sh
cd ~
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

## PX4 & MAVSDK Dependencies:

- Add user to dialout group:
```sh
sudo usermod -a -G dialout $USER
```

- Install various dependencies:
```sh
sudo apt install -y python3-jinja2 python3-numpy python3-pandas python3-matplotlib python3-pyqt5.qtsvg protobuf-compiler libeigen3-dev \ libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-good gstreamer1.0-libav gstreamer1.0-tools \ libopencv-dev ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs ros-noetic-mavlink ros-noetic-geographic-msgs
```

- Install Python packages via pip:
```sh
sudo pip3 install pyulog mavsdk
pip install mavsdk
pip install ultralytics
```

- Install GeographicLib geoid data:
```sh
wget "https://downloads.sourceforge.net/project/geographiclib/geoids-distrib/egm96-5.tar.bz2"
tar xjf egm96-5.tar.bz2
sudo cp -r geoids/* /usr/share/GeographicLib/geoids/
```

## PX4 Autopilot Setup:

- Clone PX4 Firmware:
```sh
git clone https://github.com/PX4/Firmware.git --recursive
```

- Run PX4 setup script:
```sh
cd Firmware
bash ./Tools/setup/ubuntu.sh
```

- Reboot system to apply changes:
```sh
sudo reboot
```

- Update submodules & build simulation:
```sh
cd Firmware
git submodule update --init --recursive
DONT_RUN=1 make px4_sitl_default gazebo
```

- Configure environment variables:
```sh
cd ~
echo 'source Firmware/Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic' >> ~/.bashrc
source ~/.bashrc
```
