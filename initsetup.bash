#!/bin/bash
echo "============================="
echo "Installing apt dependencies"
echo "============================="
sudo apt install -y \
ninja-build \
exiftool \
python-argparse \
python-empy \
python-toml \
python-numpy \
python-yaml \
python-dev \
python-pip \
ninja-build \
protobuf-compiler \
libeigen3-dev \
genromfs
echo "============================="
echo "Python dependencies"
echo "============================="
pip install \
pandas \
jinja2 \
pyserial \
cerberus \
numpy \
toml \
pyquaternion
echo "============================="
echo "Installing ROS melodic"
echo "============================="
echo "Install ROS (y/n)?"
read ans
if [ "$ans" = "y" ]
then
    echo "Installing ros melodic!"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt-get update
    sudo apt-get install ros-melodic-desktop-full
    source /opt/ros/melodic/setup.bash
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python-rosinstall python-rosinstall-generator python-catkin-tools python-wstool build-essential
    # install ros-gazebo plugins
    sudo apt install ros-melodic-gazebo-*
fi
echo "============================="
echo "MAVROS"
echo "============================="
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
cd ~/Downloads/
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
echo "============================="
echo "PX4"
echo "============================="
mkdir -p ~/catkin_ws
cd ~/catkin_ws
mkdir -p ./src
cd src
#it could take a while
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.8.0
make posix_sitl_default gazebo