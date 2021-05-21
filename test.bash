echo "Install ROS (y/n)?"
read ans
if [ "$and" = "y"]; then
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