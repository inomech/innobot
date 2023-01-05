#! /bin/bash

sudo apt update -y
sudo apt upgrade -y
sudo apt install -y curl gnupg2 lsb-release git meld build-essential libfontconfig1 mesa-common-dev libglu1-mesa-dev python3-vcstool

cd $HOME

# ROS2 packages source
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update -y

# ROS2 install
sudo apt install -y ros-foxy-desktop ros-foxy-moveit
sudo apt install -y ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-xacro
sudo apt install -y ros-foxy-ros-base
sudo apt install -y ros-dev-tools
sudo apt install -y python3-colcon-common-extensions python3-argcomplete

# rosdep setup
sudo apt install python3-rosdep
sudo rosdep init
rosdep update


# disable screen power-off timer
gsettings set org.gnome.desktop.session idle-delay 0
