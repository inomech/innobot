#! /bin/bash



sudo apt update -y
sudo apt upgrade -y
sudo apt install -y curl gnupg2 lsb-release git meld build-essential libfontconfig1 mesa-common-dev libglu1-mesa-dev python3-vcstool

cd $HOME

# ROS2 packages source

# Set locale
sudo apt update -y 
sudo apt install-y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# First ensure that the Ubuntu Universe repository is enabled.
sudo apt install software-properties-common
sudo add-apt-repository universe

# Now add the ROS 2 GPG key with apt.
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
# Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update -y
sudo apt upgrade -y

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
