#!/bin/bash
ROS2_RELEASE=foxy      # ROS2 release version

#==================================================
# verify that PC configuration matches requirements
#==================================================

function print_result() {
  if [ $? -eq 0 ]; then   # check command result
    echo -e "\e[00;32m[OK]\e[00m"
  else
    echo -e "\e[00;31m[FAIL]\e[00m"
  fi
}

# replace print_result() call with print_disabled() call to temporarily disable a test
function print_disabled() {
  echo -e "\e[00;30m[DISABLED]\e[00m"
}

function check_internet() {
  echo "Checking internet connection... "
  printf "  - %-30s" "google.com:"
  print_result $(ping -q -c1 google.com &> /dev/null)

} #end check_internet()


function check_deb() {
  printf "  - %-30s" "$1:"
  print_result $(dpkg-query -s $1 &> /dev/null)
}

function disable_deb() {
  printf "  - %-30s" "$1:"
  print_disabled $(dpkg-query -s $1 &> /dev/null)
}

function check_debs() {
  echo "Checking debian packages... "
  check_deb git
  check_deb meld
  check_deb build-essential
  check_deb libfontconfig1
  check_deb mesa-common-dev
  check_deb libglu1-mesa-dev
  check_deb python3-argcomplete
  check_deb python3-vcstool
   echo "Checking Python3 packages for ROS2:"
  check_deb python3-colcon-bash
  check_deb python3-colcon-core
  check_deb python3-colcon-ros
  check_deb python3-colcon-common-extensions
   echo "Checking ROS2 packages:"
  check_deb ros-$ROS2_RELEASE-ros-base
  check_deb ros-$ROS2_RELEASE-moveit
  check_deb ros-$ROS2_RELEASE-ros2-control
  check_deb ros-$ROS2_RELEASE-ros2-controllers
  check_deb ros-$ROS2_RELEASE-xacro
}

function check_bashrc() {
  echo "Checking .bashrc... "
  printf "  - %-30s" "\$ROS_VERSION:"
  if [ -z ${ROS_VERSION+x} ]; then
	print_disabled $(false)
  else
	print_result $([ $ROS_VERSION == "2" ])
  fi
  printf "  - %-30s" "\$ROS_DISTRO:"
  if [ -z ${ROS_DISTRO+x} ]; then
	print_disabled $(false)
  else
	print_result $([ $ROS_DISTRO == "$ROS2_RELEASE" ])
  fi
}


#---------------------------------------
# run the actual tests
#---------------------------------------

check_internet
check_debs
check_bashrc
