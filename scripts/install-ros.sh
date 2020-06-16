#!/bin/bash
# setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# setup keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

echo "Please select ROS version:"

select os in kinetic melodic noetic
do
case $os in "kinetic"|"melodic"|"noetic")
# install
sudo apt-get update
sudo apt-get install ros-$os-ros-base
sudo rosdep init
rosdep update

# environment setup
echo "source /opt/ros/$os/setup.bash" >> ~/.bashrc
source ~/.bashrc
;;
# Matching with invalid data
*)
echo "Invalid entry."
break
esac
done

# dependencies for building packages
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# python
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
