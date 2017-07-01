#!/bin/bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget -O install_packages.sh https://raw.githubusercontent.com/dudekw/elektron/kinetic/env-conf/install_packages.sh;
bash install_packages.sh

sudo apt-get install -y libgazebo7-dev;
sudo apt-get install -y gazebo7;

mkdir -p ~/rapp/robots/src;
cd ~/rapp/robots/src;
if [ -d "elektron" ]; then
	cd ~/rapp/robots/src/elektron;
	git pull;
else
	git clone https://github.com/dudekw/elektron.git;
fi

cd ~/rapp/robots/src/elektron;
touch elektron_base/elektron-real-effectors/CMakeLists.txt;
git submodule update --init --recursive elektron-simulation;
source /opt/ros/kinetic/setup.bash;
cd ../..;

if [ -d "install" ]; then
	catkin clean -y;
	catkin init;
	catkin config --install;
	catkin build
else
	catkin init;
	catkin config --install;
	catkin build
fi

