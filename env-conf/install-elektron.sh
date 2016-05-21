#!/bin/bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install -y libzbar-dev;
sudo apt-get install -y ros-indigo-desktop-full;
sudo apt-get remove -y gazebo2;
sudo apt-get install -y libgazebo5-dev;
sudo apt-get install -y gazebo5;
sudo apt-get install -y ros-indigo-std-srvs;
sudo apt-get install -y python-catkin-tools;
sudo apt-get install -y ros-indigo-control-toolbox;
sudo apt-get install -y ros-indigo-controller-manager;
sudo apt-get install -y ros-indigo-transmission-interface;
sudo apt-get install -y ros-indigo-joint-limits-interface ;
sudo apt-get install -y ros-indigo-joint-state-controller ;
sudo apt-get install -y ros-indigo-diff-drive-controller ;
sudo apt-get install -y ros-indigo-effort-controllers ;
sudo apt-get install -y ros-indigo-position-controllers;
sudo apt-get install -y ros-indigo-image-transport;
sudo apt-get install -y ros-indigo-cv-bridge;
sudo apt-get install -y ros-indigo-eigen-conversions;
sudo apt-get install -y ros-indigo-tf-conversions;
sudo apt-get install -y ros-indigo-nodelet;
sudo apt-get install -y ros-indigo-driver-base;
sudo apt-get install -y ros-indigo-polled-camera;
sudo apt-get install -y ros-indigo-camera-info-manager;
sudo apt-get install -y ros-indigo-ros-core;
sudo apt-get install -y ros-indigo-xacro;
sudo apt-get install -y ros-indigo-robot-state-publisher
mkdir -p ~/rapp;
mkdir -p ~/rapp/rapp-api/src ;
cd ~/rapp/rapp-api/src;

if [ -d "rapp-api" ]; then
	cd ~/rapp/rapp-api/src/rapp-api;
	git pull;
else
	git clone -b wut https://github.com/rapp-project/rapp-api.git;
fi

cd ~/rapp/rapp-api/src;

if [ -d "rapp-robots-api" ]; then
	cd ~/rapp/rapp-api/src/rapp-robots-api;
	git pull;
else
	git clone -b cpp https://github.com/rapp-project/rapp-robots-api.git;
fi

cd ~/rapp/rapp-api/src;

if [ -d "rapp-cloud-api" ]; then
	cd ~/rapp/rapp-api/src/rapp-cloud-api;
	git pull;
else
	git clone https://github.com/maciek-slon/rapp-cloud-api.git;
fi

source /opt/ros/indigo/setup.bash;
cd ~/rapp/rapp-api;

if [ -d "install" ]; then
	catkin clean -y;
	catkin init;
	catkin config --cmake-args -DBUILD_ALL=ON;
	catkin config --install;
	catkin build;
else
	catkin init;
	catkin config --cmake-args -DBUILD_ALL=ON;
	catkin config --install;
	catkin build;
fi


mkdir -p ~/rapp/rapp-apps/src;
cd ~/rapp/rapp-apps/src;
if [ -d "rapp-samples" ]; then
	cd ~/rapp/rapp-api/src/rapp-samples;
	git pull;
else
	git clone https://github.com/maciek-slon/rapp_sample.git rapp-samples;
fi

source ~/rapp/rapp-api/install/setup.bash;

cd ~/rapp/rapp-apps;

if [ -d "install" ]; then
	catkin clean -y;
	catkin init;
	catkin config --install;
	catkin config --isolate-install;
	catkin build;
else
	catkin init;
	catkin config --install;
	catkin config --isolate-install;
	catkin build;
fi

mkdir -p ~/rapp/robots/src;
cd ~/rapp/robots/src;
if [ -d "elektron" ]; then
	cd ~/rapp/robots/src/elektron;
	git pull;
else
	git clone https://github.com/dudekw/elektron.git;
fi

cd ~/rapp/robots/src/elektron;
git submodule update --init -- gazebo_ros_pkgs elektron_base/real_effectors/src/nf elektron_base/real_effectors/src/serialcomm rapp-api-elektron;
source /opt/ros/indigo/setup.bash;
source ~/rapp/rapp-api/install/setup.bash;
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

