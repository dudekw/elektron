#!/bin/bash

export RAPP_BASE=~/rapp;
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

mkdir -p ~/rapp/robots/src;
cd ~/rapp/robots/src;
if [ -d "elektron" ]; then
	cd ~/rapp/robots/src/elektron;
	git pull;
else
	git clone https://github.com/dudekw/elektron.git;
fi

cd ~/rapp/robots/src/elektron;
git submodule update --init --recursive elektron-simulation rapp-api-elektron elektron_apps/elektron-rapps;
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


