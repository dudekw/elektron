#!/bin/bash

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




export RAPP_BASE=~/rapp;
rm -rf ~/rapp;
mkdir -p ~/rapp;
mkdir -p ~/rapp/rapp-api/src ;
cd ~/rapp/rapp-api/src;
git clone -b wut https://github.com/rapp-project/rapp-api.git;
git clone -b cpp https://github.com/rapp-project/rapp-robots-api.git;

source /opt/ros/indigo/setup.bash;
cd ~/rapp/rapp-api;
catkin init;
catkin config --cmake-args -DBUILD_ALL=ON;
catkin config --install;
catkin build;

export RAPP_BASE=$HOME/rapp;
mkdir -p ~/rapp ;

mkdir -p ~/rapp/rapp-apps/src;
cd ~/rapp/rapp-apps/src;
git clone https://github.com/maciek-slon/rapp_sample.git rapp-samples;
source ~/rapp/rapp-api/install/setup.bash;
cd ~/rapp/rapp-apps;
catkin init;
catkin config --install;
catkin config --isolate-install;
catkin build;

mkdir -p ~/rapp/robots/src;
cd ~/rapp/robots/src;
git clone https://github.com/dudekw/elektron.git;
cd elektron;
git submodule update --init;
source /opt/ros/indigo/setup.bash;
source ~/rapp/rapp-api/install/setup.bash;
cd ../..;
catkin init;
catkin config --install;
catkin build
