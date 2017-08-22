#!/bin/bash

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install -y git;
sudo apt-get install -y libzbar-dev;
#sudo apt-get install -y ros-kinetic-desktop;
sudo apt-get install -y ros-kinetic-std-srvs;
sudo apt-get install -y python-catkin-tools;
sudo apt-get install -y ros-kinetic-control-toolbox;
sudo apt-get install -y ros-kinetic-controller-manager;
sudo apt-get install -y ros-kinetic-transmission-interface;
sudo apt-get install -y ros-kinetic-joint-limits-interface ;
sudo apt-get install -y ros-kinetic-joint-state-controller ;
sudo apt-get install -y ros-kinetic-diff-drive-controller ;
sudo apt-get install -y ros-kinetic-effort-controllers ;
sudo apt-get install -y ros-kinetic-position-controllers;
sudo apt-get install -y ros-kinetic-image-transport;
sudo apt-get install -y ros-kinetic-cv-bridge;
sudo apt-get install -y ros-kinetic-eigen-conversions;
sudo apt-get install -y ros-kinetic-tf-conversions;
sudo apt-get install -y ros-kinetic-nodelet;
sudo apt-get install -y ros-kinetic-driver-base;
sudo apt-get install -y ros-kinetic-polled-camera;
sudo apt-get install -y ros-kinetic-camera-info-manager;
sudo apt-get install -y ros-kinetic-ros-core;
sudo apt-get install -y ros-kinetic-xacro;
sudo apt-get install -y ros-kinetic-robot-state-publisher;
sudo apt-get install -y ros-kinetic-scan-tools;
sudo apt-get install -y ros-kinetic-yocs-cmd-vel-mux;
sudo apt-get install -y ros-kinetic-navigation;
sudo apt-get install -y ros-kinetic-gmapping;
sudo apt-get install -y ros-kinetic-cv-bridge;
