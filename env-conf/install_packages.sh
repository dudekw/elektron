#!/bin/bash

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install -y git;
sudo apt-get install -y libzbar-dev;
sudo apt-get install -y ros-indigo-desktop;
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
sudo apt-get install -y ros-indigo-robot-state-publisher;
sudo apt-get install -y ros-indigo-scan-tools;
sudo apt-get install -y ros-indigo-yocs-cmd-vel-mux;
sudo apt-get install -y ros-indigo-navigation;


