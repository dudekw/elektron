#!/bin/bash

export RAPP_BASE=$HOME/rapp
mkdir -p $RAPP_BASE 

mkdir -p $RAPP_BASE/rapp-api/src 
cd $RAPP_BASE/rapp-api/src
git clone -b wut https://github.com/rapp-project/rapp-api.git
git clone -b cpp https://github.com/rapp-project/rapp-robots-api.git

source /opt/ros/indigo/setup.bash
cd $RAPP_BASE/rapp-api
catkin init
catkin config --cmake-args -DBUILD_ALL=ON
catkin config --install
catkin build
