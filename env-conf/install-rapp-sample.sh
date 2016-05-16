#!/bin/bash

export RAPP_BASE=$HOME/rapp
mkdir -p $RAPP_BASE 

mkdir -p $RAPP_BASE/rapp-apps/src
cd $RAPP_BASE/rapp-apps/src
git clone https://github.com/maciek-slon/rapp_sample.git rapp-samples
source $RAPP_BASE/rapp-api/install/setup.bash
cd $RAPP_BASE/rapp-apps
catkin init
catkin config --install
catkin config --isolate-install
catkin build
