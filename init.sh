#!/bin/bash

echo ""
echo "Initialize the grasp_ws"
echo ""

source ~/.bashrc

echo ""
echo "Install required ros packages..."
echo ""
rosdep install --from-paths src --ignore-src -r -y


echo ""
echo "Compiling..."
echo ""
catkin_make

source devel/setup.bash


