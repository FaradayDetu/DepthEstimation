#!/bin/bash

echo "Starting roscore..."
gnome-terminal -e "roscore"
echo "done!"
sleep 5
gnome-terminal -e "rosparam set enable_statistics true"
echo "done setting params true"
gnome-terminal -e "rosrun joy joy_node"
echo "joystick node started"
sleep 2
echo "Starting ZED node..."
gnome-terminal -e "roslaunch zed_wrapper zed.launch"
echo "Done!"
sleep 5
echo "Starting get_image.py..."
gnome-terminal -e "rosrun ros_deep_learning get_image.py"
echo "Done!"
sleep 5
echo "Starting detectnet..."
gnome-terminal -e "rosrun ros_deep_learning detectnet /detectnet/image_in:=image_topic_l_publisher _model_name:=coco-bottle"
echo "Done!"
sleep 5
echo "Starting depth inference..."
gnome-terminal -e "roslaunch /home/nvidia/catkin_ws/src/stereo_dnn_ros/launch/ResNet18_2D_fp32.launch"
echo "Done!"
sleep 10
echo "Starting bottle depth node"
gnome-terminal -e "rosrun ros_deep_learning depth_estimation.py depth_statistics_32"
echo "Done!"
sleep 5
echo "Starting controller..."
gnome-terminal -e "rosrun ros_deep_learning runner.py"
echo "Done!"
sleep 5
gnome-terminal -e "rqt_graph"

