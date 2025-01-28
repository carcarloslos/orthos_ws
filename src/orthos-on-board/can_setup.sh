#!/bin/bash

sudo busybox devmem 0x0c303000 32 0x0000C400
sudo busybox devmem 0x0c303008 32 0x0000C458
sudo busybox devmem 0x0c303010 32 0x0000C400
sudo busybox devmem 0x0c303018 32 0x0000C458

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set can0 type can bitrate 500000
sudo ip link set can1 type can bitrate 1000000
sudo ip link set up can0
sudo ip link set up can1
sudo ip link set can0 txqueuelen 15
sudo ip link set can1 txqueuelen 15

sudo chmod 777 /dev/ttyTHS0

sleep 15

. /opt/ros/noetic/setup.sh
. /home/orthos/orthos_ws/devel/setup.sh

export ROS_MASTER_URI=http://192.168.2.10:11311
export ROS_IP=192.168.2.10

sleep 15

roslaunch orthos_driver robota.launch

sleep 30

exit 0
