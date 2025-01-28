#!/bin/bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 1000000 #loopback on
sudo ip link set up can0

sudo chmod a+rw /dev/ttyS0
echo "Serial port open for communication"

