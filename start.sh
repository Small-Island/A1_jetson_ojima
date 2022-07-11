#!/bin/bash
cd ~/A1_jetson_ojima
./momo_arm64.run
sleep 20
~/ojima-librealsense/build/examples/pointcloud/ojima-pointcloud-A1 &
./udp_client
