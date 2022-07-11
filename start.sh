#!/bin/bash
cd ~/A1_jetson_ojima
./momo_arm64.run
sleep 20
~/ojima-librealsense/build/example/pointcloud/ojima_pointcloud_A1 &
./udp_client
