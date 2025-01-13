#!/bin/bash

exec_pwd=$(cd $(dirname $0); pwd)

cd $exec_pwd/../pyscr

python3 pc_plotter.py \
	--read_rosbag_path $HOME/rosbag/test.bag \
	--target_pc_topic /OUSTER_1/points/downsampled \
	--num_show 5 \
    --interval_sec 10 \
	--target_image_topic /usb_cam_node_c920e/image_raw/compressed
