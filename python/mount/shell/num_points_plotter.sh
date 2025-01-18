#!/bin/bash

exec_pwd=$(cd $(dirname $0); pwd)

cd $exec_pwd/../pyscr

python3 num_points_plotter.py \
	--read_rosbag_path $HOME/rosbag/test.bag \
	--write_image_path $exec_pwd/../output/test.png \
	--target_pc_topic /OUSTER_1/points/downsampled \
    --interval_sec 10