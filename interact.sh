#!/bin/bash

xhost +

image="pc_plotter"
tag="latest"
exec_pwd=$(cd $(dirname $0); pwd)
home_dir="/home/user"

docker run \
	-it \
	--rm \
	-e local_uid=$(id -u $USER) \
	-e local_gid=$(id -g $USER) \
	-e "DISPLAY" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--privileged \
	--net=host \
	-v $exec_pwd/mount:$home_dir/catkin_ws/src/$image \
	-v $exec_pwd/rosbag:$home_dir/rosbag:ro \
	$image:$tag