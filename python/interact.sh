#!/bin/bash

xhost +

image="pc_plotter"
tag="python"
exec_pwd=$(cd $(dirname $0); pwd)
home_dir="/home/user"
catkin_ws_src=$home_dir/catkin_ws/src

docker run \
	-it \
	--rm \
	-e local_uid=$(id -u $USER) \
	-e local_gid=$(id -g $USER) \
	-e "DISPLAY" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--privileged \
	--net=host \
	-v $exec_pwd/mount:$catkin_ws_src/$image \
	-v $exec_pwd/mount/shell:$catkin_ws_src/$image/shell \
	-v $exec_pwd/mount/output:$catkin_ws_src/$image/output \
	-v $exec_pwd/mount/launch:$catkin_ws_src/$image/launch \
	-v $exec_pwd/../rosbag:$home_dir/rosbag:ro \
	$image:$tag