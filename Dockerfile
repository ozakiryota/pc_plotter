########## Pull ##########
FROM ros:noetic
########## User ##########
ARG home_dir="/home/user"
COPY copy/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN apt-get update && \
	apt-get install -y \
		gosu \
		sudo && \
	chmod +x /usr/local/bin/entrypoint.sh && \
	mkdir -p $home_dir
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
########## Non-interactive ##########
ENV DEBIAN_FRONTEND=noninteractive
########## ROS Setup ##########
RUN mkdir -p ~/catkin_ws/src && \
	cd ~/catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make" && \
	echo "source /opt/ros/noetic/setup.bash" >> $home_dir/.bashrc && \
	echo "source ~/catkin_ws/devel/setup.bash" >> $home_dir/.bashrc && \
	echo "export ROS_WORKSPACE=~/catkin_ws" >> $home_dir/.bashrc
## cmk
RUN echo 'function cmk(){(cd $ROS_WORKSPACE; catkin_make $@)}' >> $home_dir/.bashrc
########## Common Tool ##########
# RUN apt-get update && \
# 	apt-get install -y 
########## Python ##########
RUN apt-get update && \
	apt-get install -y \
		python3-tk \
		python3-pip && \
	pip3 install matplotlib
########## Initial Position ##########
WORKDIR $home_dir/catkin_ws/src/pc_plotter
CMD ["bash"]