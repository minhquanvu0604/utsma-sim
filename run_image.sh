#!/bin/bash

# Make sure you put this file and the img in the same directory
docker image load -i utsma_demo_img

# Give xhost permission to display GUI
xhost +local:*

##
docker run --privileged \
           --detach \
           --name utsma_demo_sim \
           --network "host" \
           -e ROS_MASTER_URI="http://localhost:11311" \
           -e DISPLAY=$DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           utsma_demo_img:latest \
           bash -c "source /opt/ros/noetic/setup.bash && source /utsma_ws/devel/setup.bash && \
           roslaunch utsma_gazebo simulation.launch"

docker run --privileged \
           --detach \
           --name utsma_demo_path_planning \
           --network "host" \
           -e ROS_MASTER_URI="http://localhost:11311" \
           -e DISPLAY=$DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           utsma_demo_img:latest \
           bash -c "source /opt/ros/noetic/setup.bash && source /utsma_ws/devel/setup.bash && \
           sleep 10 && \
           rosrun d_triang utsma_path_planner"
           
docker run --privileged \
           --detach \
           --name utsma_demo_code_viewer \
           --network "host" \
           -e ROS_MASTER_URI="http://localhost:11311" \
           -e DISPLAY=$DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           utsma_demo_img:latest \
           bash -c "sleep infinity"
           
