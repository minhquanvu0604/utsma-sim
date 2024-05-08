# PowerShell script version

# Make sure you put this file and the img in the same directory
#docker image load -i utsma_demo_img

# Give xhost permission to display GUI
#& 'C:\Program Files\VcXsrv\xhost.exe' +local:*

# Run the Docker container for simulation
docker run --privileged `
           --detach `
           --name utsma_demo_sim `
           --network "host" `
           -e ROS_MASTER_URI=http://localhost:11311 `
           -e DISPLAY=host.docker.internal:0.0 `
           utsma_dtriang_img:latest `
           bash -c "source /opt/ros/noetic/setup.bash && source /utsma_ws/devel/setup.bash && roslaunch utsma_gazebo simulation.launch"

# Run the Docker container for path planning
docker run --privileged `
           --detach `
           --name utsma_demo_path_planning `
           --network "host" `
           -e ROS_MASTER_URI=http://localhost:11311 `
           -e DISPLAY=host.docker.internal:0.0 `
           utsma_dtriang_img:latest `
           bash -c "source /opt/ros/noetic/setup.bash && source /utsma_ws/devel/setup.bash && sleep 10 && rosrun d_triang utsma_path_planner"

docker run --privileged `
           --detach `
           --name utsma_demo_code_viewer `
           --network "host" `
           -e ROS_MASTER_URI=http://localhost:11311 `
           -e DISPLAY=host.docker.internal:0.0 `
           utsma_dtriang_img:latest `
           bash -c "sleep infinity"