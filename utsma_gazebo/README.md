# How to launch Gazebo simulation:

## Requirement:
### Software System (this is my current set up):
*  Ubuntu 20.04
*  Ros Noetic

### Simulation Environment
  1. Prerequisites:
  Before diving into the project setup, ensure you have the following prerequisites installed:****
     *  ROS (Robot Operating System): I refer ROS Noetic with Ubuntu 20.04.
     *  Gazebo: The primary simulation tool we use.

  2. Running the Project:
  *  In the first terminal, Launching the Simulation:
  ```
  roslaunch utsma_gazebo small_track.launch
  ```

  3. To control the car:
     You should publish to topic '/cmd_vel_out' to control the car. There are 5 variables in this topic but you only need to care about 2 which are 'steering angle' and 'acceleration'. The steering angle is used to control the turning angle of the car while the acceleration is use to control speed.
