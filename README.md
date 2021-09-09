# team cerberus
### Docker image

Assumes `docker` and `nvidia-docker2` are installed.

```bash

# build image once
sudo docker build -t cerberus .

# run container and launch file
sudo chmod a+x run_cerberus.sh
sudo ./run_cerberus.sh

# for bash access in a separate terminal
sudo docker exec -it cerberus bash
```

# About cerberus

This bot uses scissor mechanism to expand/compress. Various payloads (gripper, spray, scrubbers) are attached as its rotating end-effector. 
Based on the task, the end-effector rotates to choose the payload.

the entire scissor mechanism can movel along up and down a spine.

This spine can rotate in the horizontal plane while the scissor can rotate in the vertical plane. (overactated)

The entire bot is placed on 

Please note the simulation is paused initially

Launching the simulation:


```bash
source ~/catkin_ws/devel/setup.bash
roslaunch cerberus_gazebo combined_launch.launch # launch gazebo with gripper

roslaunch cerberus_navigation mavigation.launch # to build the map

roslaunch cerberus_gazebo move_base.launch # to begin various tasks

```


























### Launching the simulation
For gripper action, use the main branch.

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch cerberus_gazebo combined_launch.launch # launch gazebo with gripper
```

On another terminal, run the docker and in the bash, run the following.

```bash
source ~/catkin_ws/devel/setup.bash
rostopic pub -1 /artpark/grip_signal std_msgs/Float64 "data: 0.0" 
rostopic pub -1 /artpark/drop_signal std_msgs/Float64 "data: 0.0" 
```

For navigation stack, use the navload branch,

```bash
catkin build
source ~/catkin_ws/devel/setup.bash
roslaunch cerberus_gazebo setup.launch
```

