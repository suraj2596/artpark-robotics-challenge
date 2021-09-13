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

Please note the simulation is paused initially

Launching the simulation:


### Launching the simulation (Running docker will run automatically)

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch cerberus_gazebo combined_launch.launch # launch gazebo with gripper
```

On another terminal, run the docker and in the bash, run the following.

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch cerberus_navigation explore.launch
```

For bot to start moving and picking up trash items, run the following:

```bash
catkin build
source ~/catkin_ws/devel/setup.bash
rosrun trash_can_detector trash_dropoff
```

You shall see the bot picking up trash items and dropping them one by one.
