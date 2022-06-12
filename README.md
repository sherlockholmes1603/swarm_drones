# swarm_drones

So this is a project on swarm drones using ardupilot and gazebo

Problems I am currently facing:-
  - In the launch file drone_connect.launch two drones should connect with namespaces "/drone1" and "/drone2" respectively but only "/drone1" is getting connected. We can the rostopics and rosservices of "/drone2" but the connection is not working.
  - To change the origin in local position of both drones with the origin of gazebo
  - To make it follow a proper circle


## How to setup The enviornment

First you nedd to install ROS (Robotics Operating System) whose tutorials you can find here

After which you need to setup ardupilot on your system whose tutorials you can find [here](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md)


Now you need to setup the world for which you can follow the following tutorial [here](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/swarming_ardupilot.md)


## Launch the enviornment 

Open up a terminal and write the following commands

```bash 
roslaunch swarm_drones runway.launch
```

Openup a new terminal and write the command

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0
```

Openup a new terminal and write the command

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I1
```

Openup a new terminal and write the command

```bash
roslaunch swarm_drones drone_connect.launch
```

Now when you will do rostopic list you will find all the commands for both the drones i.e. "/drone1" and "/drone2". But the commands for "/drone2" are not working and it is also not connected which is problem I am trying to debug

For the drone to follow a circle run the following command

```bash
rosrun swarm_drones drone_circle.py "/drone1"
```

You will see the drone move like this 


https://user-images.githubusercontent.com/77875542/173238605-d7586ffc-56a3-4f93-80e6-c720ea509f5b.mp4

