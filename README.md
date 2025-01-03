# Distributed project

The aim of this project was to develop a environment in ROS and Gazebo for the deployment and testing of distributed tasks.

The original project, developed by [Marco Sterlini](https://github.com/Marcosterlo) consists in a 3d target estimation made by many unicycles.

This project starts from the clone of the [Distributed Project](https://github.com/Marcosterlo/Distributed_project) and a new task, the search and rescue task, has been developed.

So, this environment simulation system provides the possibility to run a 3d target estimation task spawning a desired number 
of unicycles and one target in a room, and a search and rescue task where a variable set of drones has to find and rescue a desired number of targets in a room.

The quadrotor drone is a model implemented in the [hector-quadrotor-noetic](https://github.com/RAFALAMAO/hector-quadrotor-noetic) Github repository.

## Project structure

This project follows the catkin workspace folder hierarchy as recommended by ROS. For more information on the catkin workspace structure, please refer to the [ROS wiki](http://wiki.ros.org/catkin/workspaces).

To support the quadrotor drone, the clone of the entire repository and its build is necessary. The repository must be cloned in the `src` folder of the catkin workspace.
More in detail, the `geographic_info`, `hector_quadrotor_noetic`, `teleop_twist_keyboard` and `unique_identifier` ROS package has been added.

Instead, the `dist_project` package contains all the nodes and logic of the project. 
In particular, this package is structured as follows:

- `bagfiles/`: Where simulation bagfiles are stored.
- `coordinates/`: Here there are the csv files containing the initial spawn coordinates of the UWB tags, unicycles, drones and targets. These files can be edited to dynamically change the number and position of the entities of the simulation.
- `launch/`: ROS launch files to initialize the simulation.
- `mesh/`: Contains the stl files used to render the robot in the Gazebo simulation.
- `models/`: Contains the models folders created in Gazebo used to spawn the room and the target in the simulation (for target estimation).
- `msg/`: Custom ROS messages definition.
- `plotjuggler/`: Contains the settings file for the third party node "plotjuggler" used to better plot data from the topics both from bag file and in real time.
- `plots/`: Contains the scripts used to create the plots that have been used in the final report.
- `scripts/`: Here are all the executable python scripts, they will manage the setup of the environment and all the logic of the ROS nodes
- `urdf/`: URDF files for robot description. 


## Requirements

- Docker installed

## Installation 
There are many requirements necessary, in order to simplify the execution the whole project has been developed inside a Docker container in order to guarantee compatibility along all platforms. 

This github repository must be cloned into a folder on the user computer with:
```sh
git clone https://github.com/Marcosterlo/Distributed_project.git
```


To run the Docker container for this project, use the following command. You will need to replace `/path/to/your/local/folder` with the local folder where this GitHub repository has been cloned to.

```sh
sudo docker run --name ubuntu_ros -v /tmp/.X11-unix/:/tmp/.X11-unix/ -v /home/luca/Documenti/Università/AIS/3Semestre/Distributed_robot_system/ros-environment-simulation-system:/home/marco/shared --env="DISPLAY=$DISPLAY" --privileged --shm-size 2g --rm -i -t --user=marco --workdir=/home/marco marcosterlo/ubuntu:ros bash
```

To attach to the container with another terminal, use the following command:

```sh
docker exec -it ubuntu_ros bash
```

If you want to use root privileges, the user is `marco` and the password is `marco`.

## Usage

### Target estimation task

Once inside the docker container, in order to start the target estimation simulation run the following command:

```sh
roslaunch dist_project target_estimation.launch
```
It will also open a window containing the point of view of the camera of one of the unicycles and a open source software called plotjuggler. See `plotjuggler` folder for further instructions

### Search and rescue task

This task consists in spawning a variable number of drones and targets in a room. For each drone, a controller must be run in order to make the drone move and search for the targets.
Then, the drone's nodes must be run in order to simulate the localization, the blob detection and the target rescue.

Each of the three parts has to be run in a different terminal to avoid overloading the system.

Once inside the docker container, in order to start the search and rescue simulation run the following command:

```sh
roslaunch dist_project search_and_rescue.launch
```

This command will spawn the drones and the targets in the room. Then, to initialize the controller, run the following command:

```sh
roslaunch dist_project drone_controller.launch num_drones:=<number_of_drones>
```

where `<number_of_drones>` is the number of drones spawned.

Finally, to initialize the drone's nodes, run the following command:

```sh
roslaunch dist_project run_search_and_rescue.launch num_drones:=<number_of_drones>
```

where `<number_of_drones>` is the number of drones spawned.


### Docker image

The starting docker image can be found at the following link
```
https://hub.docker.com/r/marcosterlo/ubuntu/tags
```