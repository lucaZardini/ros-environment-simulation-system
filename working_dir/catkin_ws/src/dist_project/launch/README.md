# Launch files

This folder contains ROS launch files, which are used to start nodes and set up configurations for running ROS applications.

Launch files have the `.launch` extension and are written in XML format.

- `target_estimation.launch`: Main launch file to start the target estimation simulation.
- `search_and_rescue.launch`: Main launch file to start the search and rescue simulation, by spawning the drones and targets in the room.
- `drone_controller.launch`: Launch file to initialize the controller for the drones.
- `run_search_and_rescue.launch`: Launch file to initialize the drone's nodes and start the simulation.
- `start_drone_nodes.launch`: Launch file to start the drone's nodes, used internally by `run_search_and_rescue.launch`.
- `start_unicycle_nodes.launch`: Launch file to start the unicycle's nodes, used internally by `target_estimation.launch`.
- `plotjuggler.launch`: Launch file to start the PlotJuggler software for data visualization.

To run a launch file, use the following command:

```sh
roslaunch dist_project <launch_file_name> [arguments]
```