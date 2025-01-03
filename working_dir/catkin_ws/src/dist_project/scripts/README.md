# Scripts

These python scripts hold all the nodes and logic of the project.
The common files are in the folder, while the task specific files are in their respective folders.

## `gazebo_init.py`
### Key Features
This node dynamically spawns tags, targets, robots and the room model in the Gazebo simulation environment. It also recursively initialize all the other ROS nodes for each robot based on the specified number of lines in the csv file.
It also continuosly publishes static transforms for the tags, aiding in localization tasks.

### Topics
The script interacts with the following topics:
- **Publication**: 
  - Static transforms are published for tags to the `/tf_static` topic using `tf2_ros.StaticTransformBroadcaster`.

### Functions
- **`spawn_tag(x, y, id)`**: Spawns a tag at the given coordinates using a URDF model.
- **`spawn_target(x, y, id)`**: Spawns a target at the given coordinates using an SDF model.
- **`spawn_unicycle(x, y, id)`**: Spawns an unicycle at the given coordinates using a URDF model.
- **`spawn_drone(x, y, id)`**: Spawns a drone at the given coordinates using a URDF model.
- **`spawn_room()`**: Spawns the room model in the simulation using an SDF file.
- **`publish_static_transform(x, y, id)`**: Publishes the static transform for a tag.

## `initialize_robot.py`

### Key Features
This script initializes the robots. For unicycles, they are already ready to move, while drones need to take off and hover at a certain height.

### Topics
The script interacts with the following topics:
- **Subscription**:
  - `cmd_vel` (Twist): Subscribes to the velocity commands to control the robot's movement and reach a desired height (drone).
- **Publication**:
  - `init` (Pose): Publishes the initialization signal to indicate the robot is ready to start the mission.

# Target estimation task

## `uwb_dist_sim.py`

### Key Features
It dynamically retrieves and stores the positions of UWB anchors in the simulation environment, ti also computes the distance between the robot and each UWB anchor, adding realistic noise to the measurements, finally it publishes the calculated distances and corresponding anchor IDs to a ROS topic.

### Topics
The script interacts with the following topics:
- **Subscription**: 
  - `/ground_truth/state`: Subscribes to this topic to get the robot's current ground truth position.
- **Publication**:
  - `uwb_data_topic`: Publishes UWB distance data using a custom message type `uwb_data`.

## Functions
- **`get_anchor_pos()`**: Retrieves and stores the positions of UWB anchors using the `tf` package.
- **`subscribe_data(data)`**: Callback function for the ground truth subscriber, updates the robot's position, and triggers distance calculation.
- **`uwb_sim()`**: Calculates distances from the robot to each UWB anchor and calls the publish function.
- **`calculate_distance(uwb_pose)`**: Computes the distance between the robot and a given UWB anchor position, adding noise to simulate measurement errors.
- **`publish_data(ids, distances)`**: Publishes the calculated distances and anchor IDs to the `uwb_data_topic`.


## `blob_detector.py`

### Key Features
It identifies yellow spheres in the image stream using color thresholding and blob detection, then it publishes the coordinates of detected blobs to a ROS topic and finally it visualizes detected blobs and the search window on the processed image.

### Topics
The script interacts with the following topics:
- **Subscription**:
  - `camera/image_raw`: Subscribes to the raw image stream from the robot's camera.
- **Publication**:
  - `target/image_blob`: Publishes the processed image with detected blobs.
  - `target/point_blob`: Publishes the coordinates of the detected blob.

### Functions
- **`blob_detect(self, image, hsv_min, hsv_max, blur=0, blob_params=None, search_window=None)`**: Detects blobs in the given image based on HSV thresholds and blob detection parameters.
- **`draw_keypoints(self, image, keypoints, line_color=(0,0,255))`**: Draws detected blobs on the image.
- **`draw_window(self, image, window_adim, color=(255,0,0), line=5)`**: Draws the search window on the image.
- **`draw_frame(self, image, dimension=0.3, line=2)`**: Draws X and Y axes on the image for reference.
- **`apply_search_window(self, image, window_adim=[0.0, 0.0, 1.0, 1.0])`**: Applies the search window mask to the image.
- **`blur_outside(self, image, blur=5, window_adim=[0.0, 0.0, 1.0, 1.0])`**: Blurs the regions outside the search window.
- **`get_blob_relative_position(self, image, keyPoint)`**: Calculates the relative position of a blob in the image frame.
- **`set_threshold(self, thr_min, thr_max)`**: Sets the HSV threshold values.
- **`set_blur(self, blur)`**: Sets the blur value.
- **`set_blob_params(self, blob_params)`**: Sets the blob detection parameters.
- **`callback(self, data)`**: Callback function for the image subscriber, processes the image to detect blobs and publishes the results.

## `kalman_localization.py`

### Key Features

The script dynamically retrieves the positions of UWB anchors from the simulation, then it utilizes a Kalman Filter to predict and correct the robot's state, integrating odometry and UWB sensor data, it processes Odometry messages to predict the robot's movement based on velocity and angular data, it incorporates UWB sensor measurements to correct the robot's state by comparing expected distances with actual measurements, the estimated position and orientation, along with the associated uncertainties, are published to the ROS topic `localization_data_topic` and finally it notifies the motion planner to start once the Kalman Filter has stabilized its estimates.

### Topics
The script interacts with the following topics:
- **Subscription:**
  - `odom` (Odometry): Receives robot's movement data for prediction.
  - `uwb_data_topic` (uwb_data): Receives UWB sensor measurements for correction.

- **Publication:**
  - `localization_data_topic` (Pose): Publishes the estimated robot state including position, orientation, and uncertainties.
  - `init_move` (Pose): Signals the motion planner to start moving once the filter's estimates are reliable.

### Functions
- **`prediction_step(self, odom_data: Odometry)`**: Updates and predicts the robot’s state based on Odometry data and publishes the estimated state.
- **`correction_step(self, uwb_data: uwb_data)`**: Refines the robot’s state estimate using UWB data and notifies the motion planner when ready.
- **`subscribe_odom_data(self, odom_data: Odometry)`**: Callback for Odometry data, calls `prediction_step` to update the robot’s state.
- **`subscribe_uwb_data(self, uwb_data: uwb_data)`**: Callback for UWB data, calls `correction_step` to adjust the robot’s state estimate.
- **`publish_data(self, pose_x, pose_y, pose_yaw)`**: Publishes the estimated robot state and uncertainties to the ROS topic.
- **`get_anchors_pos(self)`**: Retrieves and updates UWB anchor positions from the simulation environment.

## `motion_planner.py`

### Key Features
This script manages robot behavior using a finite state machine (FSM), it uses PID control for target orientation and movement and it publishes velocity commands and target height data.

### Topics
The script interacts with the following topics:
- **Subscription:**
  - `target/point_blob` (Point): Receives target position data to adjust movement.
  - `init_move` (Pose): Receives a signal to start the FSM process.

- **Publication:**
  - `cmd_vel` (Twist): Publishes velocity commands to control the robot’s movement.
  - `target_height` (Point): Publishes the target height once reached.

### Functions
- **`init_callback(self, data)`**: Changes FSM state to "SEARCH" upon receiving initialization signal.
- **`blob_callback(self, data)`**: Updates FSM state based on detected target position and adjusts movement.
- **`move_towards_target(self, error)`**: Computes PID control and updates robot movement towards the target.
- **`publish_velocity(self, linear_x, angular_z)`**: Publishes movement commands to `cmd_vel`.
- **`execute_state_machine(self)`**: Executes FSM logic to handle state transitions and actions.

## `target_estimator.py`

### Key Features
This script estimates the target's position and height using data from multiple robots. It publishes target estimates and robot tags to facilitate collaborative localization.

### Topics
The script interacts with the following topics:
- **Subscription:**
  - `target_height` (Point): Receives target height data from local sensors.
  - `localization_data_topic` (Pose): Receives robot position and orientation data.
  - `/processing_data` (robot_data): Receives tags from other robots for target estimation.

- **Publication:**
  - `/processing_data` (robot_data): Publishes robot data to share with other robots.
  - `target_estimate` (Pose): Publishes the estimated target position and height.

### Functions
- **`target_callback(self, data)`**: Updates the target height from local sensor data.
- **`localization_callback(self, actual_pos)`**: Updates robot position and orientation; starts publishing if target height is available.
- **`least_square_estimation(self, slopes, sigma_slopes, y_intercepts, sigma_y_intercepts)`**: Estimates target position using least squares method.
- **`gather_callback(self, data)`**: Processes tags from other robots and computes target position based on collaborative data.

# Search and rescue task

## `gps_to_coordinate.py`

### Key Features
This script converts GPS coordinates to Cartesian coordinates, providing cartesian data used for localize the drone.

### Topics
The script interacts with the following topics:
- **Subscription:**
  - `fix` (NavSatFix): Receives GPS coordinates to convert to Cartesian.

- **Publication:**
  - `gps_cartesian_point` (Point): Publishes Cartesian coordinates for drone localization.

### Functions
- **`subscribe_gps_data(self, data)`**: Converts GPS coordinates to Cartesian and publishes the result.
- **`wgs84_to_ecef(self, lat, lon, h)`**: Converts WGS84 geodetic coordinates to ECEF.
- **`ecef_to_local(self, ecef, ecef_ref, lat_ref, lon_ref)`**: Converts ECEF coordinates to local XYZ.

## `kalman_localization_3d.py`

### Key Features
This script estimates the drone's position and orientation in 3D space using a Kalman Filter, integrating GPS and IMU data, it predicts the drone's state based on IMU data and corrects the state using GPS data, it publishes the estimated position and orientation to a ROS topic.

### Topics
The script interacts with the following topics:
- **Subscription:**
  - `raw_imu` (Imu): Receives IMU data for prediction.
  - `fix_velocity` (Vector3Stamped): Receives velocity data for prediction.
  - `gps_cartesian_point` (Point): Receives GPS data for correction.
- **Publication:**
  - `localization_data_topic` (Pose): Publishes the estimated drone state.

### Functions
- **subscriber callbacks**: IMU and velocity data are used to predict the drone's state, while GPS data is used to correct the state.
- **`prediction_step(self, imu_data: Imu, velocity_data: Vector3Stamped)`**: Updates and predicts the drone’s state based on IMU and velocity data.
- **`correction_step(self, gps_data: Point)`**: Refines the drone’s state estimate using GPS data.
- **`publish_data(self, pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw)`**: Publishes the estimated drone state to the ROS topic.

## `image_processing_3d.py`
### Key Features
It identifies yellow spheres in the image stream using color thresholding and blob detection, then it publishes the coordinates of detected blobs to a ROS topic and finally it visualizes detected blobs and the search window on the processed image.

### Topics
The script interacts with the following topics:
- **Subscription**:
  - `downward_cam/camera/imag`: Subscribes to the raw image stream from the robot's camera.
- **Publication**:
  - `target/image_blob`: Publishes the processed image with detected blobs.
  - `target/point_blob`: Publishes the coordinates of the detected blob.

### Functions
- **`blob_detect(self, image, hsv_min, hsv_max, blur=0, blob_params=None, search_window=None)`**: Detects blobs in the given image based on HSV thresholds and blob detection parameters.
- **`draw_keypoints(self, image, keypoints, line_color=(0,0,255))`**: Draws detected blobs on the image.
- **`draw_window(self, image, window_adim, color=(255,0,0), line=5)`**: Draws the search window on the image.
- **`draw_frame(self, image, dimension=0.3, line=2)`**: Draws X and Y axes on the image for reference.
- **`apply_search_window(self, image, window_adim=[0.0, 0.0, 1.0, 1.0])`**: Applies the search window mask to the image.
- **`blur_outside(self, image, blur=5, window_adim=[0.0, 0.0, 1.0, 1.0])`**: Blurs the regions outside the search window.
- **`get_blob_relative_position(self, image, keyPoint)`**: Calculates the relative position of a blob in the image frame.
- **`set_threshold(self, thr_min, thr_max)`**: Sets the HSV threshold values.
- **`set_blur(self, blur)`**: Sets the blur value.
- **`set_blob_params(self, blob_params)`**: Sets the blob detection parameters.
- **`callback(self, data)`**: Callback function for the image subscriber, processes the image to detect blobs and publishes the results.

## `motion_planner_3d.py`

### Key Features
This script manages the movement of the drones in the 3D space using a finite state machine (FSM), it contains the rescue logic and the termination of the task.

### Topics
The script interacts with the following topics:
- **Subscription:**
  - `target/point_blob` (Point): Receives target position data to adjust movement.
  - `init` (Pose): Receives a signal to start the FSM process.
  - `localization_data_topic` (Pose): Receives the estimated drone state.
  - `/found_targets` (target_data): Receives target found by the drones.
  - `/total_found_targets` (targets_data): Receives the targets found by the drones.
  - `/total_rescued_targets` (targets_data): Receives the rescued targets by the drones.
  - `/neighbors` (target_data): Receives the neighbors position.
  - `/ready_to_rescue` (target_data): Receives the message that a drone is waiting for support and it is ready to rescue the target.
  - `/target_assignment` (target_assignment_data): Receives the target assignment message.
  - `/all_targets_found` (Bool): Receives the message that all the targets have been found.
  - `/target_rescued` (target_data): Receives the message that a target has been rescued.
  - `/gazebo/model_states` (ModelStates): Receives the model states from Gazebo.
- **Publication:**
  - `cmd_vel` (Twist): Publishes velocity commands to control the drone’s movement.
  - `/found_targets` (target_data): Publishes the target found by the drone.
  - `/total_found_targets` (targets_data): Publishes the targets found by the drones.
  - `/total_rescued_targets` (targets_data): Publishes the rescued targets by the drones.
  - `/neighbors` (target_data): Publishes the neighbors position.
  - `/ready_to_rescue` (target_data): Publishes the message that a drone is waiting for support and it is ready to rescue the target.
  - `/target_assignment` (target_assignment_data): Publishes the target assignment message.
  - `/all_targets_found` (Bool): Publishes the message that all the targets have been found.
  - `/target_rescued` (target_data): Publishes the message that a target has been rescued.

### Functions

The functions are documented inside the code, to avoid redundancy in the documentation. Here, I briefly describe the main behaviour:

when the init signal is received, the drone starts moving in the 3d space by randomly choosing a point and moving towards it.
During its movement, it scans down to find the target. The drone change its searching state in three cases:
1. It founds a target
2. It receives a message from another drone that assigns it a target 
3. The mission is completed

In case 1 and 2, the drone moves towards the target. Once it reaches the target, it checks if the drones ready to rescue it are enough.
If they are enough, the drone rescues the target and sends a message to the other drones that the target has been rescued, otherwise 
it sends a message to the other drones that it is waiting for support.
Once the target is rescued, the drone goes back to the searching state if there are still targets to find, otherwise it goes to the mission completed state.

### Assumptions

1. The number of drones must be higher than the number of targets to rescue. In particular
```python
len(num_drones) >= len(num_targets) * (number_of_drones_for_rescue - 1) + 1
```
where `number_of_drones_for_rescue` is the number of drones needed to rescue a target. This is a parameter that can be set launching the `run_search_and_rescue.launch` file.

2. When a drone detects a target, it will try to rescue it if it is not rescuing another drone. If the drone is rescuing another drone, it will ignore the detected target.

3. The message sent to assign drones to rescue target is sent to the nearest drones to the target. It continues to check the nearest drones
and send the message until the number of drones needed to rescue the target is reached and arrive to the target.
