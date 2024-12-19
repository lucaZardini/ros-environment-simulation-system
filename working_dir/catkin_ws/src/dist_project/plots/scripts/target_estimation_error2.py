

# Define the bag file to process

import rosbag
import matplotlib.pyplot as plt
import numpy as np


# Function to compute Euclidean distance (error) in 2D (using only x, y)
def compute_error(estimated_pos, ground_truth_pos):
    return np.linalg.norm(np.array(estimated_pos[:2]) - np.array(ground_truth_pos))  # Only use x and y for 2D distance



# Function to check if a new detection is within the radius of an existing target
def is_within_radius(new_position, existing_positions, radius=0.5):
    for position in existing_positions:
        if np.linalg.norm(np.array(new_position[:2]) - np.array(position)) <= radius:
            return True
    return False


# Load data from the rosbag file
def load_robot_detections(bagfile, found_targets_topic):
    robot_detections = []

    with rosbag.Bag(bagfile, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=found_targets_topic):
            robot_id = msg.robot_id
            timestamp = t.to_sec()  # Convert ROS timestamp to seconds
            position = [msg.x, msg.y, msg.z]  # Target position estimated by the robot
            robot_detections.append((robot_id, timestamp, position))

    return robot_detections


# Ground truth positions for the targets (example for three targets)
ground_truth_positions = {
    1: [-3.0, 3.0],  # Target 1 ground truth position
    2: [3.0, -2.0],  # Target 2 ground truth position
    3: [4.0, 2.0],  # Target 3 ground truth position
}

bagfile = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/quad_test.bag'

# The topic where robots publish the target detections
found_targets_topic = '/found_targets'

# Create a figure for the plots (one plot for each target)
fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(10, 15))  # Three targets, one plot per target

# Load robot detections from the bagfile
robot_detections = load_robot_detections(bagfile, found_targets_topic)

# Dictionary to track detected targets and their positions by each robot
detected_targets = {target_id: [] for target_id in ground_truth_positions}

# Iterate over each target and generate the plots
for target_id, target_pos in ground_truth_positions.items():
    ax = axes[target_id - 1]  # Select corresponding subplot

    # Filter the detections and associate them with the closest target
    target_detections = []
    for robot_id, timestamp, estimated_pos in robot_detections:
        # Find the closest ground truth target to the robot's estimated position
        closest_target_id = None
        min_distance = float('inf')

        # Find the closest target based on distance
        for current_target_id, ground_truth_pos in ground_truth_positions.items():
            distance = np.linalg.norm(np.array(estimated_pos[:2]) - np.array(ground_truth_pos))  # 2D distance
            if distance < min_distance:
                min_distance = distance
                closest_target_id = current_target_id

        # Debugging: Check the distance for each detection
        # print(
        #     f"Robot {robot_id} - Timestamp: {timestamp} - Estimated Position: {estimated_pos} - Closest Target: {closest_target_id} - Distance: {min_distance}")

        # Add the detection if it's within the radius of the closest target
        if min_distance <= 0.5 and closest_target_id == target_id:  # Only keep detections for the current target
            target_detections.append((robot_id, timestamp, estimated_pos))

    # Sort the detections by timestamp
    target_detections.sort(key=lambda x: x[1])

    # Initialize a list to store the errors over time
    errors = []
    timestamps = []

    # Track which robot discovered the target and which robot helped
    discovered_robot_id = None
    helper_robot_id = None
    helper_robot_detected_at = None

    for robot_id, timestamp, estimated_pos in target_detections:
        # Compute the error (Euclidean distance between the estimated position and ground truth)
        error = compute_error(estimated_pos, ground_truth_positions[target_id])
        errors.append(error)
        timestamps.append(timestamp)

        # Mark the first robot as the discovering robot
        if discovered_robot_id is None:
            discovered_robot_id = robot_id
            ax.annotate(f"Robot {robot_id} discovered", (timestamp, error), textcoords="offset points", xytext=(0, 5),
                        ha='center', color='green')

        # Mark the helper robot when it first detects the target
        if robot_id != discovered_robot_id and helper_robot_id is None:
            helper_robot_id = robot_id
            helper_robot_detected_at = timestamp
            ax.annotate(f"Robot {robot_id} helps", (timestamp, error), textcoords="offset points", xytext=(0, 5),
                        ha='center', color='red')
            ax.axvline(x=timestamp, color='red', linestyle='--', alpha=0.7)  # Vertical line for helper robot detection

    # Plot the error vs. time for this target
    if len(errors) > 0:  # Only plot if there are errors
        ax.plot(timestamps, errors, label=f"Target {target_id} Error", color='blue')

    # Add labels and title to the plot
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (meters)')
    ax.set_title(f'Estimation Error for Target {target_id}')
    ax.legend()

# Adjust layout and show the plot
plt.tight_layout()
plt.show()
