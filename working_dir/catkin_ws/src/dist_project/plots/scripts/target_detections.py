import os
import rosbag
import numpy as np
import matplotlib.pyplot as plt


# Function to process drone detections and find new or existing targets
def process_detections(bagfile, ground_truth_positions, radius=0.5):
    targets = []  # List of detected targets with positions and associated robot ids
    robot_detections = []  # List of robot detections with their positions and robot_id

    with rosbag.Bag(bagfile, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/found_targets']):
            robot_id = msg.robot_id
            detection_position = np.array([msg.x, msg.y, msg.z])

            # Store robot detection data for plotting
            robot_detections.append((robot_id, detection_position))

            if len(targets) == 0:
                # No targets detected yet, create the first one
                targets.append([detection_position, [robot_id]])  # Append the position and associated robot_id
                continue

            # Calculate distances between the new detection and all existing targets
            distances = np.array([np.linalg.norm(detection_position - target[0]) for target in targets])

            # Check if the detection is within the threshold radius of any existing target
            if np.min(distances) <= radius:
                # If the detection is close enough, update the corresponding target
                nearest_target_idx = np.argmin(distances)
                targets[nearest_target_idx][0] = (targets[nearest_target_idx][0] * len(
                    targets[nearest_target_idx][1]) + detection_position) / (len(targets[nearest_target_idx][1]) + 1)
                targets[nearest_target_idx][1].append(robot_id)  # Add the robot_id to the target's list
            else:
                # If no close target is found, create a new target
                targets.append([detection_position, [robot_id]])

    return targets, robot_detections


bag_root = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/bagfiles'
bag_folder = "drones"
bag_experiment = "5_drones_2_rescue"

bagfiles = os.listdir(os.path.join(os.path.join(bag_root, bag_folder), bag_experiment))

ground_truth_positions = [
    np.array([-3.0, 3.0, 0.0]),  # Target 1
    np.array([3.0, -2.0, 0.0]),  # Target 2
    np.array([4.0, 2.0, 0.0]),  # Target 3
    np.array([-4.0, -2.0, 0.0])  # Target 4
]

# ground_truth_positions = [
#     np.array([-4.0, -3.0, 0.0]),  # Target 1
#     np.array([3.0, 3.5, 0.0])  # Target 2
# ]


for bagfile_name in bagfiles:
    bagfile = os.path.join(os.path.join(os.path.join(bag_root, bag_folder), bag_experiment), bagfile_name)
    # Process the detections and get the updated targets
    targets, robot_detections = process_detections(bagfile, ground_truth_positions, radius=1.5)

    # Plotting
    fig, ax = plt.subplots()

    # Plot ground truth targets
    for idx, position in enumerate(ground_truth_positions):
        ax.scatter(position[0], position[1], marker='*', color='green', label=f"Ground Truth Target {idx + 1}", s=100, zorder=2)

    # Plot robot detections with smaller, semi-transparent circles
    for robot_id, position in robot_detections:
        ax.scatter(position[0], position[1], alpha=0.5, s=20, zorder=1)

    # Plot the targets
    for idx, (position, robot_ids) in enumerate(targets):
        ax.scatter(position[0], position[1], marker='x', color='red', label=f"Target {idx + 1}")

    # Customize the plot
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Target Estimation and Robot Detections')
    ax.legend(loc='best')

    plt.show()
    # plt.savefig(f"{bagfile_name}.png")

    if len(targets) != len(ground_truth_positions):
        print(f"Number of targets detected: {len(targets)}")
        print(f"Number of ground truth targets: {len(ground_truth_positions)}")
        continue

    targets = sorted(targets, key=lambda x: (x[0][0], x[0][1]))
    ground_truth_positions = sorted(ground_truth_positions, key=lambda x: (x[0], x[1]))


    estimation_error = []
    for target, gt_target in zip(targets, ground_truth_positions):
        error = np.linalg.norm(target[0] - gt_target)
        estimation_error.append(error)

    print(f"Estimation error: {estimation_error}")
    print(f"Mean estimation error: {np.mean(estimation_error)}")
    print(f"Median estimation error: {np.median(estimation_error)}")
    print(f"Max estimation error: {np.max(estimation_error)}")
    print(f"Min estimation error: {np.min(estimation_error)}")

    # # Print results for each target
    # for idx, (position, robot_ids) in enumerate(targets):
    #     print(f"Target {idx + 1}: Position = {position}, Detected by robots {robot_ids}")
