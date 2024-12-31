import numpy as np
import rosbag
from scipy import interpolate
import matplotlib.pyplot as plt
import os


def compute_error(loc_data, gt_data):
    """
    Computes the estimation error between localization data and ground truth data.
    Uses interpolation to match timestamps and calculates the Euclidean distance.
    """
    errors = {}

    for drone_key, loc_points in loc_data.items():
        # Extract the drone identifier from the topic name (e.g., '/drone0/localization_data_topic' -> 'drone0')
        drone_id = drone_key.split('/')[1]

        # Find the corresponding ground truth data based on the drone ID
        gt_key = f"/{drone_id}/ground_truth/state"

        if gt_key in gt_data:
            # Synchronize data based on timestamps
            loc_points = np.array(loc_points)
            gt_points = np.array(gt_data[gt_key])

            # Ensure gt_positions is a 2D array with shape (n, 2)
            gt_timestamps = gt_points[:, 0]
            gt_positions = np.array([pt[1] for pt in gt_points])  # Extract the [x, y] pairs as a 2D array

            # Perform interpolation to match the localization data's timestamps
            interpolator = interpolate.interp1d(gt_timestamps, gt_positions, axis=0, fill_value="extrapolate")

            matched_errors = []
            for loc_time, loc_pos in loc_points:
                # Interpolate ground truth position for the current localization timestamp
                gt_pos = interpolator(loc_time)

                # Calculate Euclidean distance
                distance_error = np.linalg.norm(loc_pos - gt_pos)
                matched_errors.append(distance_error)

            errors[drone_key] = matched_errors

            # Print average error for this drone
            avg_error = np.mean(matched_errors)
            print(f"{drone_id}: Average Estimation Error = {avg_error:.3f}")

    return errors


# Path of the rosbag file to extract data from
bagroot = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/bagfiles/drones'
bagfile_name = 'sim1.bag'
bagfile = os.path.join(bagroot, bagfile_name)

# I edit this lane when necessary to extract data from a particular topic
num_drones = 5
estimated_topic = "/localization_data_topic"
ground_truth_topic = "/ground_truth/state"
position_estimated_topics = []
# finish_topic = ["/all_targets_found"]
ground_truth_topics = []
for drone in range(num_drones):
    position_estimated_topics.append(f"/drone{drone}{estimated_topic}")
    ground_truth_topics.append(f"/drone{drone}{ground_truth_topic}")

# Empty lists to contain all data
loc_data = {}
gt_data = {}

# Data extraction
# Packing of data iterating all messages stored
with rosbag.Bag(bagfile, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=position_estimated_topics):
        x = msg.position.x
        y = msg.position.y
        timestamp = t.to_sec()  # Use timestamp of message
        if topic in loc_data:
            loc_data[topic].append([timestamp, [x, y]])
        else:
            loc_data[topic] = [[timestamp, [x, y]]]

with rosbag.Bag(bagfile, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=ground_truth_topics):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        timestamp = t.to_sec()  # Use timestamp of message
        if topic in gt_data:
            gt_data[topic].append([timestamp, [x, y]])
        else:
            gt_data[topic] = [[timestamp, [x, y]]]

# Ensure data length consistency
assert len(loc_data) == len(gt_data)

# Compute errors for each drone
errors = compute_error(loc_data, gt_data)

# Optionally, plot results
for drone_key, error in errors.items():
    plt.plot(error, label=drone_key)

plt.title("Estimation Error Over Time")
plt.xlabel("Time Step")
plt.ylabel("Estimation Error (m)")
plt.legend()
plt.show()
# plt.savefig(f"error_plot/{bagfile_name}.png")

# Display lengths of data for each drone
for drone_id in range(num_drones):
    est_key = f"/drone{drone_id}{estimated_topic}"
    drone_loc_data = np.array(loc_data[est_key])
    gt_key = f"/drone{drone_id}{ground_truth_topic}"
    drone_gt_data = np.array(gt_data[gt_key])
    print(f"Len of drone {drone_id} loc data: {len(drone_loc_data)} and gt data: {len(drone_gt_data)}")
