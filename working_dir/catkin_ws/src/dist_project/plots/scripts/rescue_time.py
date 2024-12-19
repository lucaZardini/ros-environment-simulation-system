import rosbag
import matplotlib.pyplot as plt
import numpy as np


def seconds_to_minutes_seconds(total_seconds):

    # Calculate minutes and remaining seconds
    minutes = int(total_seconds // 60)
    seconds = total_seconds % 60

    return minutes, seconds


if __name__ == "__main__":

    # Path of the rosbag file to extract data from
    # In order to take all the necessary data I edit this line when necessary
    bagfile = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/quad_test.bag'
    # bagfile = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/bagfiles/more_tags.bag'

    # I edit this lane when necessary to extract data from a particular topic
    num_drones = 5
    init_topic = "/cmd_vel"
    start_topic = []
    finish_topic = ["/all_targets_found"]
    # finish_topic = ["/target_rescued"]
    for drone in range(num_drones):
        start_topic.append(f"/drone{drone}{init_topic}")

    # Data extraction
    # Packing of data iterating all messages stored
    with rosbag.Bag(bagfile, 'r') as bag:
        smallest_t = None
        for init_t in start_topic:
            for topic, msg, t in bag.read_messages(topics=[init_t]):
                start_t = t
                if smallest_t is None or t < smallest_t:
                    smallest_t = t
                break

    with rosbag.Bag(bagfile, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=finish_topic):
            finish_t = t

    elasped_time = finish_t - smallest_t
    minutes, seconds = seconds_to_minutes_seconds(elasped_time.secs)
    print(f"Total time to rescue all targets: {minutes} minutes and {seconds} seconds")
