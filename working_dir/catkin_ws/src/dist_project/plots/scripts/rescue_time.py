import rosbag
import os


def seconds_to_minutes_seconds(total_seconds):

    # Calculate minutes and remaining seconds
    minutes = int(total_seconds // 60)
    seconds = total_seconds % 60

    return minutes, seconds


if __name__ == "__main__":

    # Path of the rosbag file to extract data from
    bag_root = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/bagfiles/drones'
    bag_folder = "noise_communication_range"
    bag_experiment = "noise_no_comm_range"

    bagfiles = os.listdir(os.path.join(os.path.join(bag_root, bag_folder), bag_experiment))

    num_drones = 5
    init_topic = "/cmd_vel"
    start_topic = []
    # finish_topic = ["/all_targets_found"]
    finish_topic = ["/target_rescued"]
    rescue_times = []
    for drone in range(num_drones):
        start_topic.append(f"/drone{drone}{init_topic}")

    # Data extraction
    # Packing of data iterating all messages stored
    for bagfile in bagfiles:
        print(f"Extracting data from {bagfile}")
        bagfile = os.path.join(os.path.join(os.path.join(bag_root, bag_folder), bag_experiment), bagfile)
        with rosbag.Bag(bagfile, 'r') as bag:
            smallest_t = None
            # finish_t = None
            for init_t in start_topic:
                for topic, msg, t in bag.read_messages(topics=[init_t]):
                    start_t = t
                    if smallest_t is None or t < smallest_t:
                        smallest_t = t
                    # if finish_t is None or t > finish_t:
                    #     finish_t = t

        with rosbag.Bag(bagfile, 'r') as bag:
            finish_t = None
            for topic, msg, t in bag.read_messages(topics=finish_topic):
                if finish_t is None or t > finish_t:
                    finish_t = t

        elasped_time = finish_t - smallest_t
        minutes, seconds = seconds_to_minutes_seconds(elasped_time.secs)
        rescue_times.append(elasped_time.secs)
        print(f"Total time to rescue all targets: {minutes} minutes and {seconds} seconds")

    print(f"Average time to rescue all targets: {sum(rescue_times) / len(rescue_times)} seconds")
    print(f"Max time to rescue all targets: {max(rescue_times)} seconds")
    print(f"Min time to rescue all targets: {min(rescue_times)} seconds")
