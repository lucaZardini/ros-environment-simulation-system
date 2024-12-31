import os
import rosbag
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":

    # Path of the rosbag file to extract data from
    bag_root = '/home/marco/shared/working_dir/catkin_ws/src/dist_project/bagfiles'
    bag_folder = "drones"
    bag_experiment = "experiment"

    bagfiles = os.listdir(os.path.join(os.path.join(bag_root, bag_folder), bag_experiment))
    num_drones = 5
    topics = [[] for i in range(num_drones)]
    for drone in range(num_drones):
        topics[drone].append(f"/drone{drone}/localization_data_topic")
        topics[drone].append(f"/drone{drone}/ground_truth/state")
        # topics[drone].append(f"/drone{drone}/gps_cartesian_point")

    # Empty lists to contain all data
    for bagfile_name in bagfiles:
        print(bagfile_name)
        bagfile = os.path.join(os.path.join(os.path.join(bag_root, bag_folder), bag_experiment), bagfile_name)
        # Data extraction
        # Packing of data iterating all messages stored
        for i in range(num_drones):
            loc_data = []
            gt_data = []
            geo_data = []
            drone_i_topics = topics[i]
            with rosbag.Bag(bagfile, 'r') as bag:
                for topic, msg, t in bag.read_messages(topics=[drone_i_topics[0]]):
                    x = msg.position.x
                    y = msg.position.y
                    loc_data.append([x, y])

            with rosbag.Bag(bagfile, 'r') as bag:
                for topic, msg, t in bag.read_messages(topics=[drone_i_topics[1]]):
                    x = msg.pose.pose.position.x
                    y = msg.pose.pose.position.y
                    gt_data.append([x, y])

            # with rosbag.Bag(bagfile, 'r') as bag:
            #     for topic, msg, t in bag.read_messages(topics=[drone_i_topics[2]]):
            #         x = msg.x
            #         y = msg.y
            #         geo_data.append([x, y])

            # Conversion of python list to numpy array
            loc_data = np.array(loc_data)
            gt_data = np.array(gt_data)
            # geo_data = np.array(geo_data)

            # List of tag position
            # tags = np.array(
            #     [[-10,-3],
            #     [-10,3],
            #     [-5,0],
            #     [4,3],
            #     [4,-3]]
            # )

            # Data plotting
            plt.figure()
            plt.plot(loc_data[:,0], loc_data[:,1], label='Estimated position')
            plt.plot(gt_data[:,0], gt_data[:,1], label='Ground truth position')
            # plt.plot(geo_data[:,0], geo_data[:,1], label='GPS position')
            # plt.plot(tags[:,0], tags[:,1], markersize=12, marker='*', linestyle='None', label='UWB tag')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title(f'Localization of drone {i}')
            plt.legend()
            # plt.savefig(f'{bag_experiment}_{bagfile_name}_drone{i}_localization_no_gps.png')
            plt.show()
