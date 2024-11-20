import rospy
from dist_project.msg import uwb_data
import tf
import time
from nav_msgs.msg import Odometry
import numpy as np


# Class creation
class UwbSimulation3D:

    def __init__(self):
        # Node initialization
        rospy.init_node('uwb_simulation_3d', anonymous=True)

        # Topic to publish the values of the distances of the robot from the anchors
        self.pub = rospy.Publisher('uwb_data_topic', uwb_data, queue_size=10)

        # List to store the tags' position
        self.anchor_pos = []
        # Maximum number of anchors to check
        self.max_anchors = 20
        # Function to be called once, it scans for every uwb anchor in the area and stores their position
        self.get_anchor_pos()
        self.tag_height = rospy.get_param("tag_height")

        # Initialization of current ground truth pose of the robot
        self.robot_pose = Odometry()

        # Subscription to local ground_truth topic coming from the robot
        sub = rospy.Subscriber('ground_truth/state', Odometry, self.subscribe_data)

    def get_anchor_pos(self):
        anchor_pos = []

        # This is relative to how I declared the id of each anchor in the gazebo_init script in the publish_static_transform function
        # I called them tag_0 with increasing numbers
        uwb_id = 'uwb_anchor_'

        # I start the listener from the package tf to get the transformation between the world frame and each tag. This was a easy way to implement id dynamically without using directly the coordinate csv file. I will retrieve the information of the anchors directly from the simulation.
        listener = tf.TransformListener()

        try:
            for i in range(self.max_anchors):

                # For each possible tag name
                try:
                    time.sleep(0.5)
                    # I store the translation of each tag w.r.t. world frame
                    (trans, rot) = listener.lookupTransform('/world', uwb_id + str(i), rospy.Time(0))
                    # I store the position found
                    anchor_pos.append(trans)
                
                # Error handling
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, KeyboardInterrupt):
                    break
            
                if anchor_pos == []:
                    # In case no uwb anchor was detected I run the function again. Usually this won't happen since I tried to make this function run after some initialization time interval to wait for the gazebo environment to correctly set up
                    rospy.logwarn("No UWB anchor detected. Trying again")
                    self.get_anchor_pos()
                else:
                    rospy.loginfo("UWB anchor list:\n" + str(anchor_pos))
                    # I update the current list of anchor positions
                    self.anchor_pos = anchor_pos
        
        except (KeyboardInterrupt):
            # I added this nested try-except to better catch the KeyboardInterrupt exception
            pass

    def subscribe_data(self, data):
        # I update the actual position of the robot
        self.robot_pose = data.pose.pose.position
        # Everytime I receive a new position I run the following function that computes the distances from each tag and publishes the result
        self.uwb_sim()

    def uwb_sim(self):
        time.sleep(0.1)
        distances = []
        ids = []

        # For each anchor I calculate its distance from the robot along with its id
        for i in range(len(self.anchor_pos)):
            dist = self.calculate_distance(self.anchor_pos[i])
            distances.append(dist)
            ids.append(i)

        # Once the distances have been calculated these information can be published
        self.publish_data(ids, distances)
    
    def calculate_distance(self, uwb_pose):
        # To compute the distance I simplify the process in the simulation by adding noise to the distance taken from the ground truth topic
        p1 = np.array([uwb_pose[0], uwb_pose[1], self.tag_height])
        p2 = np.array([self.robot_pose.x, self.robot_pose.y, self.robot_pose.z])

        # I calculate the difference vector squared between the tag and robot position

        noise_p1 = p1 + np.random.normal(0, 0.015, 3)
        noise_p2 = p2 + np.random.normal(0, 0.015, 3)
        uwb_dist = np.linalg.norm(np.array(noise_p2) - np.array(noise_p1))

        return uwb_dist
    
    def publish_data(self, ids, distances):
        # I initialize the message defined in the msg folder
        uwb_data_cell = uwb_data()
        uwb_data_cell.tag_id = ids
        uwb_data_cell.distance = distances
        # I publish the uwb data
        self.pub.publish(uwb_data_cell)


if __name__ == "__main__":
    
    # Initialize simulator class
    sim = UwbSimulation3D()

    # Start and run node
    rospy.spin()