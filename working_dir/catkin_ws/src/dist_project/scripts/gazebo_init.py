import roslaunch.parent
import roslaunch.rlutil
import rospy
from gazebo_msgs.srv import SpawnModel
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import csv
import tf2_ros
import time
import roslaunch

# Import parameters defined in init.launch

# Tag parameters import
z_height = rospy.get_param("tag_height")
freq = rospy.get_param("tag_distance_rate")
tag_file_dir = rospy.get_param("tag_coordinate_file")
tag_urdf_file = rospy.get_param("tag_urdf_file")

# Target parameters import
target_file_dir = rospy.get_param("target_coordinate_file")
target_sdf_file = rospy.get_param("target_sdf_file")

# Room parameters import
room_sdf_file = rospy.get_param("room_sdf_file")

# Unicycle parameters import
unicycle_file_dir = rospy.get_param("unicycle_coordinate_file", None)
unicycle_description = rospy.get_param("unicycle_description", None)

# Drone parameters import
drone_file_dir = rospy.get_param("drone_coordinate_file", None)
drone_description = rospy.get_param("drone_description", None)

# General parameters import
init_time = rospy.get_param("/initialization_time")

def spawn_tag(x, y, id):
    # I use the service given by gazebo package spawn_urdf_model to dynamically spawn the tags at startup
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z_height
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0

        # Actual call to the service
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        # Model name to be visualized in gazebo
        spawn_model_client(
        model_name="tag_" + id,
            model_xml=open(tag_urdf_file, 'r').read(),
            initial_pose = pose,
            reference_frame = 'world'
        )
    except rospy.ServiceException as e:
        rospy.logerr("Tag initializer failed: %s", e)

def spawn_target(x, y, id):
    # Same logic as before but this time the model is a sdf file
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 1
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0

        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_client(
        model_name="target_" + id,
            model_xml=open(target_sdf_file, 'r').read(),
            initial_pose = pose,
            reference_frame = 'world'
        )
    except rospy.ServiceException as e:
        rospy.logerr("Target initializer failed: %s", e)

def spawn_robot(x, y, id):
    # Same logic as before with some twist
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 1
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0

        spawn_unicycle = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        # The arguments required are the name of the robot, the parsed robot description from the URDF file, the namespace definition, the initial pose and the parent reference frame
        spawn_unicycle("unicycle_" + id,
            unicycle_description,
            "/unicycle" + id,
            pose,
            "world"
        )

    except rospy.ServiceException as e:
        rospy.logerr("Robot " + id + " initializer failed: %s", e)


def spawn_drone(x, y, z, id):
    # Same logic as before with some twist
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0

        spawn_drone_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        # The arguments required are the name of the robot, the parsed robot description from the URDF file, the namespace definition, the initial pose and the parent reference frame
        spawn_drone_service("drone_" + id,
            drone_description,
            "/drone" + id,
            pose,
            "world"
        )

    except rospy.ServiceException as e:
        rospy.logerr("Drone robot " + id + " initializer failed: %s", e)

def spawn_room():
    # Same logic as before but simplier
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0

        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_client(
        model_name="room",
            model_xml=open(room_sdf_file, 'r').read(),
            initial_pose = pose,
            reference_frame = 'world'
        )
    except rospy.ServiceException as e:
        rospy.logerr("Room initializer failed: %s", e)

def publish_static_transform(x, y, id):
    # Creation of message to be published from each tag, it will be used by the localization script to get the static positions of each anchor
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    # Frame to measure the distance of each anchor with
    static_transformStamped.header.frame_id = "world"
    # Id used to identify each anchor
    static_transformStamped.child_frame_id = "uwb_anchor_" + id
    static_transformStamped.transform.translation.x = x 
    static_transformStamped.transform.translation.y = y
    static_transformStamped.transform.translation.z = z_height
    static_transformStamped.transform.rotation.x = 0
    static_transformStamped.transform.rotation.y = 0
    static_transformStamped.transform.rotation.z = 0
    static_transformStamped.transform.rotation.w = 1

    # Pubblication of the transformation
    broadcaster.sendTransform(static_transformStamped)

if __name__ == "__main__":

    task = rospy.get_param("task", None)
    if task is None:
        rospy.logerr("Task not specified")
        exit(1)

    if task not in ["target_estimation", "search_and_rescue"]:
        rospy.logerr("Task not recognized")
        exit(1)

    # List to add the tags' coordinates read from csv file
    tag_positions = []
    with open(tag_file_dir, 'r') as f:
        csvreader = csv.reader(f)
        for row in csvreader:
            # I cast to float each coordinate value
            row = [float(x) for x in row]
            tag_positions.append(row)

    # List to add the targets' coordinates read from csv file
    target_positions = []
    with open(target_file_dir, 'r') as f:
        csvreader = csv.reader(f)
        for row in csvreader:
            # I cast to float each coordinate value
            row = [float(x) for x in row]
            target_positions.append(row)

    # Initialization of the node
    rospy.init_node("gazebo_initializer_node")

    # Gazebo room spawn
    spawn_room()

    # Gazebo tag spawn
    for i, pos in enumerate(tag_positions):
        # I call the function for each line (tag)
        spawn_tag(pos[0], pos[1], str(i))

    # Gazebo target spawn
    for i, pos in enumerate(target_positions):
        # I call the function for each line (target)
        spawn_target(pos[0], pos[1], str(i))

    # List to add the robots' initial coordinates read from csv file
    unicycle_positions = []
    if task == "target_estimation":
        with open(unicycle_file_dir, 'r') as f:
            csvreader = csv.reader(f)
            for row in csvreader:
                # I cast to float each coordinate value
                row = [float(x) for x in row]
                unicycle_positions.append(row)
        for i, pos in enumerate(unicycle_positions):
            # I call the function for each line (unicycle)
            spawn_robot(pos[0], pos[1], str(i))

    drone_positions = []
    if task == "search_and_rescue":
        # List to add the drone' initial coordinates read from csv file
        with open(drone_file_dir, 'r') as f:
            csvreader = csv.reader(f)
            for row in csvreader:
                row = [float(x) for x in row]
                drone_positions.append(row)
        for i, pos in enumerate(drone_positions):
            # I call the function for each line (drone)
            print(pos)
            spawn_drone(pos[0], pos[1], pos[2], str(i))

    # Tf publishing 
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    time.sleep(init_time)

    # The next piece of code is used to recursively launch a launch file that initialize all the nodes with respect to each unicycle robot.
    # I run it here directly so to create the correct and dynamic number of nodes and namespace with respect to the number of unicycle robots in the csv file\
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    if task == "target_estimation":
        unicycle_number = len(unicycle_positions) - 1
        launch_argument = ['/home/marco/shared/working_dir/catkin_ws/src/dist_project/launch/start_unicycle_nodes.launch',
                           'num_unicycles:=' + str(unicycle_number)]
        roslaunch_args = launch_argument[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_argument)[0], roslaunch_args)]
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()

    if task == "search_and_rescue":
        drone_number = len(drone_positions) - 1

        launch_argument = ['/home/marco/shared/working_dir/catkin_ws/src/dist_project/launch/start_drone_nodes.launch',
                           'num_drones:=' + str(drone_number)]
        roslaunch_args = launch_argument[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_argument)[0], roslaunch_args)]
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()

    # I declare the frequency of the publishing of the tags' position
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        for i, pos in enumerate(tag_positions):
            # I continuosly publish the static positions of the tags
            publish_static_transform(pos[0], pos[1], str(i))
        rate.sleep()
