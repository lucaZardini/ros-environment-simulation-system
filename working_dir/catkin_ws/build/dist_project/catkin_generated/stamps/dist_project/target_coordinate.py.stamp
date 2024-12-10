# Import necessary ROS libraries

import rospy
import math
from geometry_msgs.msg import Pose, Twist, Point
from gazebo_msgs.msg import ModelStates


class TargetCoordinate:

    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.drones_positions = {}
        self.targets_positions = {}

        # Subscribe to the point blob topic to get the target location
        self.point_sub = rospy.Subscriber("target/point_blob", Point, self.blob_callback)

        # Subscribe to the odometry topic to get the real position of the drone
        self.model_position_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.odom_callback)

    def odom_callback(self, msg):
        """
        Helper function to get the real position of the drones and the targets from the gazebo model states
        """
        # Callback to get the current position of the drones
        for i, name in enumerate(msg.name):
            if "drone" in name:  # Assuming all drones have "drone" in their names
                drone_number = int(name.split("_")[1])
                position = msg.pose[i].position
                self.drones_positions[drone_number] = position

            if "target" in name:
                position = msg.pose[i].position
                self.targets_positions[name] = position

    @property
    def current_position(self):
        """
        Get the real current position of the drone, used to check if it is inside the communication range
        """
        return self.drones_positions[self.drone_id]

    def compute_relative_distance(self, u, v, z_drone, width=640, height=480, h_fov=90):
        """
        Computes the relative distance between the image center and the target keypoint in meters.

        Parameters:
        - u (float): x-coordinate of the keypoint in pixels (horizontal).
        - v (float): y-coordinate of the keypoint in pixels (vertical).
        - z_drone (float): drone's height from the ground in meters.
        - width (int): camera resolution width in pixels (default: 640).
        - height (int): camera resolution height in pixels (default: 480).
        - h_fov (float): horizontal field of view in degrees (default: 100).

        Returns:
        - (float, float): relative distances (d_x, d_y) in meters.
        """
        # Convert degrees to radians
        h_fov_rad = math.radians(h_fov)

        # Calculate focal lengths
        f_x = width / (2 * math.tan(h_fov_rad / 2))
        v_fov_rad = 2 * math.atan((height / width) * math.tan(h_fov_rad / 2))
        f_y = height / (2 * math.tan(v_fov_rad / 2))

        # Convert pixel coordinates to normalized camera coordinates
        x_norm = (u - width / 2) / f_x
        y_norm = -(v - height / 2) / f_y  # Flip y-axis for top-down correction

        # Compute distances
        d_x = z_drone * x_norm
        d_y = z_drone * y_norm

        return d_x, d_y

    def blob_callback(self, msg):
        """
        Callback to receive the target location from the point blob topic. When a drone detects a target, it sends the
        target location to all the other drones. It then starts to move to the target location to rescue the target.
        """
        x_pixels = msg.x
        y_pixels = msg.y
        current_position = self.current_position
        target_height = 1.10

        z_distance = current_position.z - target_height

        d_x, d_y = self.compute_relative_distance(x_pixels, y_pixels, z_distance)

        target_x = current_position.x + d_y
        target_y = current_position.y - d_x

        rospy.loginfo(f"Drone {self.drone_id} received target location: ({target_x}, {target_y})")
        rospy.loginfo(f"Current position of drone {self.drone_id}: ({current_position}")

        # target_height = 0.6
        # for i in range(200):
        #     new_target_height = target_height + i * 0.01
        #     z_distance = current_position.z - new_target_height
        #     d_x, d_y = self.compute_relative_distance(x_pixels, y_pixels, z_distance)
        #     target_x = current_position.x + d_y
        #     target_y = current_position.y - d_x
        #     rospy.loginfo(f"({target_x}, {target_y})")
        #     if 1.48 < target_x < 1.52 and -0.02 < target_y < 0.02:
        #         rospy.loginfo(f"Target height: {new_target_height}")


if __name__ == '__main__':
    rospy.init_node('target_coordinate', anonymous=True)
    drone_id = rospy.get_param('~namespace')  # Get unique ID for this drone
    target_coordinate = TargetCoordinate(drone_id)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program shutdown: Altitude stabilizer node interrupted")
    except rospy.ROSInternalException:
        rospy.loginfo("Altitude stabilizer node interrupted")
