# Import necessary ROS libraries
import random
from enum import Enum

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Twist, Point
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from dist_project.msg import target_data

MIN_VEL = 0.05
MAX_VEL = 0.2
COMMUNICATION_RANGE = 20


class RobotState(Enum):
    INIT = "INIT"
    SEARCH = "SEARCH"
    RESCUE = "RESCUE"
    RESCUING = "RESCUING"
    FINISH = "FINISH"


class MotionPlanner3d:
    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.current_state = RobotState.INIT
        self.target_found = False
        self.target_location = None
        self.all_targets_found = False
        self.drones_positions = None
        # TODO: define it
        self.min_x = 0
        self.max_x = 100
        self.min_y = 0
        self.max_y = 100

        # ROS publishers and subscribers
        self.found_target_pub = rospy.Publisher('/found_targets', target_data, queue_size=10)
        self.vel_pub = rospy.Publisher("cmd_vel", Odometry, queue_size=1)

        self.point_sub = rospy.Subscriber("target/point_blob", Point, self.blob_callback)
        self.drone_position_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.odom_callback)
        self.target_sub = rospy.Subscriber('/found_targets', target_data, self.target_callback)
        self.all_targets_sub = rospy.Subscriber('/all_targets_found', Bool, self.all_targets_callback)

        # Parameters
        self.targets_to_find = 3  # Total number of targets TODO: parametrize
        self.found_targets = []  # List of found target locations

    @property
    def current_position(self):
        return self.drones_positions[self.drone_id]

    def odom_callback(self, msg):
        # Callback to get the current position of the drones
        for i, name in enumerate(msg.name):
            if "drone" in name:  # Assuming all drones have "drone" in their names
                drone_number = int(name.split("_")[1])
                position = msg.pose[i].position
                self.drones_positions[drone_number] = position

    def target_callback(self, msg):
        # This message arrive only if the real position of the receiving drone is inside the communication range
        robot_id = msg.robot_id
        # if distance between robot_id and self.drone_id is less than the communication range, then the message is received
        sender_position = self.drones_positions[robot_id]
        distance = ((self.current_position.x - sender_position.x) ** 2 + (self.current_position.y - sender_position.y) ** 2) ** 0.5
        if distance < COMMUNICATION_RANGE:
            already_found = False
            target_to_substitute = None
            for target in self.found_targets:
                if abs(target.x - msg.x) < 0.5 or abs(target.y - msg.y) < 0.5:
                    already_found = True
                    target_to_substitute = target

            if not already_found:
                self.found_targets.append(msg)
                rospy.loginfo(f"Drone {self.drone_id} received target location: {msg}")
            else:
                self.found_targets.remove(target_to_substitute)
                self.found_targets.append(msg)

    def all_targets_callback(self, msg):
        # Callback to signal that all targets have been found
        self.all_targets_found = msg.data
        if msg.data:
            rospy.loginfo(f"Drone {self.drone_id} received signal: All targets found!")

    def move_randomly(self, rate_val, current_step):
        # Simulate random movement
        rospy.loginfo(f"Drone {self.drone_id} is searching randomly...")
        # move randomly inside the map

        movement_time = 10
        if current_step == 0:
            twist = Twist()
            x = random.uniform(MIN_VEL, MAX_VEL)
            y = random.uniform(MIN_VEL, MAX_VEL)

            next_x = self.current_position.x + twist.linear.x * rate_val * movement_time
            next_y = self.current_position.y + twist.linear.y * rate_val * movement_time

            if next_x < self.min_x or next_x > self.max_x:
                x = -x

            if next_y < self.min_y or next_y > self.max_y:
                y = -y

            twist.linear.x = x
            twist.linear.y = y
            self.vel_pub.publish(twist)

    def move_to_target(self, target):
        # Navigate to the target's location
        rospy.loginfo(f"Drone {self.drone_id} is moving to target at {target}")

        dx = target.x - self.current_position.x
        dy = target.y - self.current_position.y

        distance = (dx ** 2 + dy ** 2) ** 0.5
        if distance < 0.1:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            self.vel_pub.publish(twist)
            self.current_state = RobotState.RESCUING

        unit_dx = dx / distance
        unit_dy = dy / distance

        vel_x = unit_dx * min(MAX_VEL, abs(dx))
        vel_y = unit_dy * min(MAX_VEL, abs(dy))
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        self.vel_pub.publish(twist)

    def blob_callback(self, target_location):
        # Simulate detecting a target (e.g., through a sensor or camera)
        rospy.loginfo(f"Drone {self.drone_id} found a target at {target_location}")
        target_data_cell = target_data()
        target_data_cell.robot_id = self.drone_id
        target_data_cell.x = target_location.x
        target_data_cell.y = target_location.y

        self.found_target_pub.publish(target_data_cell)
        self.found_targets.append(target_data_cell)
        # TODO: explore all or rescue one?
        self.target_found = True
        # self.current_state = RobotState.RESCUE

    def main_loop(self):
        rate_val = 10
        rate = rospy.Rate(rate_val)

        while not rospy.is_shutdown():
            if self.all_targets_found:
                rospy.loginfo(f"Drone {self.drone_id} has completed its mission!")
                self.all_targets_sub.publish(True)
                break

            current_step = 0
            while self.current_state == RobotState.SEARCH:
                self.move_randomly(rate_val, current_step)
                current_step = (current_step + 1) % 10

            while self.current_state == RobotState.RESCUE:
                self.move_to_target(self.target_location)
                rospy.loginfo(f"Drone {self.drone_id} rescued the target at {self.target_location}")

            self.target_found = False

            # Check if all targets have been found
            if len(self.found_targets) >= self.targets_to_find:
                rospy.loginfo(f"Drone {self.drone_id} signals: All targets found!")
                self.all_targets_found = True
                self.current_state = RobotState.FINISH

            rate.sleep()


if __name__ == '__main__':
    # TODO: cambi di stato:
    #  - DA init a search
    #  - DA search a rescue (capire quando farlo)
    #  - DA rescuing a finish (o un altro rescue/search)
    rospy.init_node('motion_planner', anonymous=True)
    drone_id = rospy.get_param('~drone_id', 1)  # Get unique ID for this drone
    drone = MotionPlanner3d(drone_id)
    try:
        drone.main_loop()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program shutdown: Altitude stabilizer node interrupted")
    except rospy.ROSInternalException:
        rospy.loginfo("Altitude stabilizer node interrupted")
