# Import necessary ROS libraries
import random
from enum import Enum

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Twist, Point
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel
from dist_project.msg import target_data, target_assignment_data

MIN_VEL = -0.2
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
        self.target_location = None
        self.all_targets_found = False
        self.rescue_drones = []
        self.drones_positions = {}
        self.targets_positions = {}
        self.estimated_drones_positions = {}
        self.robots_required_per_rescue = 2  # Number of robots required to rescue a target
        # TODO: define it
        self.min_x = -7.5
        self.max_x = 7.5
        self.min_y = -5
        self.max_y = 5
        self.targets_to_find = 1  # Total number of targets TODO: parametrize
        self.targets_rescued = []  # Total number of targets TODO: parametrize
        self.found_targets = []  # List of found target locations

        # ROS publishers and subscribers

        # Publish and receive global message related to the targets
        self.found_target_pub = rospy.Publisher('/found_targets', target_data, queue_size=10)
        self.target_sub = rospy.Subscriber('/found_targets', target_data, self.target_callback)

        # Publish velocity to move the drone
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Publish and receive messages related to the position of the neighbors
        self.neighbor_pub = rospy.Publisher('/neighbors', target_data, queue_size=10)
        self.neighbor_sub = rospy.Subscriber('/neighbors', target_data, self.neighbor_callback)

        self.ready_to_rescue_pub = rospy.Publisher("/ready_to_rescue", target_data, queue_size=10)
        self.ready_to_rescue_sub = rospy.Subscriber("/ready_to_rescue", target_data, self.ready_to_rescue_callback)

        # Publish and receive messages related to the target assignment to rescue targets
        self.target_assignment_pub = rospy.Publisher('/target_assignment', target_assignment_data, queue_size=10)
        self.target_assignment_sub = rospy.Subscriber('/target_assignment', target_assignment_data, self.assignment_callback)

        # Publish and receive messages related to the completion of the rescue
        self.all_targets_pub = rospy.Publisher('/all_targets_found', Bool, queue_size=10)
        self.all_targets_sub = rospy.Subscriber('/all_targets_found', Bool, self.all_targets_callback)

        # Subscribe to the point blob topic to get the target location
        self.point_sub = rospy.Subscriber("target/point_blob", Point, self.blob_callback)

        # Subscribe to the localization topic to get the estimated position of the drone
        self.localization_sub = rospy.Subscriber("localization_data_topic", Pose, self.position_callback)

        # Subscribe to the odometry topic to get the real position of the drone
        self.model_position_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.odom_callback)

        self.target_rescued_pub = rospy.Publisher('/target_rescued', target_data, queue_size=10)
        self.target_rescued_sub = rospy.Subscriber('/target_rescued', target_data, self.update_rescued_target_callback)

        # Subscribe to the init topic to start the movement when the drone is ready
        self.init_sub = rospy.Subscriber('init', Pose, self.init_callback)

    @property
    def current_position(self):
        """
        Get the real current position of the drone, used to check if it is inside the communication range
        """
        return self.drones_positions[self.drone_id]

    def init_callback(self, msg):
        """
        When the robot is initialized, i.e. it is flying at the desired height, then it is ready to start the mission.
        """
        self.current_state = RobotState.SEARCH
        self.init_sub.unregister()

    def update_rescued_target_callback(self, msg):
        """
        The drone is not rescuing, but it is receiving a message from another drone that has rescued a target.
        Add this target to the list of rescued targets. If the number of rescued targets is enough, then the drone
        stops and the mission is completed.
        """
        if msg.robot_id != self.drone_id:
            # TODO: communication_range
            is_already_present = False
            for rescued in self.targets_rescued:
                if abs(rescued.x - msg.x) < 0.5 and abs(rescued.y - msg.y) < 0.5:
                    is_already_present = True
            if not is_already_present:
                self.targets_rescued.append(msg)

            if len(self.targets_rescued) >= self.targets_to_find:
                self.all_targets_found = True
                self.current_state = RobotState.FINISH
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                self.vel_pub.publish(twist)

    def remove_rescued_target(self, msg):
        """
        The target has been rescued, so to avoid other drones to rescue the same target, the target is removed from the
        simulation. Since multiple drones can rescue the same target, this function will fail if the target has already
        been removed.
        """
        # Find the name of the target to remove. For now, it is the target with less distance from the message
        target_to_remove = None
        min_distance = None
        for target_id, target_position in self.targets_positions.items():
            distance = ((target_position.x - msg.x) ** 2 + (target_position.y - msg.y) ** 2) ** 0.5
            if min_distance is None or distance < min_distance:
                min_distance = distance
                target_to_remove = target_id

        if target_to_remove is not None:
            # Unspawn the target
            rospy.wait_for_service('/gazebo/delete_model')
            try:
                # Create a handle for the delete_model service
                delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

                # Call the service with the model name
                response = delete_model_service(target_to_remove)
                rospy.loginfo(f"Model {target_to_remove} rescued: {response.success}")
                self.targets_positions.pop(target_to_remove, None)
            except rospy.ServiceException as e:
                rospy.info(f"Unabled to remove target {target_to_remove}: {e}, maybe it was already removed")
            except Exception as e:
                rospy.info(f"Unabled to remove target {target_to_remove}: {e}, maybe it was already removed")

    def ready_to_rescue_callback(self, msg):
        """
        Condition: the drone is rescuing, so it is waiting other drones to rescue the target
        Message: another drone is ready to rescue the same target
        Effect: add the drone to the list of drones that are ready to rescue the target. If the number of drones is
        enough, then the target is rescued.
        """
        if msg.robot_id != self.drone_id:
            if self.current_state == RobotState.RESCUING:
                if self.target_location.x - 0.5 < msg.x < self.target_location.x + 0.5 and self.target_location.y - 0.5 < msg.y < self.target_location.y + 0.5:
                    self.rescue_drones.append(msg.robot_id)

        if len(self.rescue_drones) >= self.robots_required_per_rescue:
            self.targets_rescued.append(msg)
            # try to unspawn the target
            self.remove_rescued_target(msg)
            if len(self.targets_rescued) >= self.targets_to_find:
                self.all_targets_found = True
                self.current_state = RobotState.FINISH
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                self.vel_pub.publish(twist)
                self.target_rescued_pub.publish(msg)
            else:
                self.current_state = RobotState.SEARCH
                self.rescue_drones = []

    def position_callback(self, msg):
        """
        Read the estimated position of the drone from the localization system and publish it to the neighbors.
        """
        target_data_cell = target_data()
        target_data_cell.robot_id = self.drone_id
        target_data_cell.x = msg.position.x
        target_data_cell.y = msg.position.y
        target_data_cell.z = msg.position.z
        self.neighbor_pub.publish(target_data_cell)

    def assignment_callback(self, msg: target_assignment_data):
        """
        The robot receives a message from another robot that assigns a target to the receiving robot.
        This is a global topic, so it is received by all the drones (also the sender).
        Condition: the message is received only if the real position of the receiving drone is inside the communication
        range, the message is not sent by the receiving drone, and it is assigned to the receiving drone.

        If the drone is searching, then it is assigned to the target. If the drone is rescuing, then it is assigned to
        the target only if the sender has a lower ID than the previous sender.
        """
        if msg.assigned_robot_id == self.drone_id and msg.sender_robot_id != self.drone_id:
            # communication range
            sender_robot_id = msg.sender_robot_id
            sender_position = self.drones_positions[sender_robot_id]
            distance = ((self.current_position.x - sender_position.x) ** 2 + (self.current_position.y - sender_position.y) ** 2) ** 0.5
            if distance < COMMUNICATION_RANGE:
                # Assign
                if self.current_state == RobotState.SEARCH:
                    self.target_location = msg
                    self.current_state = RobotState.RESCUE
                elif self.current_state == RobotState.RESCUE and msg.sender_robot_id <= self.target_location.sender_robot_id:
                    self.target_location = msg

    def neighbor_callback(self, msg):
        """
        Callback from global topic where the estimated position of the neighbors is published.
        The message is received only if the real position of the receiving drone is inside the communication range, and it
        helps to estimate the position of the neighbors used to assign the target.
        """
        # This message arrive only if the real position of the receiving drone is inside the communication range
        robot_id = msg.robot_id
        if robot_id != self.drone_id:
            # if distance between robot_id and self.drone_id is less than the communication range, then the message is received
            sender_position = self.drones_positions[robot_id]
            distance = ((self.current_position.x - sender_position.x) ** 2 + (self.current_position.y - sender_position.y) ** 2) ** 0.5
            if distance < COMMUNICATION_RANGE:
                self.estimated_drones_positions[robot_id] = msg
            else:
                self.estimated_drones_positions.pop(robot_id, None)

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

    def target_callback(self, msg):
        """
        Global topic to receive the target location from the other drones. When a drone individuates a target, it sends
        the target location to all the other drones.
        Each drone has a list of found targets, and if the target is already present in the list, then it is substituted.
        """
        robot_id = msg.robot_id
        # if distance between robot_id and self.drone_id is less than the communication range, then the message is received
        sender_position = self.drones_positions[robot_id]
        if self.drone_id != robot_id:
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
        """
        Callback to signal that all targets have been rescued
        """
        self.all_targets_found = msg.data
        if msg.data:
            rospy.loginfo(f"Drone {self.drone_id} received signal: All targets rescued!")

    def generate_random_goal(self):
        """
        To move randomly in the environment, the drone generates a random point in the environment and moves to it.
        """
        # x = random.uniform(self.min_x, self.max_x)
        # y = random.uniform(self.min_y, self.max_y)
        if self.drone_id == 0:
            x = 1
            y = 0
        else:
            x = 2
            y = 2
        point = Point()
        point.x = x
        point.y = y
        return point

    def move_to_point(self, random_goal):
        """
        Move the drone to the random goal point. If the drone is close enough to the point, then it stops.
        """
        # Navigate to the target's location
        # TODO: here I am using the current, but it should be the estimated one
        dx = random_goal.x - self.current_position.x
        dy = random_goal.y - self.current_position.y

        distance = (dx ** 2 + dy ** 2) ** 0.5
        if distance < 0.1:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            self.vel_pub.publish(twist)
            return True

        unit_dx = dx / distance
        unit_dy = dy / distance

        vel_x = unit_dx * min(MAX_VEL, abs(dx))
        vel_y = unit_dy * min(MAX_VEL, abs(dy))
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        self.vel_pub.publish(twist)
        return False

    def move_to_target(self, target):
        """
        Move the drone to the target location. If the drone is close enough to the target, then it stops and changes
        the state to RESCUING.
        """
        # Navigate to the target's location
        # rospy.loginfo(f"Drone {self.drone_id} is moving to target at {target}")

        target_x = target.x
        current_position_x = self.current_position.x
        dx = target_x - current_position_x
        dy = target.y - self.current_position.y

        distance = (dx ** 2 + dy ** 2) ** 0.5
        if distance < 0.1:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            self.vel_pub.publish(twist)
            self.current_state = RobotState.RESCUING
            self.rescue_drones.append(self.drone_id)

        unit_dx = dx / distance
        unit_dy = dy / distance

        vel_x = unit_dx * min(MAX_VEL, abs(dx))
        vel_y = unit_dy * min(MAX_VEL, abs(dy))
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        self.vel_pub.publish(twist)

    def blob_callback(self, msg):
        """
        Callback to receive the target location from the point blob topic. When a drone detects a target, it sends the
        target location to all the other drones. It then starts to move to the target location to rescue the target.
        """
        target_location = target_assignment_data()
        # TODO: here I am using the current, but it should be the estimated one
        projected_x = msg.y
        projected_y = msg.x

        # height_difference = self.current_position.z - 1.2  # TODO: real value? -> in reality use the sensor
        # scaled_x = projected_x * (self.current_position.z / height_difference)
        # scaled_y = projected_y * (self.current_position.z / height_difference)
        #
        # target_location.x = self.current_position.x + scaled_x
        # target_location.y = self.current_position.y + scaled_y
        target_location.x = self.current_position.x - projected_x
        target_location.y = self.current_position.y - projected_y
        target_location.sender_robot_id = self.drone_id
        target_location.assigned_robot_id = self.drone_id

        # Simulate detecting a target (e.g., through a sensor or camera)
        # rospy.loginfo(f"Drone {self.drone_id} found a target at {target_location}")

        target_data_cell = target_data()
        target_data_cell.robot_id = self.drone_id
        target_data_cell.x = target_location.x
        target_data_cell.y = target_location.y

        new_target = True
        for found_target in self.found_targets:
            if abs(found_target.x - target_location.x) < 0.5 and abs(found_target.y - target_location.y) < 0.5:
                # rospy.loginfo(f"Drone {self.drone_id} already knows the existence of target at {target_location}")
                new_target = False

        if new_target:
            self.found_target_pub.publish(target_data_cell)
            self.found_targets.append(target_data_cell)
            # TODO: explore all or rescue one?
            # if len(self.found_targets) >= self.targets_to_find:
            #     self.all_targets_found = True
            #     self.start_rescue()  # Case 1: Start rescue after all targets are localized
            self.target_location = target_location
            self.current_state = RobotState.RESCUE
            self.start_rescue(target_data_cell)  # Case 2: Start rescue immediately for this target

    def start_rescue(self, target):
        """
        Start rescuing based on the current targets and state.
        If the drone is rescuing, then it is waiting for other required drones to rescue the target (if needed).
        Found the nearest drones to the target, and if the number of drones is enough, then assign the target to the
        drones publishing the target assignment message.
        """
        rospy.loginfo(f"Drone {self.drone_id} initiating rescue operation.")
        nearby_robots = self.find_nearby_robots(target)
        # TODO: what if the condition not holds?
        if len(nearby_robots) >= self.robots_required_per_rescue - 1:
            self.assign_rescue_task(target, nearby_robots[:self.robots_required_per_rescue - 1])

    def find_nearby_robots(self, target):
        """
        To find the nearest robots to the target, the drone knows the estimates of the positions of the other drones.
        Then it sends the message only if the real position of the receiving drone is inside the communication range.
        """
        nearby_robots = []
        for drone_id, position in self.estimated_drones_positions.items():
            if drone_id != self.drone_id:
                distance = ((position.x - target.x) ** 2 + (position.y - target.y) ** 2) ** 0.5
                if distance <= COMMUNICATION_RANGE:
                    nearby_robots.append(drone_id)
        return sorted(nearby_robots, key=lambda d_id: self.distance_to_target(d_id, target))

    def distance_to_target(self, drone_id, target):
        """Calculate distance between a drone and a target."""
        position = self.drones_positions[drone_id]
        return ((position.x - target.x) ** 2 + (position.y - target.y) ** 2) ** 0.5

    def assign_rescue_task(self, target, rescuers):
        """
        Send the target assignment message to the drones that are assigned to rescue the target.
        """
        rospy.loginfo(f"Assigning rescue for target at {target} to drones {rescuers}.")
        # Notify the rescuing robots

        for rescuer in rescuers:
            target_assignment_data_cell = target_assignment_data()
            target_assignment_data_cell.sender_robot_id = self.drone_id
            target_assignment_data_cell.assigned_robot_id = rescuer
            target_assignment_data_cell.x = target.x
            target_assignment_data_cell.y = target.y
            target_assignment_data_cell.z = target.z
            self.target_assignment_pub.publish(target_assignment_data_cell)

    def main_loop(self):
        """
        Continue to spin until the mission is completed.
        The states are:
        - INIT: the drone is initializing (no movement)
        - SEARCH: the drone is moving randomly in the environment
        - RESCUE: the drone is moving to the target location
        - RESCUING: the drone is near the target location and is waiting for other drones to rescue the target
        - FINISH: the drone has completed the mission
        """
        rate_val = 10
        rate = rospy.Rate(rate_val)

        while not rospy.is_shutdown():
            if self.all_targets_found:
                rospy.loginfo(f"Drone {self.drone_id} has completed its mission!")
                self.all_targets_pub.publish(True)
                break

            while self.current_state == RobotState.SEARCH:
                random_goal = self.generate_random_goal()
                reached = False
                # rospy.loginfo(f"Drone {self.drone_id} is searching randomly moving to point [{random_goal.x}, {random_goal.y}]")

                while not reached and self.current_state == RobotState.SEARCH:
                    reached = self.move_to_point(random_goal)

            while self.current_state == RobotState.RESCUE:
                self.move_to_target(self.target_location)
                # rospy.loginfo(f"Drone {self.drone_id} start to rescue the target at {self.target_location}")

            while self.current_state == RobotState.RESCUING:
                # rospy.loginfo(f"Drone {self.drone_id} is rescuing the target at {self.target_location}")
                target_data_cell = target_data()
                target_data_cell.robot_id = self.drone_id
                # TODO: here I am using the current, but it should be the estimated one
                # TODO: could be that it continue to send the target assignment message to the other drones?
                target_data_cell.x = self.current_position.x
                target_data_cell.y = self.current_position.y
                self.ready_to_rescue_pub.publish(target_data_cell)

            if len(self.targets_rescued) >= self.targets_to_find:
                rospy.loginfo(f"Drone {self.drone_id} has completed its mission!")
                self.all_targets_found = True
                self.current_state = RobotState.FINISH

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('motion_planner', anonymous=True)
    drone_id = rospy.get_param('~namespace')  # Get unique ID for this drone
    drone = MotionPlanner3d(drone_id)
    try:
        drone.main_loop()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program shutdown: Altitude stabilizer node interrupted")
    except rospy.ROSInternalException:
        rospy.loginfo("Altitude stabilizer node interrupted")
