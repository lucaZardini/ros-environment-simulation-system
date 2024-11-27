import rospy
from enum import Enum
from geometry_msgs.msg import Twist, Pose


class RobotType(Enum):
    DRONE = "DRONE"
    UNICYCLE = "UNICYCLE"


class RobotInitializer:

    def __init__(self, robot_type: RobotType, drone_height: float = 2.5, unicycle_speed: float = 0.2,
                 drone_speed: float = 0.2):

        self.robot_type = robot_type
        self.drone_height = drone_height
        self.unicycle_speed = unicycle_speed
        self.drone_speed = drone_speed
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.init_pub = rospy.Publisher("init", Pose, queue_size=1)
        self.run_initialization()

        # self.sub = rospy.Subscriber("fix", NavSatFix, self.gps_callback)  # TODO: rename


    def run_initialization(self):
        if self.robot_type == RobotType.DRONE:
            self.initialize_drone()

        elif self.robot_type == RobotType.UNICYCLE:
            pass

    def initialize_drone(self):
        # The drone starts at time 0 at height 0
        # The drone goes up at speed drone_speed

        time_to_reach_height = self.drone_height / self.drone_speed
        time_elapsed = 0

        twist_message = Twist()
        twist_message.linear.z = self.drone_speed

        while time_elapsed < time_to_reach_height:
            self.pub.publish(twist_message)
            time_elapsed += 1
            rospy.sleep(1)

        twist_message.linear.z = 0
        self.pub.publish(twist_message)
        rospy.loginfo(f">>>> Drone has reached the desired height of {self.drone_height} =====")
        while True:
            self.init_pub.publish(Pose())


if __name__ == "__main__":

    rospy.init_node("robot_initializer_node", anonymous=True)
    robot_initializer = RobotInitializer(RobotType.DRONE)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program shutdown: robot initializer node interrupted")
    except rospy.ROSInternalException:
        rospy.loginfo("robot initializer node interrupted")
