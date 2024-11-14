import random

import rospy
from geometry_msgs.msg import Wrench, Point, Pose
from sensor_msgs.msg import NavSatFix
from enum import Enum

# Constants
TARGET_REACHED_Y_THRESHOLD = -0.5
PID_KP = 3.5
PID_KI = 0.1
PID_KD = 0.2
FORWARD_SPEED = 0.2
TURN_SPEED = 0.5
TARGET_ALTITUDE = 2.5
RATE = 10

# The idea was to create a finite state machine that switch between
# INIT -> the robot goes forward in one direction for a given amount of time until the estimation of its orientation is good enough since we don't have previous information on it
# SEARCH -> In this state the robot turns on itself until it spots the target
# APPROACH -> In this state the robot approaches the target and corrects the orientation with a simple PID controller
# TARGET_REACHED -> In this state the robots stops moving and starts publishing the target height seen from its own point of view in its own camera reference frame

class AltitudeStabilizer:

    def __init__(self):

        # Variable to hold the target height
        self.current_altitude = 0
        # Variables for PD control
        self.integral = 0
        self.previous_error = 0
        self.target_reached = False

        # Topic to publish control input
        self.pub = rospy.Publisher("vertical_lift_force", Wrench, queue_size=1)
        rospy.loginfo(f">>>> Publishing vertical force on /vertical_list_force topic =====")

        # Topic to get drone altitude
        self.sub = rospy.Subscriber("fix", NavSatFix, self.gps_callback)
        rospy.loginfo(f"<<<< Subscribed to local fix topic =====")

    def gps_callback(self, data):
        x, y, altitude = data.latitude, data.longitude, data.altitude

        error = TARGET_ALTITUDE - altitude
        # self.stabilize_altitude(error)

        if error > 0:
            self.publish_force(14.7 + 0.05)
        else:
            self.publish_force(14.7 - 0.05)
        # else:
        #     force = 14.7 + random.uniform(-0.1, 0.1)
        #     self.publish_force(force)

        # I call a function that holds the control logic of a simple PID controller

    def stabilize_altitude(self, error: float):
        # This functions manages the control input of the robot, it just takes as input the error
        
        # Update integral and derivative values
        dt = 1.0 / RATE
        self.integral += error * dt
        # integral_limit = 100
        # self.integral = min(max(self.integral, -integral_limit), integral_limit)
        # self.integral = self.integral * 0.9  # Gradually reduce integral action

        derivative = (error - self.previous_error) / dt

        # Simple logic
        control_input = PID_KP * error + PID_KI * self.integral + PID_KD * derivative

        # max_thrust = 13.7 + 5.0  # Maximum force you can apply
        # min_thrust = 13.7 + 0.0  # Minimum force you can apply
        # control_input = max(min(control_input, max_thrust), min_thrust)

        # Update of previous error with the current value
        self.previous_error = error

        # I publish to the cmd_vel topic the linear velocity and rotational input that the gazebo plugin will manage in order to command the robot to move in the simulation
        self.publish_force(control_input)
    
    def publish_force(self, force_altitude):
        # The cmd_vel topic accepts messages of type Twist
        command = Wrench()
        command.force.x = 0
        command.force.y = 0
        command.force.z = force_altitude
        command.torque.x = 0
        command.torque.y = 0
        command.torque.z = 0
        self.pub.publish(command)


if __name__ == "__main__":
    
    # Node initialization
    rospy.init_node("altitude_stabilizer_drone", anonymous=True)

    # Parameters import
    rate_val = RATE

    # Class initialilzation
    altitude_stabilizer = AltitudeStabilizer()

    # Ropsy rate of node imposition
    rate = rospy.Rate(rate_val)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program shutdown: Altitude stabilizer node interrupted")
    except rospy.ROSInternalException:
        rospy.loginfo("Altitude stabilizer node interrupted")



        