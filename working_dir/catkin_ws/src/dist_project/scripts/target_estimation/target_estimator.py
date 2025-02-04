import rospy
from dist_project.msg import robot_data
from geometry_msgs.msg import Pose, Point
import numpy as np
# from scipy.optimize import minimize

# Vertical tilt of the camera taken out empirically
VERTICAL_TILT = 67 * np.pi / 180

class TargetEstimator:

    def __init__(self, namespace):

        # Initialization of variables
        self.target_height = 0
        self.x = 0
        self.sigma_x = 0
        self.y = 0
        self.sigma_y = 0
        self.theta = 0
        self.sigma_theta = 0
        self.tag = robot_data()
        self.start_publishing = 0
        self.namespace = namespace

        # Target lectures list from other robots
        self.robot_tags = []

        # Subscription to global topic to publish data to share
        self.pub_tag = rospy.Publisher("/processing_data", robot_data, queue_size=1)

        # Subscription to local topic to publish the final target estimate
        self.pub_target = rospy.Publisher("target_estimate", Pose, queue_size=1)

        # Subscription to local topic for target height
        self.sub_height = rospy.Subscriber("target_height", Point, self.target_callback)

        # Subscription to local topic for x,y values (robot position)
        self.sub_pos = rospy.Subscriber("localization_data_topic", Pose, self.localization_callback)

    
    def target_callback(self, data):
        # Everytime the robots receives a new data from the local camera it uploads the current stored value for the target height
        self.target_height = -data.z

    def localization_callback(self, acutual_pos):
        # Everytime the robot receives new data from the localization module uploads the current stored values
        self.x = acutual_pos.position.x
        self.y = acutual_pos.position.y
        self.theta = acutual_pos.orientation.z
        self.sigma_x = acutual_pos.orientation.x
        self.sigma_y = acutual_pos.orientation.y
        self.sigma_theta = acutual_pos.orientation.w
        
        # If it has already stored a value for the target height it means the robot is on position and then can start publishing the data to the others robot
        if self.target_height != 0:
            # Compilation of the self-tag with all the informations
            self.tag.robot_id = self.namespace
            self.tag.target_height = self.target_height
            self.tag.x = self.x
            self.tag.y = self.y
            self.tag.theta = self.theta
            self.tag.sigma_x = self.sigma_x
            self.tag.sigma_y = self.sigma_y
            self.tag.sigma_theta = self.sigma_theta

            # Local variable to tell the module to start publishing
            if not self.start_publishing:
                # Subscription to global topic for communication
                self.sub_data = rospy.Subscriber("/processing_data", robot_data, self.gather_callback)
                self.start_publishing = True

            # Addition of the self-tag to the list of tags
            present = False
            for i, tag in enumerate(self.robot_tags):
                if tag.robot_id == self.namespace:
                    present = True
                    self.robot_tags[i] = self.tag

            if not present:
                self.robot_tags.append(self.tag)
            
            self.pub_tag.publish(self.tag)


    # def intersect(self, slopes, sigma_slopes, y_intercepts, sigma_intercepts):
    #     # This function finds the intersection points of each pair of lines representing each robot position and orientation.
    #     # It estimates then the target position with a weighted average of all the intersection points
    #     # The assigned weight is inversely proportional to each point's uncertainty

    #     n = len(slopes)
    #     intersection_points = []
    #     weights = []

    #     for i in range(n):
    #         for j in range(i+1, n):
    #             if (slopes[i] - slopes[j] > 1e-3):

    #                 # Simple geometrical formuals to extract the intersection and law of propagation of errors to get the uncertainties
    #                 x_intersect = (y_intercepts[j] - y_intercepts[i]) / (slopes[i] - slopes[j])
    #                 sigma_x_intersect = np.sqrt((sigma_intercepts[i] / (slopes[i] - slopes[j]))**2 + (sigma_intercepts[j] / (slopes[i] - slopes[j]))**2 + ((y_intercepts[i] - y_intercepts[j]) * (sigma_slopes[i] + sigma_slopes[j]) / (slopes[i] - slopes[j])**2)**2)

    #                 y_intersect = slopes[i] * x_intersect + y_intercepts[i]
    #                 sigma_y_intersect = np.sqrt((x_intersect * sigma_slopes[i])**2 + (slopes[i] * sigma_x_intersect)**2 + sigma_intercepts[i]**2)

    #                 intersection_points.append((x_intersect, y_intersect))

    #                 # Weights computing as the inverse of the overall uncertaintyy of the intersection point just found
    #                 weights.append(1 / np.sqrt(sigma_x_intersect**2 + sigma_y_intersect**2))
        
    #     # Weighted average point computation
    #     if intersection_points:
    #         total_weight = sum(weights)
    #         weighted_sum_x = sum(w * point[0] for w, point in zip(weights, intersection_points))
    #         weighted_sum_y = sum(w * point[1] for w, point in zip(weights, intersection_points))

    #         avg_x = weighted_sum_x / total_weight
    #         avg_y = weighted_sum_y / total_weight

    #         avg_x_err = np.sqrt(1/total_weight)
    #         avg_y_err = np.sqrt(1/total_weight)

    #         return avg_x, avg_x_err, avg_y, avg_y_err
        
    #     else:
    #         return None, None, None, None

    def least_square_estimation(self, slopes, sigma_slopes, y_intercepts, sigma_y_intercepts):
        # Number of unicycles
        n = len(slopes)

        # Matrix initialization
        A = np.zeros((n,2))
        b = np.zeros(n)
        C_inv = np.zeros((n, n))

        # Loop to fill all the matrices
        for i in range(n):
            m = slopes[i]
            b_i = y_intercepts[i]
            sigma_m = sigma_slopes[i]
            sigma_b = sigma_y_intercepts[i]

            # Combined uncertainty computation
            combined_sigma = np.sqrt(sigma_m**2 + sigma_b**2)

            A[i, 0] = m
            A[i, 1] = -1
            b[i] = -b_i

            C_inv[i, i] = 1.0 / combined_sigma**2
        
        # Weighting A and b matrices using the inverse of the covariance matrix
        A_weighted = C_inv @ A
        b_weighted = C_inv @ b
        
        # Solving the LSQ problem
        # Equivalent of p = (Aw^T . Aw)^-1 . Aw^T . bw with Aw and bw being the previosuly weighted matrices
        p, residuals, rank, s = np.linalg.lstsq(A_weighted, b_weighted, rcond=None)

        # Covariance matrix computation
        cov_matrix = np.linalg.inv(A.T @ C_inv @ A)

        # Uncertaintes extraction
        sigma_x = np.sqrt(cov_matrix[0, 0])
        sigma_y = np.sqrt(cov_matrix[1, 1])

        # Returns slope, y-intercepts with corresponding uncertaintes
        return p[0], sigma_x, p[1], sigma_y

    
    def gather_callback(self, data):
        # Called upon reception of other tags from other robots
        
        # Check to assure the tag is not already present
        present = False
        for i, tag in enumerate(self.robot_tags):
            if tag.robot_id == data.robot_id:
                # If the id matches the tag gets updated
                present = True
                self.robot_tags[i] = data
        
        # Otherwise gets added to the list
        if not present:
            self.robot_tags.append(data)

        # When the list contains at least 2 different tags from 2 robots it estimates the target position
        if len(self.robot_tags) >= 2:
            
            # lines will be a list of tuples containing the slope and y intercept of the direction line of each robot
            slopes = []
            sigma_slopes = []
            intercepts = []
            sigma_intercepts = []
            # quotes will contain the estimate z coordinate of every robot
            quotes = []

            for tag in self.robot_tags:
                # Slope computation
                slope = np.tan(tag.theta)
                # I apply the law of propagation of uncertainties
                sigma_slope = tag.sigma_theta/(np.cos(tag.theta))**2 
                # y_intercept computation
                y_intercept = tag.y - slope * tag.x
                sigma_intercept = np.sqrt(tag.sigma_y**2 + slope**2 * tag.sigma_x**2 + tag.x**2 * sigma_slope**2)
 
                slopes.append(slope)
                sigma_slopes.append(sigma_slope)
                intercepts.append(y_intercept)
                sigma_intercepts.append(sigma_intercept)

            # Target position with uncertainties extraction
            # target_x, sigma_target_x, target_y, sigma_target_y = self.intersect(slopes, sigma_slopes, intercepts, sigma_intercepts)
            target_x, sigma_target_x, target_y, sigma_target_y = self.least_square_estimation(slopes, sigma_slopes, intercepts, sigma_intercepts)

            # Once the plane position of the target has been computed its height estimation will be carried out
            if target_x != None:
                # z-coordinate estimation considering the vertical fov is 67 degrees. At a value of target_height of 1 corresponds 67 degrees. For 0 is 0 degrees.

                squared_sum_sigma = 0                

                for i, tag in enumerate(self.robot_tags):
                    # Plane distance from robot and target
                    distance = np.sqrt((tag.x - target_x)**2 + (tag.y - target_y)**2)
                    # Vertical angle computation proportional to target_height value
                    vertical_angle = tag.target_height * VERTICAL_TILT
                    # Direct consequence of the definition of tangent
                    quotes.append(np.tan(vertical_angle) * distance)

                    # Uncertainty computation via law of propagation of error
                    sigma_distance = np.sqrt((2 * (tag.x - target_x) * self.tag.sigma_x)**2 + (2 * (tag.y - target_y) * self.tag.sigma_y)**2)
                    sigma_vertical_angle = VERTICAL_TILT * 0.001
                    sigma_quotes = np.sqrt((np.tan(vertical_angle) * sigma_distance)**2 + (distance / np.cos(vertical_angle)**2 * np.pi / 3 * sigma_vertical_angle)**2)
                    squared_sum_sigma = squared_sum_sigma + sigma_quotes**2
                
                # It takes the mean of all the computed values
                target_z = np.mean(np.array(quotes))
                sigma_target_z = np.sqrt(squared_sum_sigma) / len(quotes)

                # Composition and publishing of target estimate
                message = Pose()
                message.position.x = target_x
                message.position.y = target_y
                message.position.z = target_z
                message.orientation.x = sigma_target_x
                message.orientation.y = sigma_target_y
                message.orientation.z = sigma_target_z

                try:
                    self.pub_target.publish(message)
                # Sometimes it raises an exception when publishing the first message, i catch the exception since it has no effects on the program execution
                except rospy.exceptions.ROSException as e:
                    rospy.loginfo(str(e))
        

if __name__ == '__main__':

    # Node initialization
    rospy.init_node('target_estimator', anonymous=True)

    # Parameters import
    rate_val = rospy.get_param("/target_estimator_rate")
    id = rospy.get_param('~namespace')
    # TODO: change it
    namespace = "unicycle" + str(id)

    # Class initialization
    target_estimator = TargetEstimator(namespace)
    
    # Rospy rate of node imposition
    rate = rospy.Rate(rate_val)

    try:
        while not rospy.is_shutdown():
            # Start self-tag publishing
            if target_estimator.start_publishing:
                target_estimator.pub_tag.publish(target_estimator.tag)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program shutdown: Target estimator node interrupted")
    except rospy.ROSInternalException:
        rospy.loginfo("Target estimator node interrupted")
