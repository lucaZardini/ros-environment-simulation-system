from enum import Enum

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from dist_project.msg import uwb_data
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import tf
import time


class KalmanEstimator:

    def __init__(self):
        # Publisher to notify the motion planner to start
        self.init_pub = rospy.Publisher('gps_cartesian_point', Point, queue_size=10)

        # Subscription to odom topic directly from gazebo simulation
        rospy.Subscriber('fix', NavSatFix, self.subscribe_gps_data)

    def subscribe_gps_data(self, fix_value: NavSatFix):
        lat, lon, h = fix_value.latitude, fix_value.longitude, fix_value.altitude
        lat_ref, lon_ref, h_ref = 49.860246, 8.687077, 0.0  # Reference point (origin of local frame)

        # Convert WGS84 to ECEF
        ecef = self.wgs84_to_ecef(lat, lon, h)
        ecef_ref = self.wgs84_to_ecef(lat_ref, lon_ref, h_ref)

        # Convert ECEF to local XYZ
        x, y, z = self.ecef_to_local(ecef, ecef_ref, lat_ref, lon_ref)
        point = Point()
        point.x = y
        point.y = -x
        point.z = z
        self.init_pub.publish(point)

    def wgs84_to_ecef(self, lat, lon, h):
        """
        Convert WGS84 geodetic coordinates to ECEF.
        Parameters:
            lat (float): Latitude in degrees
            lon (float): Longitude in degrees
            h (float): Height in meters
        Returns:
            tuple: (X, Y, Z) ECEF coordinates in meters
        """
        # WGS84 ellipsoid constants
        a = 6378137.0  # semi-major axis in meters
        e2 = 6.69437999014e-3  # first eccentricity squared

        # Convert latitude and longitude to radians
        lat = np.radians(lat)
        lon = np.radians(lon)

        # Calculate N, the radius of curvature in the prime vertical
        N = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)

        # Compute ECEF coordinates
        X = (N + h) * np.cos(lat) * np.cos(lon)
        Y = (N + h) * np.cos(lat) * np.sin(lon)
        Z = (N * (1 - e2) + h) * np.sin(lat)

        return X, Y, Z

    def ecef_to_local(self, ecef, ecef_ref, lat_ref, lon_ref):
        """
        Convert ECEF coordinates to local XYZ.
        Parameters:
            ecef (tuple): (X, Y, Z) ECEF coordinates
            ecef_ref (tuple): (X0, Y0, Z0) Reference point ECEF coordinates
            lat_ref (float): Latitude of reference point in degrees
            lon_ref (float): Longitude of reference point in degrees
        Returns:
            tuple: (X_local, Y_local, Z_local) local coordinates in meters
        """
        # Convert reference latitude and longitude to radians
        lat_ref = np.radians(lat_ref)
        lon_ref = np.radians(lon_ref)

        # Define the rotation matrix
        R = np.array([
            [-np.sin(lon_ref), np.cos(lon_ref), 0],
            [-np.cos(lon_ref) * np.sin(lat_ref), -np.sin(lon_ref) * np.sin(lat_ref), np.cos(lat_ref)],
            [np.cos(lon_ref) * np.cos(lat_ref), np.sin(lon_ref) * np.cos(lat_ref), np.sin(lat_ref)]
        ])

        # Calculate the delta vector in ECEF
        delta = np.array(ecef) - np.array(ecef_ref)

        # Transform to local XYZ using the rotation matrix
        local_xyz = R @ delta

        return tuple(local_xyz)


if __name__ == "__main__":

    # Node initialization
    rospy.init_node('gps_conversion', anonymous=True)

    rate_val = rospy.get_param("/localization_rate")

    # Class initialization
    estimator = KalmanEstimator()

    # Rospy rate of node impostion
    rate = rospy.Rate(rate_val)

    try:
        while not rospy.is_shutdown():
            # We don't need to cast any particular function since the estimator already works on the callbacks of the topics
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program shutdown: Kalman filter estimator node interrupted")
    except rospy.ROSInternalException:
        rospy.loginfo("Kalman localization node interrupted")
