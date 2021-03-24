 
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import rospy

import numpy
 
import utils

class my_navsat_transform:
    def __init__(self, hi=None):
        self.origin = None
        self.seq = 0
        self.hi = hi

    def set_origin(self, origin):
        self.origin = origin


    def transform_to_odometry(self, navsatfix_msg):
        position = utils.longlat_position(navsatfix_msg.longitude, navsatfix_msg.latitude)
        xy_position = self.longlat_to_xy(position)
        # add gaussian noise
        x_coordinate = xy_position.x
        y_coordinate = xy_position.y
        if self.hi is not None:
            noise = numpy.random.normal(loc=0, scale=self.hi.GPS_NOISE, size=2)
        else:
            noise = [0,0]
        x_coordinate += noise[0]
        y_coordinate += noise[1]
        self.seq += 1
        odom_msg = Odometry()
        odom_msg.header.seq = self.seq
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = ""
        odom_msg.pose.pose.position.x = x_coordinate
        odom_msg.pose.pose.position.y = y_coordinate
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation.x = 0
        odom_msg.pose.pose.orientation.y = 0
        odom_msg.pose.pose.orientation.z = 0
        odom_msg.pose.pose.orientation.w = 0
        odom_msg.twist.twist.linear.x = 0
        odom_msg.twist.twist.linear.y = 0
        odom_msg.twist.twist.linear.z = 0
        odom_msg.twist.twist.angular.x = 0
        odom_msg.twist.twist.angular.y = 0
        odom_msg.twist.twist.angular.z = 0
        cov = [0 for i in range(36)]
        cov[0] = navsatfix_msg.position_covariance[0]
        cov[7] = navsatfix_msg.position_covariance[4]
        cov[14] = navsatfix_msg.position_covariance[8]
        odom_msg.pose.covariance = cov
        return odom_msg

    def longlat_to_xy(self, longlat_pos):
        y_coordinate = utils.get_distance_between_longlat(
            self.origin,
            utils.longlat_position(
                self.origin.longitude, # x
                longlat_pos.latitude # y
            )
        )
        if longlat_pos.latitude < self.origin.latitude:
            y_coordinate *= -1
        x_coordinate = utils.get_distance_between_longlat(
            self.origin,
            utils.longlat_position(
                longlat_pos.longitude, # x
                self.origin.latitude # y
            )
        )
        if longlat_pos.longitude < self.origin.longitude:
            x_coordinate *= -1
        return utils.xy_position(x_coordinate, y_coordinate)