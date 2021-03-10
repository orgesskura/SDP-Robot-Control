 
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import rospy
 
import utils

class my_navsat_transform:
    def __init__(self):
        self.origin = None
        self.seq = 0

    def set_origin(self, origin):
        self.origin = origin

    def transform_to_odometry(self, navsatfix_msg):
        y_coordinate = utils.get_distance_between(
            self.origin,
            utils.position(
                self.origin.longitude, # x
                navsatfix_msg.latitude # y
            )
        )
        x_coordinate = utils.get_distance_between(
            self.origin,
            utils.position(
                navsatfix_msg.longitude, # x
                self.origin.latitude # y
            )
        )
        self.seq += 1
        odom_msg = Odometry()
        odom_msg.header.seq = self.seq
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "odom"
        odom_msg.pose.pose.position.x = x_coordinate
        odom_msg.pose.pose.position.y = y_coordinate
        odom_msg.pose.pose.position.z = 0
        cov = [1e-9 for i in range(36)]
        cov[0] = navsatfix_msg.position_covariance[0]
        cov[7] = navsatfix_msg.position_covariance[4]
        cov[14] = navsatfix_msg.position_covariance[8]
        odom_msg.pose.covariance = cov
        return odom_msg