#! /usr/bin/python3
"""main_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

import math
from controller import Robot
import rospy
import tf
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry

from my_navsat_transform import my_navsat_transform
from hardware_interface import hardware_interface
from robot_movement import robot_movement
import utils

class main_controller:

    def __init__(self):
        # initialize ros node
        rospy.init_node("main_controller")
        # more ros stuff
        self.msg_seq = 0
        self.imu0_pub = rospy.Publisher("/imu0_topic", Imu, queue_size=5)
        self.compass_pub = rospy.Publisher("/compass_topic", PoseWithCovarianceStamped, queue_size=5)
        self.gps_pub = rospy.Publisher("/gps_topic", NavSatFix, queue_size=5)
        self.gps_odom_pub = rospy.Publisher("/odom_gps", Odometry, queue_size=5)

        self.init_time = rospy.Time.now()
        # create the Robot instance.
        self.robot = Robot()
        # get the time step of the current world.
        self.timestep = int(self.robot.getBasicTimeStep())
        # initialize hardware interface
        self.hi = hardware_interface(self.robot)
        # enable devices
        self.hi.enable_devices()
        # set the propellers to turn indefinitely
        self.hi.set_right_propeller_position(float('+inf'))
        self.hi.set_left_propeller_position(float('+inf'))
        # initialize robot movement object
        self.rm = robot_movement(self.hi)
        # initialize navsat transform
        self.my_navsat_transform = my_navsat_transform()

        # setup stuff
        self.initial_global_yaw = -100
    

    def main_loop(self):
        cr = self.hi.get_compass_reading()
        gr = self.hi.get_gps_values()
        tmp = (0, 0)
        rubbish_pos = utils.position(tmp[1], tmp[0])
        print("Angle to turn: {}".format(self.rm.get_angle_to_target(rubbish_pos)))
        print("Distance to target: {}".format(self.rm.get_distance_to_target(rubbish_pos)))
        self.hi.set_left_propeller_velocity(-2)
        self.hi.set_right_propeller_velocity(-2)
        # if self.rm.is_in_proximity(rubbish_pos):
        #     self.rm.open_arms()
        # else:
        #     self.rm.close_arms()
        # self.rm.goto_long_lat(rubbish_pos)
        self.robot.step(self.timestep)
        self.send_sensor_readings_to_localization()


    def send_sensor_readings_to_localization(self):
        # Robot's axes:
        # x points right
        # y points front
        # z points up

        # z-axis angular velocity should be positive when rotating counter-clockwise
        z_axis_angular_vel = self.hi.get_gyro_reading()[2] # only interested in z axis angular velocity

        # get roll, pitch, yaw values
        # yaw should be zero when robot's axes are aligned with the global axes
        # i.e. y points north, x points east
        roll = self.hi.get_imu_reading()[1]
        pitch = self.hi.get_imu_reading()[2] + math.pi/2
        yaw = self.hi.get_imu_reading()[0] + math.pi/2
        print("Roll: {}".format(roll))
        print("Pitch: {}".format(pitch))
        print("Yaw: {}".format(yaw))
        if yaw > math.pi:
            yaw -= math.pi*2
        if self.initial_global_yaw == -100:
            self.initial_global_yaw = yaw

        # get acceleration values
        linear_acc = self.hi.get_accelerometer_reading() # accelerometer reading with gravitational acceleration bias
        g_quat = [0, 0, 9.81, 0]
        rotate_by = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        rotated_g = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_multiply(rotate_by, g_quat),
                tf.transformations.quaternion_conjugate(rotate_by)
            )[:3]
        linear_acc[0] -= rotated_g[0]
        linear_acc[1] -= rotated_g[1]
        linear_acc[2] -= rotated_g[2]
        print("accelerometer: {}".format(linear_acc))
        y_axis_linear_acc = linear_acc[1]

        # construct local-frame imu message
        self.msg_seq += 1
        imu_msg = Imu()
        # header
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.seq = self.msg_seq
        imu_msg.header.frame_id = "base_link"
        # quaternion
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        imu_msg.orientation_covariance = [0.0001 for i in range(9)]
        imu_msg.orientation_covariance[8] = 0.0001
        # angular velocity
        av = Vector3()
        av.x = 0
        av.y = 0
        av.z = z_axis_angular_vel
        imu_msg.angular_velocity = av
        avc = [0.0001 for i in range(9)]
        avc[8] = 0.0001
        imu_msg.angular_velocity_covariance = avc # TODO: FIX THIS
        # linear acceleration
        la = Vector3()
        la.x = 0
        la.y = y_axis_linear_acc
        la.z = 0
        imu_msg.linear_acceleration = la
        lac = [0.05 for i in range(9)]
        lac[4] = 0.05
        imu_msg.linear_acceleration_covariance = lac # TODO: AND THIS
        self.imu0_pub.publish(imu_msg)

        compass_reading = self.hi.get_compass_reading()
        # format compass_reading to be the counter-clockwise angle from east
        compass_reading *= -1
        if compass_reading > math.pi:
            compass_reading -= 2*math.pi
        if compass_reading < -math.pi:
            compass_reading += 2*math.pi
        print("Compass: {}".format(compass_reading))
        comp_msg = PoseWithCovarianceStamped()
        comp_msg.header.stamp = rospy.Time.now()
        comp_msg.header.seq = self.msg_seq
        comp_msg.header.frame_id = "base_link"
        quat = tf.transformations.quaternion_from_euler(0, 0, compass_reading)
        comp_msg.pose.pose.orientation.x = quat[0]
        comp_msg.pose.pose.orientation.y = quat[1]
        comp_msg.pose.pose.orientation.z = quat[2]
        comp_msg.pose.pose.orientation.w = quat[3]
        comp_cov = [0 for i in range(36)] # TODO: and this...
        for i in [0,7,14,21,28,35]:
            comp_cov[i] = 0.0001
        comp_msg.pose.covariance = comp_cov
        self.compass_pub.publish(comp_msg)

        gps_reading = self.hi.get_gps_values()
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.seq = self.msg_seq
        gps_msg.header.frame_id = "base_link"
        gps_msg.latitude = gps_reading[0]
        gps_msg.longitude = gps_reading[1]
        gps_msg.altitude = 0
        pos_cov = [0 for i in range(9)] # TODO: change!
        pos_cov[0] = 1e-6
        pos_cov[4] = 1e-6
        gps_msg.position_covariance = pos_cov
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN # TODO: change?
        #self.gps_pub.publish(gps_msg)
        if self.my_navsat_transform.origin is None:
            if gps_reading[0] is None or gps_reading[1] is None:
                return
            if math.isnan(gps_reading[0]) or math.isnan(gps_reading[1]):
                return
            self.my_navsat_transform.set_origin(utils.position(gps_msg.longitude, gps_msg.latitude))
        odom_gps_msg = self.my_navsat_transform.transform_to_odometry(gps_msg)
        self.gps_odom_pub.publish(odom_gps_msg)





if __name__ == "__main__":
    mc = main_controller()
    # while not rospy.is_shutdown():
    #     mc.main_loop()
    # while not rospy.is_shutdown():
    #     mc.rm.face(utils.position(1,-1))
    #     mc.send_sensor_readings_to_localization()
    #     mc.robot.step(mc.timestep)
    # move towards box
    tmp = [7.1375163565142205e-06, -2.5756252302227544e-05]
    rubbish_pos = utils.position(tmp[1], tmp[0])
    while not mc.rm.is_at(rubbish_pos) and not rospy.is_shutdown():
        cr = mc.hi.get_compass_reading()
        gr = mc.hi.get_gps_values()
        print("Angle to turn: {}".format(mc.rm.get_angle_to_target(rubbish_pos)))
        print("Distance to target: {}".format(mc.rm.get_distance_to_target(rubbish_pos)))
        if mc.rm.is_in_proximity(rubbish_pos):
            mc.rm.open_arms()
        else:
            mc.rm.close_arms()
        mc.rm.goto_long_lat(rubbish_pos)
        mc.send_sensor_readings_to_localization()
        mc.robot.step(mc.timestep)
    # move towards bottle
    tmp = [2.1141504335055434e-05, -1.5974212838350998e-05]
    rubbish_pos = utils.position(tmp[1], tmp[0])
    while not mc.rm.is_at(rubbish_pos) and not rospy.is_shutdown():
        cr = mc.hi.get_compass_reading()
        gr = mc.hi.get_gps_values()
        print("Angle to turn: {}".format(mc.rm.get_angle_to_target(rubbish_pos)))
        print("Distance to target: {}".format(mc.rm.get_distance_to_target(rubbish_pos)))
        if mc.rm.is_in_proximity(rubbish_pos):
            mc.rm.open_arms()
        else:
            mc.rm.close_arms()
        mc.rm.goto_long_lat(rubbish_pos)
        mc.send_sensor_readings_to_localization()
        mc.robot.step(mc.timestep)
    # move towards sphere
    tmp = [2.6291356923053307e-05, -5.860236671450979e-05]
    rubbish_pos = utils.position(tmp[1], tmp[0])
    while not mc.rm.is_at(rubbish_pos) and not rospy.is_shutdown():
        cr = mc.hi.get_compass_reading()
        gr = mc.hi.get_gps_values()
        print("Angle to turn: {}".format(mc.rm.get_angle_to_target(rubbish_pos)))
        print("Distance to target: {}".format(mc.rm.get_distance_to_target(rubbish_pos)))
        if mc.rm.is_in_proximity(rubbish_pos):
            mc.rm.open_arms()
        else:
            mc.rm.close_arms()
        mc.rm.goto_long_lat(rubbish_pos)
        mc.send_sensor_readings_to_localization()
        mc.robot.step(mc.timestep)
    # finished collection
    while not rospy.is_shutdown():
        mc.rm.close_arms()
        mc.hi.set_left_propeller_velocity(1)
        mc.hi.set_right_propeller_velocity(1)
        mc.robot.step(mc.timestep)
