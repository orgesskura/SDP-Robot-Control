#! /usr/bin/python3
"""main_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

import math
import threading

from controller import Robot
import rospy
import tf
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Imu, NavSatFix, Image
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from my_navsat_transform import my_navsat_transform
from hardware_interface import hardware_interface
from robot_movement import robot_movement
import utils
from WebAppFirebase import updateDatabase

class main_controller:

    def __init__(self):
        # initialize ros node
        rospy.init_node("main_controller")
        # publishers
        self.msg_seq = 0
        self.imu0_pub = rospy.Publisher("/imu0_topic", Imu, queue_size=5)
        self.compass_pub = rospy.Publisher("/compass_topic", PoseWithCovarianceStamped, queue_size=5)
        self.gps_odom_pub = rospy.Publisher("/odom_gps", Odometry, queue_size=5)
        self.front_image_pub = rospy.Publisher("/front_camera_view", Image, queue_size=1)
        self.water_image_pub = rospy.Publisher("/water_camera_view", Image, queue_size=1)
        # subscribers
        self.robot_state_sub = rospy.Subscriber("/map_prediction", Odometry, callback=self.update_robot_state)
        self.is_object_detected_sub = rospy.Subscriber("/is_object_detected", Bool, callback=self.update_is_object_detected)
        self.object_dist_from_center_sub = rospy.Subscriber("/object_dist_from_center", Float64, callback=self.update_object_dist_from_center)

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

        # setup camera image reading stuff
        self.bridge = CvBridge()
        self.image_read_rate = rospy.Rate(20) # Hz
        self.image_read_thread = threading.Thread(target=self.publish_camera_images, daemon=True)
        self.image_read_thread.start()
        # setup object detection variables
        self.is_object_detected = False
        self.object_dist_from_center = None
        
        # battery
        self.battery_level = 100

        # setup server data uploader
        self.data_send_rate = rospy.Rate(1/5) # Hz
        self.data_send_thread = threading.Thread(target=self.send_data_to_server, daemon=True)
        self.data_send_thread.start()

        # path planning

    
    def send_data_to_server(self):
        while True:
            self.data_send_rate.sleep()
            gps_reading = self.hi.get_gps_values()
            long_lat_pos = utils.longlat_position(gps_reading[1], gps_reading[0])
            battery = self.battery_level
            if long_lat_pos is None or battery is None:
                continue
            updateDatabase(long_lat_pos, battery)
    
    def update_is_object_detected(self, is_object_detected):
        self.is_object_detected = is_object_detected.data
        if not self.is_object_detected:
            self.object_dist_from_center = None
        
    def update_object_dist_from_center(self, dist):
        self.object_dist_from_center = dist.data
    
    def update_robot_state(self, state):
        # state is of type Odometry
        self.rm.robot_state = state    

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
        # print("Roll: {}".format(roll))
        # print("Pitch: {}".format(pitch))
        # print("Yaw: {}".format(yaw))
        if yaw > math.pi:
            yaw -= math.pi*2

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
        imu_msg.orientation_covariance = [0 for i in range(9)]
        imu_msg.orientation_covariance[8] = math.radians(1)**2
        # angular velocity
        av = Vector3()
        av.x = 0
        av.y = 0
        av.z = z_axis_angular_vel
        imu_msg.angular_velocity = av
        avc = [0 for i in range(9)]
        avc[8] = 0.02**2
        imu_msg.angular_velocity_covariance = avc # TODO: FIX THIS
        # linear acceleration
        la = Vector3()
        la.x = 0
        la.y = y_axis_linear_acc
        la.z = 0
        imu_msg.linear_acceleration = la
        lac = [0 for i in range(9)]
        lac[4] = 0.1**2
        imu_msg.linear_acceleration_covariance = lac # TODO: AND THIS
        self.imu0_pub.publish(imu_msg)

        compass_reading = self.hi.get_compass_reading()
        # compass_reading is the counter-clockwise angle from east
        # if compass_reading > math.pi:
        #     compass_reading -= 2*math.pi
        # if compass_reading < -math.pi:
        #     compass_reading += 2*math.pi
        # print("Compass: {}".format(compass_reading))
        comp_msg = PoseWithCovarianceStamped()
        comp_msg.header.stamp = rospy.Time.now()
        comp_msg.header.seq = self.msg_seq
        comp_msg.header.frame_id = "map"
        quat = tf.transformations.quaternion_from_euler(0, 0, compass_reading)
        comp_msg.pose.pose.orientation.x = quat[0]
        comp_msg.pose.pose.orientation.y = quat[1]
        comp_msg.pose.pose.orientation.z = quat[2]
        comp_msg.pose.pose.orientation.w = quat[3]
        comp_cov = [0 for i in range(36)] # TODO: and this...
        for i in [0,7,14,21,28,35]:
            comp_cov[i] = math.radians(1)**2
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
        pos_cov[0] = 0.00001
        pos_cov[4] = 0.00001
        gps_msg.position_covariance = pos_cov
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN # TODO: change?
        if self.my_navsat_transform.origin is None:
            if gps_reading[0] is None or gps_reading[1] is None:
                return
            if math.isnan(gps_reading[0]) or math.isnan(gps_reading[1]):
                return
            self.my_navsat_transform.set_origin(utils.longlat_position(gps_msg.longitude, gps_msg.latitude))
        odom_gps_msg = self.my_navsat_transform.transform_to_odometry(gps_msg)
        self.gps_odom_pub.publish(odom_gps_msg)
    
    def publish_camera_images(self):
        while True:
            img_front = self.hi.get_front_camera_image()
            img_water = self.hi.get_water_camera_image()
            if img_front is not None and img_water is not None:
                try:
                    img_front_msg = self.bridge.cv2_to_imgmsg(img_front)
                    img_water_msg = self.bridge.cv2_to_imgmsg(img_water)
                except Exception as e:
                    print(e)
                    print("Exiting image publisher thread...")
                    return
                self.front_image_pub.publish(img_front_msg)
                self.water_image_pub.publish(img_water_msg)
            self.image_read_rate.sleep()

    
    def main_loop(self):
        # rubbish_pos = utils.xy_position(3,3)
        # if self.rm.is_in_arms_open_distance(rubbish_pos):
        #     self.rm.open_arms()
        # else:
        #     self.rm.close_arms()
        # self.rm.goto_xy(rubbish_pos)
        if self.is_object_detected:
            self.rm.face_object_vision(self.object_dist_from_center)
            print("Object distance from image center: {}".format(self.object_dist_from_center))
        self.robot.step(self.timestep)
        self.send_sensor_readings_to_localization()





if __name__ == "__main__":
    mc = main_controller()
    while not rospy.is_shutdown():
        mc.main_loop()

