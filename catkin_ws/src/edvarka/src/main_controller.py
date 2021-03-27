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
from WebAppFirebase import updateDatabase, getTargetCoordinatesFromDatabase
#navigation
import navigation.CentroidUpdate as centroidUpdate
import navigation.FirstJourney as firstJourney

class main_controller:

    def __init__(self, lochEdge, numberOfStops, lochName):
        # initialize ros node
        rospy.init_node("main_controller")
        # publishers
        self.msg_seq = 0
        self.imu0_pub = rospy.Publisher("/imu0_topic", Imu, queue_size=5)
        self.compass_pub = rospy.Publisher("/compass_topic", PoseWithCovarianceStamped, queue_size=5)
        self.gps1_odom_pub = rospy.Publisher("/odom_gps1", Odometry, queue_size=5)
        self.gps2_odom_pub = rospy.Publisher("/odom_gps2", Odometry, queue_size=5)
        self.front_image_pub = rospy.Publisher("/front_camera_view", Image, queue_size=1)
        self.water_image_pub = rospy.Publisher("/water_camera_view", Image, queue_size=1)
        # subscribers
        self.robot_state_sub = rospy.Subscriber("/map_prediction", Odometry, callback=self.update_robot_state)
        self.is_object_detected_sub = rospy.Subscriber("/is_object_detected", Bool, callback=self.update_is_object_detected)
        self.object_dist_from_center_sub = rospy.Subscriber("/object_dist_from_center", Float64, callback=self.update_object_dist_from_center)
        self.object_size_sub = rospy.Subscriber("/object_size", Float64, callback=self.update_object_size)
        self.object_y_pos_sub = rospy.Subscriber("/object_y_pos", Float64, callback=self.update_object_y_pos)

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
        self.my_navsat_transform = my_navsat_transform(self.hi)

        # setup object detection variables
        self.is_object_detected = False
        self.object_dist_from_center = None
        self.object_size = None
        self.object_y_pos = None
        self.object_collected_timer = 0
        self.OBJECT_COLLECTED_TIMER_INIT = 1000/(self.timestep) * 4 # seconds
        self.CM3_PER_PIXEL = 0.096154 # estimate
        self.OBJECT_Y_POS_NEAR_THRESH = 180 # image pixels

        # capacity stuff
        self.BASKET_CAPACITY = 9000 # cm^3
        self.current_trash_volume = 0
        
        # battery
        self.battery_level = 100

        # path planning
        self.lochEdge = lochEdge
        self.lochName = lochName
        self.numberOfStops = numberOfStops
        position = None
        gps_reading = [None, None]
        while gps_reading[0] is None or gps_reading[1] is None or math.isnan(gps_reading[0]) or math.isnan(gps_reading[1]):
            gps_reading = self.hi.get_gps_values()
            self.robot.step(self.timestep)
        position = utils.longlat_position(gps_reading[1], gps_reading[0])
        
        self.startingLong = position.longitude
        self.startingLat = position.latitude
        self.itter = 0 # This is the number of itterations of the lake that EdVarka has done 
        self.autonomous_mode = True # default should be True?
        self.firstPath = firstJourney.firstJourney(self.lochEdge, self.numberOfStops, self.startingLong, self.startingLat, self.lochName) # This will give us a set of points to visit
        self.startingPos = position # Starting position
        self.trashFound = []

        self.autonomous_mode = True # default should be True?
        # self.path = [ utils.xy_position(6,0),
        #               utils.xy_position(6,6),
        #               utils.xy_position(0,6),
        #               utils.xy_position(-6,6),
        #               utils.xy_position(-6,0),
        #               utils.xy_position(-6,-6),
        #               utils.xy_position(0,-6),
        #               utils.xy_position(6,-6) ]
        self.path = [self.startingPos] # Takes list of centroid coordinates and converts to utils xy_position
        for cent in self.firstPath.centroidList:
            self.path.append(utils.longlat_position(cent[0], cent[1]))
        self.path.append(self.startingPos)
        self.my_navsat_transform.set_origin(self.startingPos)
        self.path_pos = 0
        self.is_collecting_trash = False

        # setup server communication
        self.server_comm_rate = rospy.Rate(1/5) # Hz
        self.server_comm_thread = threading.Thread(target=self.communicate_with_server, daemon=True)
        self.server_comm_thread.start()

        # setup camera image reading stuff
        self.bridge = CvBridge()
        self.image_read_rate = rospy.Rate(20) # Hz
        self.image_read_thread = threading.Thread(target=self.publish_camera_images, daemon=True)
        self.image_read_thread.start()

        # ensure trash collection (etc)
        self.is_etc = False
        self.etc_timer = 0
        self.ETC_TIMER_INIT = 35

        # printing
        self.prev_print = ""


    
    def communicate_with_server(self):
        while True:
            self.server_comm_rate.sleep()
            gps_reading = self.hi.get_gps_values()
            long_lat_pos = utils.longlat_position(gps_reading[1], gps_reading[0])
            battery = self.battery_level
            if long_lat_pos is None or battery is None:
                continue
            fullness = self.current_trash_volume / self.BASKET_CAPACITY * 100
            updateDatabase(long_lat_pos, battery, fullness)
            if self.autonomous_mode == False:
                longlat_targ = getTargetCoordinatesFromDatabase()
                if self.my_navsat_transform.origin is not None:
                    xy_targ = self.my_navsat_transform.longlat_to_xy(longlat_targ)
                    print("target (x,y): ({},{})".format(xy_targ.x, xy_targ.y))
                    self.path = [xy_targ]
                    self.path_pos = 0
    
    def update_is_object_detected(self, is_object_detected):
        self.is_object_detected = is_object_detected.data
        if not self.is_object_detected:
            self.object_dist_from_center = None
            self.object_size = None
        
    def update_object_dist_from_center(self, dist):
        self.object_dist_from_center = dist.data
    
    def update_object_size(self, size):
        self.object_size = size.data
    
    def update_object_y_pos(self, y_pos):
        self.object_y_pos = y_pos.data
        if self.object_y_pos is not None\
           and self.object_y_pos > self.OBJECT_Y_POS_NEAR_THRESH\
           and self.object_collected_timer <= 0\
           and self.object_size is not None:
            # register collected trash
            volume = self.CM3_PER_PIXEL*self.object_size
            print("Collected object of volume: {}".format(volume))
            self.current_trash_volume += volume
            fullness = self.current_trash_volume / self.BASKET_CAPACITY * 100
            self.print_once("Fullness: {}".format(fullness))
            self.object_collected_timer = self.OBJECT_COLLECTED_TIMER_INIT
            # register collected trash position
            gps_reading = self.hi.get_gps_values() # improve on
            position = utils.longlat_position(
                gps_reading[1],
                gps_reading[0]
            )
            self.trashFound.append(position)
    
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
        imu_msg.orientation_covariance[8] = self.hi.IMU_NOISE**2
        # angular velocity
        av = Vector3()
        av.x = 0
        av.y = 0
        av.z = z_axis_angular_vel
        imu_msg.angular_velocity = av
        avc = [0 for i in range(9)]
        avc[8] = self.hi.GYROSCOPE_NOISE**2
        imu_msg.angular_velocity_covariance = avc # TODO: FIX THIS
        # linear acceleration
        la = Vector3()
        la.x = 0
        la.y = y_axis_linear_acc
        la.z = 0
        imu_msg.linear_acceleration = la
        lac = [0 for i in range(9)]
        lac[4] = self.hi.ACCELEROMETER_NOISE**2
        imu_msg.linear_acceleration_covariance = lac # TODO: AND THIS
        self.imu0_pub.publish(imu_msg)

        compass_reading = self.hi.get_compass_reading()
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
            comp_cov[i] = self.hi.COMPASS_NOISE**2
        comp_msg.pose.covariance = comp_cov
        self.compass_pub.publish(comp_msg)

        p = 2

        # "gps1" reading
        gps_reading = self.hi.get_gps_values()
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.seq = self.msg_seq
        gps_msg.header.frame_id = "base_link"
        gps_msg.latitude = gps_reading[0]
        gps_msg.longitude = gps_reading[1]
        gps_msg.altitude = 0
        pos_cov = [0 for i in range(9)] # TODO: change!
        pos_cov[0] = self.hi.GPS_NOISE**p
        pos_cov[4] = self.hi.GPS_NOISE**p
        gps_msg.position_covariance = pos_cov
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN # TODO: change?
        if self.my_navsat_transform.origin is None:
            if gps_reading[0] is None or gps_reading[1] is None:
                return
            if math.isnan(gps_reading[0]) or math.isnan(gps_reading[1]):
                return
            self.my_navsat_transform.set_origin(utils.longlat_position(gps_msg.longitude, gps_msg.latitude))
        odom_gps_msg = self.my_navsat_transform.transform_to_odometry(gps_msg)
        self.gps1_odom_pub.publish(odom_gps_msg)

        # "gps2" reading
        gps_reading = self.hi.get_gps_values()
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.seq = self.msg_seq
        gps_msg.header.frame_id = "base_link"
        gps_msg.latitude = gps_reading[0]
        gps_msg.longitude = gps_reading[1]
        gps_msg.altitude = 0
        pos_cov = [0 for i in range(9)] # TODO: change!
        pos_cov[0] = self.hi.GPS_NOISE**p
        pos_cov[4] = self.hi.GPS_NOISE**p
        gps_msg.position_covariance = pos_cov
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN # TODO: change?
        odom_gps_msg = self.my_navsat_transform.transform_to_odometry(gps_msg)
        self.gps2_odom_pub.publish(odom_gps_msg)
    
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

    def print_once(self, to_print):
        if to_print != self.prev_print:
            self.prev_print = to_print
            print(to_print)
    
    def main_loop(self):
        # self.robot.step(self.timestep)
        # self.send_sensor_readings_to_localization()
        # return
        if self.object_collected_timer > 0:
            self.object_collected_timer -= 1
        next_position = self.my_navsat_transform.longlat_to_xy(self.path[self.path_pos])
        if self.is_object_detected:
            self.is_etc = False
            self.print_once("Collecting trash...")
            self.is_collecting_trash = True
            if self.rm.is_scanning:
                self.rm.is_scanning = False
            self.rm.goto_object(self.object_dist_from_center, self.object_size)
            if self.object_y_pos is not None and self.object_y_pos > 256/3:
                self.rm.open_arms()
            else:
                self.rm.close_arms()
        else:
            if self.is_collecting_trash:
                # Has collected trash
                self.is_collecting_trash = False
                self.is_etc = True
                self.etc_timer = self.ETC_TIMER_INIT
            if self.is_etc:
                self.rm.open_arms()
                if self.etc_timer <= 0:
                    self.print_once("finished etc")
                    self.is_etc = False
                    self.hi.set_left_propeller_velocity(0)
                    self.hi.set_right_propeller_velocity(0)
                else:
                    self.print_once("etc-ing")
                    self.etc_timer -= 1
                    self.hi.set_left_propeller_velocity(self.rm.APPROACH_TRASH_VELOCITY)
                    self.hi.set_right_propeller_velocity(self.rm.APPROACH_TRASH_VELOCITY)
            else:
                self.rm.close_arms()
                if self.rm.is_at(next_position) or self.rm.is_scanning:
                    if self.rm.scan():
                        self.print_once("Scanning...")
                    else:
                        self.print_once("Next step in the path...")
                        self.path_pos += 1
                        self.print_once("Position in path: {}".format(self.path_pos))
                        if self.path_pos >= len(self.path):
                            self.path_pos = 0
                            self.itter+=1 # Update the itteration
                            self.print_once("Iteration: {}".format(self.itter))
                            update = centroidUpdate.centroidUpdate(self.lochEdge, self.lochName, self.numberOfStops, self.itter, self.trashFound) # Updates the centroids to visit for the next journey
                            self.path = [self.startingPos]
                            for pt in update.centroidList: # Makes list of xy_positions
                                self.path.append(utils.longlat_position(pt[0], pt[1]))
                            self.path.append(self.startingPos)
                            # Set back to empty list for next round of finding trash
                            self.trashFound = []
                else:
                    self.print_once("moving towards next position: {}".format(next_position))
                    self.rm.goto_xy(next_position)

        # rubbish_pos = utils.xy_position(3,3)
        # if self.rm.is_in_arms_open_distance(rubbish_pos):
        #     self.rm.open_arms()
        # else:
        #     self.rm.close_arms()
        # self.rm.goto_xy(rubbish_pos)
        self.robot.step(self.timestep)
        self.send_sensor_readings_to_localization()





if __name__ == "__main__":
    mc = main_controller("StMargaretsEdge.geojson", 5, "StMargarets")
    while not rospy.is_shutdown():
        mc.main_loop()

