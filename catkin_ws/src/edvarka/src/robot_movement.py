
import math
import tf

import utils

class robot_movement:
    def __init__(self, hardware_interface):
        self.hi = hardware_interface
        # constants
        self.MAX_FACING_THRESHOLD = math.radians(15)
        self.MIN_FACING_THRESHOLD = math.radians(5)
        self.FACING_THRESHOLD = math.radians(5)
        self.DISTANCE_THRESHOLD = 0.1 # meter(s)
        self.ARMS_OPEN_DISTANCE = 1
        self.MAX_ROT_V = 2
        self.ARMS_OPEN = 0
        self.ARMS_CLOSED = 1.5
        # PΙD controller for facing
        self.INCLUDE_I_TERM_THRESHOLD_F = math.radians(10)
        self.last_f_error = 0
        self.f_Kp = 3
        self.f_Kd = 800
        self.f_Ki = 0.2
        # PID controller for travelling
        self.INCLUDE_I_TERM_THRESHOLD_T = 0.4
        self.last_t_error = 0
        self.t_Kp = 1
        self.t_Kd = 500
        self.t_Ki = 0.1
        # Timer for stable facing
        self.TIMER_DURATION = 10 # steps
        self.is_timing = False
        self.timer = self.TIMER_DURATION
        # Estimated robot state, gets updated by the main controller through the EKF node
        self.robot_state = None
        self.DEFAULT_POSITION = utils.xy_position(0,0)
        self.DEFAULT_YAW = 0
    
    # Returns the robot's current estimate of its position on the xy plane
    # as a utils.xy_position object
    def get_own_position(self):
        if self.robot_state is None:
            return self.DEFAULT_POSITION
        x = self.robot_state.pose.pose.position.x
        y = self.robot_state.pose.pose.position.y
        return utils.xy_position(x, y)
    
    # Returns the robot's current estimate of its yaw. Yaw is zero when the 
    # robot is facing north.
    def get_own_yaw(self):
        if self.robot_state is None:
            return self.DEFAULT_YAW
        quaternion = self.robot_state.pose.pose.orientation
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        return yaw

    # Returns the least-magnitude anti-clockwise angle the robot has to 
    # rotate by to face the target position (given as a utils.xy_position).
    def get_angle_to_target(self, target_position):
        own_yaw = self.get_own_yaw()
        own_position = self.get_own_position()
        target_yaw = utils.get_yaw_xy(own_position, target_position)
        return self.get_angle_to_yaw(target_yaw)
    
    # Returns the least-magnitude anti-clockwise angle the robot has to 
    # rotate by to face in the desired direction (given as yaw).
    def get_angle_to_yaw(self, target_yaw):
        own_yaw = self.get_own_yaw()
        ret = target_yaw - own_yaw
        print("Yaw: {}\nAngle to: {}".format(own_yaw, ret))
        # answer must be between -π and π
        while ret < math.pi:
            ret += 2*math.pi
        while ret > math.pi:
            ret -= 2*math.pi
        return ret
    
    # Calculates the variable facing threshold, which increases the closer 
    # the robot is to the target.
    def get_variable_facing_threshold(self, target_position):
        dist = self.get_distance_to_target(target_position)
        thresh = math.radians(-10/3.0*dist + 55/3.0)
        if thresh < self.MIN_FACING_THRESHOLD:
            thresh = self.MIN_FACING_THRESHOLD
        if thresh > self.MAX_FACING_THRESHOLD:
            thresh = self.MAX_FACING_THRESHOLD
        return thresh

    # Returns true if the robot is facing the target position within the
    # acceptable error.
    def is_facing_target(self, target_position):
        angle_to_target = self.get_angle_to_target(target_position)
        return abs(angle_to_target) < self.get_variable_facing_threshold(target_position)
    
    # Returns true if the robot is facing in the desired direction within
    # the acceptable error.
    def is_facing_yaw(self, target_yaw):
        error = target_yaw - self.get_own_yaw()
        return error < self.FACING_THRESHOLD
    
    # Adjusts the speed of the propellers using a PID controller in order to
    # achieve the goal of the boat facing the target position.
    def face_target(self, target_position):
        own_position = self.get_own_position()
        target_yaw = utils.get_yaw_xy(own_position, target_position)
        self.face_yaw(target_yaw)

    # Adjusts the speed of the propellers using a PID controller in order to
    # achieve the goal of the boat facing in the target yaw (measured anti-clockwise)
    # from north (y-axis)).
    def face_yaw(self, target_yaw):
        error = self.get_angle_to_yaw(target_yaw)
        print("face error: {}".format(error))
        error_derivative = (error - self.last_f_error) / self.hi.timestep
        self.last_f_error = error
        error_integral = error * self.hi.timestep
        include_i_term = 0
        if abs(error) < self.INCLUDE_I_TERM_THRESHOLD_F:
            include_i_term = 1
        rotation_velocity = (self.f_Kp*error + self.f_Kd*error_derivative + include_i_term*self.f_Ki*error_integral)
        rotation_velocity = self.cap_propeller_velocity(rotation_velocity)
        self.hi.set_left_propeller_velocity(-rotation_velocity)
        self.hi.set_right_propeller_velocity(rotation_velocity)
    
    # Returns the distance, in meters, from the boat to the target (given as util.xy_position)
    def get_distance_to_target(self, target_position):
        own_position = self.get_own_position()
        return utils.get_distance_between_xy(own_position, target_position)

    # Returns true if the target location is close enough to open the robot's arms
    def is_in_arms_open_distance(self, target_position):
        dist = self.get_distance_to_target(target_position)
        return dist < self.ARMS_OPEN_DISTANCE
    
    # Returns true if the robot is located at the target location, within the acceptable error
    def is_at(self, target_position):
        dist = self.get_distance_to_target(target_position)
        return dist < self.DISTANCE_THRESHOLD

    def cap_propeller_velocity(self, v):
        if abs(v) > self.MAX_ROT_V:
            v = v/abs(v) * self.MAX_ROT_V
        return v
    
    # Adjusts the speed of the propellers using a PID controller in order to move the robot towards
    # a target location, specified as a utils.xy_position. Requires the robot to be facing the target.
    def move_towards(self, target_position):
        if not self.is_facing_target(target_position):
            self.face_target(target_position)
            return
        error = self.get_distance_to_target(target_position)
        error_derivative = (error - self.last_t_error) / self.hi.timestep
        self.last_t_error = error
        error_integral = error * self.hi.timestep
        include_i_term = 0
        if abs(error) < self.INCLUDE_I_TERM_THRESHOLD_T:
            include_i_term = 1
        rotation_velocity = (self.t_Kp*error + self.t_Kd*error_derivative + include_i_term*self.t_Ki*error_integral)
        rotation_velocity = self.cap_propeller_velocity(rotation_velocity)
        self.hi.set_right_propeller_velocity(rotation_velocity)
        self.hi.set_left_propeller_velocity(rotation_velocity)
    
    # Controls the robot's motors in order to face and move the robot towards the desired location.
    def goto_xy(self, target_position):
        if not self.is_facing_target(target_position):
            self.is_timing = False
            self.face_target(target_position)
        elif not self.is_timing:
            self.is_timing = True
            self.timer = self.TIMER_DURATION # steps
        elif self.timer != 0:
            self.timer -= 1
        else:
            self.move_towards(target_position)
        
    def open_arms(self):
        self.hi.set_arms_position(self.ARMS_OPEN)
    
    def close_arms(self):
        self.hi.set_arms_position(self.ARMS_CLOSED)
        

