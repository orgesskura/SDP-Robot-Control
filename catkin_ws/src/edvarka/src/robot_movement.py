
import math
import tf

import utils

class robot_movement:
    def __init__(self, hardware_interface):
        self.hi = hardware_interface
        # constants
        self.MAX_FACING_THRESHOLD = math.radians(15)
        self.MIN_FACING_THRESHOLD = math.radians(10)
        self.FACING_THRESHOLD = math.radians(10)
        self.DISTANCE_THRESHOLD = 1 # meter(s)
        self.ARMS_OPEN_DISTANCE = 1
        self.MAX_ROT_V = 2
        self.MAX_ROT_V_FACING = 5
        self.ARMS_OPEN = 0
        self.ARMS_CLOSED = 1.5
        self.OBJECT_FACING_THRESHOLD = 40
        self.APPROACH_TRASH_VELOCITY = 1.1
        self.SMALL_OBJ_THRESH = (256*256*0.0000001) # 1/1000th of the image
        # PΙD controller for facing
        self.INCLUDE_I_TERM_THRESHOLD_F = math.radians(40)
        self.last_f_error = 0
        self.f_Kp = 1
        self.f_Kd = 200
        self.f_Ki = 0.1
        # PID controller for travelling
        self.INCLUDE_I_TERM_THRESHOLD_T = 0.4
        self.last_t_error = 0
        self.t_Kp = 1
        self.t_Kd = 300
        self.t_Ki = 0.1
        # PID controller for facing (using vision)
        self.INCLUDE_I_TERM_THRESHOLD_V = 60
        self.last_v_error = 0
        self.v_Kp = 0.01
        self.v_Kd = 20
        self.v_Ki = 0.001
        # Timer for stable facing
        self.TIMER_DURATION = 5 # steps
        self.is_timing = False
        self.timer = self.TIMER_DURATION
        # Estimated robot state, gets updated by the main controller through the EKF node
        self.robot_state = None
        self.DEFAULT_POSITION = utils.xy_position(0,0)
        self.DEFAULT_YAW = 0
        # scan
        self.is_scanning = False
        self.scan_end_yaw = 0
        self.scan_target_list = []
        self.scan_target = None

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
        # answer must be between -π and π
        while ret < -math.pi:
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
        error = self.get_angle_to_yaw(target_yaw)
        return abs(error) < self.FACING_THRESHOLD
    
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
        error_derivative = (error - self.last_f_error) / self.hi.timestep
        self.last_f_error = error
        error_integral = error * self.hi.timestep
        include_i_term = 0
        if abs(error) < self.INCLUDE_I_TERM_THRESHOLD_F:
            include_i_term = 1
        rotation_velocity = (self.f_Kp*error + self.f_Kd*error_derivative + include_i_term*self.f_Ki*error_integral)
        print("error:", error)
        print("error_d:", error_derivative)
        print("error_i:", error_integral)
        print("rot_v:", rotation_velocity)
        rotation_velocity = self.cap_propeller_facing_velocity(rotation_velocity)
        self.hi.set_left_propeller_velocity(-rotation_velocity)
        self.hi.set_right_propeller_velocity(rotation_velocity)
    
    # Adjusts the speed of the propellers using a PID controller in order to
    # achieve the goal of the boat facing in the target yaw (measured anti-clockwise)
    # from north (y-axis)).
    def face_object_vision(self, object_pos_in_image):
        if object_pos_in_image is None:
            self.hi.set_left_propeller_velocity(0)
            self.hi.set_right_propeller_velocity(0)
            return
        error = object_pos_in_image
        error_derivative = (error - self.last_v_error) / self.hi.timestep
        print("Derv: ", error_derivative)
        self.last_v_error = error
        error_integral = error * self.hi.timestep
        include_i_term = 0
        if abs(error) < self.INCLUDE_I_TERM_THRESHOLD_V:
            include_i_term = 1
        rotation_velocity = (self.v_Kp*error + self.v_Kd*error_derivative + include_i_term*self.v_Ki*error_integral)
        rotation_velocity = self.cap_propeller_facing_velocity(rotation_velocity)
        self.hi.set_left_propeller_velocity(rotation_velocity)
        self.hi.set_right_propeller_velocity(-rotation_velocity)
    
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

    def cap_propeller_facing_velocity(self, v):
        if abs(v) > self.MAX_ROT_V_FACING:
            v = v/abs(v) * self.MAX_ROT_V_FACING
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
    
    def is_facing_object(self, object_center_distance, object_size):
        if object_size < self.SMALL_OBJ_THRESH:
            # for really small/far away objects, rotating might throw them off the
            # detection threshold
            return True
        return abs(object_center_distance) < self.OBJECT_FACING_THRESHOLD

    def goto_object(self, object_center_distance, object_size):
        if object_center_distance is None or object_size is None:
            return
        if not self.is_facing_object(object_center_distance, object_size):
            self.is_timing = False
            self.face_object_vision(object_center_distance)
        elif not self.is_timing:
            self.is_timing = True
            self.timer = self.TIMER_DURATION # steps
        elif self.timer != 0:
            self.timer -= 1
        else:
            self.hi.set_left_propeller_velocity(self.APPROACH_TRASH_VELOCITY)
            self.hi.set_right_propeller_velocity(self.APPROACH_TRASH_VELOCITY)
    
    def scan(self):
        if not self.is_scanning:
            self.is_scanning = True
            own_yaw = self.get_own_yaw()
            divisions = 4
            offsets = [i*2*math.pi/divisions for i in range(1,divisions+1)]
            self.scan_target_list = [own_yaw + offset for offset in offsets]
            for i in range(divisions):
                while self.scan_target_list[i] > math.pi:
                    self.scan_target_list[i] -= 2*math.pi
                while self.scan_target_list[i] < -math.pi:
                    self.scan_target_list[i] += 2*math.pi
            self.scan_target = 0
        target_yaw = self.scan_target_list[self.scan_target]
        if self.is_facing_yaw(target_yaw):
            self.scan_target += 1
            if self.scan_target >= len(self.scan_target_list):
                self.is_scanning = False
                return False
        else:
            self.face_yaw(self.scan_target_list[self.scan_target])
        return True

        
        
    def open_arms(self):
        self.hi.set_arms_position(self.ARMS_OPEN)
    
    def close_arms(self):
        self.hi.set_arms_position(self.ARMS_CLOSED)
