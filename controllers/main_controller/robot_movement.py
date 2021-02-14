import utils
import math

class robot_movement:
    def __init__(self, hardware_interface):
        self.hi = hardware_interface
        # constants
        self.FACING_THRESHOLD = math.radians(5)
        self.DISTANCE_THRESHOLD = 0.5 # meter(s)
        # PΙD controller for facing
        self.INCLUDE_I_TERM_THRESHOLD_F = math.radians(12)
        self.last_f_error = 0
        self.f_Kp = 2.5
        self.f_Kd = 1000
        self.f_Ki = 0.3
        # PID controller for travelling
        self.INCLUDE_I_TERM_THRESHOLD_T = 0.4
        self.last_t_error = 0
        self.t_Kp = 3
        self.t_Kd = 4
        self.t_Ki = 0.3

    def get_angle_to_target(self, target_position):
        own_azimuth = self.hi.get_compass_reading()
        gps_reading = self.hi.get_gps_values()
        own_position = utils.position(gps_reading[1], gps_reading[0])
        print("Boat coords: ", gps_reading[1], gps_reading[0])
        azimuth_to_target = utils.get_angle_between(own_position, target_position)
        ret = azimuth_to_target - own_azimuth
        if abs(ret) > math.pi:
            if ret < 0:
                ret += 2*math.pi
            else:
                ret -= 2*math.pi
        return ret
    
    def is_facing(self, target_position):
        angle_to_target = self.get_angle_to_target(target_position)
        return abs(angle_to_target) < self.FACING_THRESHOLD
    
    def face(self, target_position):
        error = self.get_angle_to_target(target_position)
        error_derivative = (error - self.last_f_error) / self.hi.timestep
        self.last_f_error = error
        error_integral = error * self.hi.timestep
        include_i_term = 0
        if abs(error) < self.INCLUDE_I_TERM_THRESHOLD_F:
            include_i_term = 1
        rotation_velocity = (self.f_Kp*error + self.f_Kd*error_derivative + include_i_term*self.f_Ki*error_integral)
        self.hi.set_left_propeller_velocity(rotation_velocity)
        self.hi.set_right_propeller_velocity(-rotation_velocity)
    
    def get_distance_to_target(self, target_position):
        gps_reading = self.hi.get_gps_values()
        own_position = utils.position(gps_reading[1], gps_reading[0])
        print("Boat coords: {}".format(own_position.longitude,own_position.latitude))
        return utils.get_distance_between(own_position, target_position)

    def is_in_proximity(self, target_position):
        dist = self.get_distance_to_target(target_position)
        return dist < self.DISTANCE_THRESHOLD
    
    def move_towards(self, target_position):
        if not self.is_facing(target_position):
            self.face(target_position)
            return
        error = self.get_distance_to_target(target_position) - self.DISTANCE_THRESHOLD
        error_derivative = (error - self.last_t_error) / self.hi.timestep
        self.last_t_error = error
        error_integral = error * self.hi.timestep
        include_i_term = 0
        if abs(error) < self.INCLUDE_I_TERM_THRESHOLD_T:
            include_i_term = 1
        rotation_velocity = (self.t_Kp*error + self.t_Kd*error_derivative + include_i_term*self.t_Ki*error_integral)
        self.hi.set_right_propeller_velocity(rotation_velocity)
        self.hi.set_left_propeller_velocity(rotation_velocity)
    
    def goto_long_lat(self, target_position):
        if not self.is_facing(target_position):
            self.face(target_position)
        elif not self.hi.propellers_have_same_velocity():
            self.hi.set_left_propeller_velocity(0)
            self.hi.set_right_propeller_velocity(0)
            for i in range(15):
                self.hi.robot.step(self.hi.timestep)
        else:
            self.move_towards(target_position)
        

