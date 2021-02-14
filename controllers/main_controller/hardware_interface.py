from controller import Robot
import utils

class hardware_interface:

    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        # sensors
        self.compass = self.robot.getDevice("compass")
        self.gps = self.robot.getDevice("gps")
        # actuators
        self.left_propeller = self.robot.getDevice("left_propeller_motor")
        self.right_propeller = self.robot.getDevice("right_propeller_motor")
        self.set_left_propeller_position(float('+inf'))
        self.set_right_propeller_position(float('+inf'))

    def enable_devices(self):
        self.gps.enable(self.timestep)
        self.compass.enable(self.timestep)

    def get_gps_values(self):
        return self.gps.getValues()

    def get_compass_reading(self):
        reading = self.compass.getValues()
        #print("Reading: {}".format(reading))
        pos1 = utils.position(0,0.5)
        pos2 = utils.position(reading[0], reading[1])
        angle = utils.angle_between_vectors(pos1, pos2)
        if reading[0] > 0:
            angle *= -1
        return angle
    
    def propellers_have_same_velocity(self):
        return self.left_propeller.getVelocity() == self.right_propeller.getVelocity()

    def set_left_propeller_position(self, p):
        self.set_propeller_position(self.left_propeller, p)

    def set_left_propeller_velocity(self, v):
        self.set_propeller_velocity(self.left_propeller, v)

    def set_left_propeller_acceleration(self, a):
        self.set_propeller_acceleration(self.left_propeller, a)

    def set_right_propeller_position(self, p):
        self.set_propeller_position(self.right_propeller, p)

    def set_right_propeller_velocity(self, v):
        self.set_propeller_velocity(self.right_propeller, v)

    def set_right_propeller_acceleration(self, a):
        self.set_propeller_acceleration(self.right_propeller, a)

    def set_propeller_position(self, propeller, p):
        propeller.setPosition(p)

    def set_propeller_velocity(self, propeller, v):
        propeller.setVelocity(v)

    def set_propeller_acceleration(self, propeller, a):
        propeller.setAcceleration(a)

