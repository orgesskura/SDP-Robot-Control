import numpy.random
import numpy as np
import math

from controller import Robot
import utils

class hardware_interface:

    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        # sensors
        self.compass = self.robot.getDevice("compass")
        self.gps = self.robot.getDevice("gps")
        self.accelerometer = self.robot.getDevice("accelerometer")
        self.gyro = self.robot.getDevice("gyro")
        self.imu = self.robot.getDevice("imu")
        self.front_camera = self.robot.getDevice("front_camera")
        self.water_camera = self.robot.getDevice("water_camera")
        # actuators
        self.left_propeller = self.robot.getDevice("left_propeller_motor")
        self.right_propeller = self.robot.getDevice("right_propeller_motor")
        self.set_left_propeller_position(float('+inf'))
        self.set_right_propeller_position(float('+inf'))
        self.set_left_propeller_velocity(0)
        self.set_right_propeller_velocity(0)
        self.left_arm = self.robot.getDevice("left_arm_motor")
        self.right_arm = self.robot.getDevice("right_arm_motor")
        # noises
        self.COMPASS_NOISE = math.radians(0.5)
        self.GPS_NOISE = 1 # meters
        self.ACCELEROMETER_NOISE = 0.1 # m/s^2
        self.GYROSCOPE_NOISE = math.radians(0.5) # rad/s
        self.IMU_NOISE = math.radians(0.5)

    def enable_devices(self):
        self.gps.enable(self.timestep)
        self.compass.enable(self.timestep)
        self.accelerometer.enable(self.timestep)
        self.gyro.enable(self.timestep)
        self.imu.enable(self.timestep)
        self.front_camera.enable(self.timestep)
        self.water_camera.enable(self.timestep)

    def get_front_camera_image(self):
        cam = self.front_camera
        camera_data = cam.getImage()
        if camera_data is None:
            return None
        image = np.frombuffer(camera_data, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))
        return image[:, :, :-1]
    
    def get_water_camera_image(self):
        cam = self.water_camera
        camera_data = cam.getImage()
        if camera_data is None:
            return None
        image = np.frombuffer(camera_data, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))
        return image[:, :, :-1]

    def get_gps_values(self):
        reading = self.gps.getValues()
        return reading

    # Returns the counter-clockwise angle in the range [-π, π] that the boat
    # frame's y axis forms with the "true north", i.e. the global-frame y axis.
    def get_compass_reading(self):
        reading = self.compass.getValues()
        pos1 = utils.xy_position(0,0.5)
        pos2 = utils.xy_position(reading[0], reading[1])
        angle = utils.angle_between_xy_vectors(pos1, pos2)
        if reading[0] < 0:
            angle *= -1
        noise = numpy.random.normal(loc=0, scale=self.COMPASS_NOISE, size=1)[0]
        angle += noise
        return angle
    
    def get_accelerometer_reading(self):
        reading = self.accelerometer.getValues()
        noise = numpy.random.normal(loc=0, scale=self.ACCELEROMETER_NOISE, size=3)
        reading[0] += noise[0]
        reading[1] += noise[1]
        reading[2] += noise[2]
        return reading
    
    def get_gyro_reading(self):
        reading = self.gyro.getValues()
        noise = numpy.random.normal(loc=0, scale=self.GYROSCOPE_NOISE, size=3)
        reading[0] += noise[0]
        reading[1] += noise[1]
        reading[2] += noise[2]
        return reading
    
    def get_imu_reading(self):
        reading = self.imu.getRollPitchYaw()
        noise = numpy.random.normal(loc=0, scale=self.IMU_NOISE, size=3)
        reading[0] += noise[0]
        reading[1] += noise[1]
        reading[2] += noise[2]
        return reading
    
    def propellers_have_same_velocity(self):
        return self.left_propeller.getVelocity() == self.right_propeller.getVelocity()

    def set_left_propeller_position(self, p):
        self.set_propeller_position(self.left_propeller, p)

    def set_left_propeller_velocity(self, v):
        # print("Left v: {}".format(v))
        self.set_propeller_velocity(self.left_propeller, v)

    def set_left_propeller_acceleration(self, a):
        self.set_propeller_acceleration(self.left_propeller, a)

    def set_right_propeller_position(self, p):
        self.set_propeller_position(self.right_propeller, p)

    def set_right_propeller_velocity(self, v):
        # print("Right v: {}".format(v))
        self.set_propeller_velocity(self.right_propeller, v)

    def set_right_propeller_acceleration(self, a):
        self.set_propeller_acceleration(self.right_propeller, a)

    def set_propeller_position(self, propeller, p):
        propeller.setPosition(p)

    def set_propeller_velocity(self, propeller, v):
        propeller.setVelocity(v)

    def set_propeller_acceleration(self, propeller, a):
        propeller.setAcceleration(a)

    def set_arms_position(self, p):
        self.left_arm.setPosition(p)
        self.right_arm.setPosition(p)


# %%
