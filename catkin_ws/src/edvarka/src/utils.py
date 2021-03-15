import math

class xy_position:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "(x: {}, y: {})".format(self.x, self.y) 

class longlat_position:
    def __init__(self, longitude, latitude):
        self.longitude = longitude
        self.latitude = latitude

    def __str__(self):
        return "(longitude: {}, latitude: {})".format(self.longitude, self.latitude) 

# Returns the anti-clockwise angle from the y-axis of position1 to position2
def get_yaw_xy(position1, position2):
        atan = math.atan2(position2.y - position1.y,
                          position2.x - position1.x)
        yaw_diff = atan - math.pi/2
        if yaw_diff > math.pi:
            yaw_diff -= 2*math.pi
        if yaw_diff < -math.pi:
            yaw_diff += 2*math.pi
        return yaw_diff

# Returns the distance in meters between two x, y points
def get_distance_between_xy(position1, position2):
    return math.sqrt((position1.y - position2.y)**2 + (position1.x - position2.x)**2)

# Returns the distance in meters between two longitude, latitude points
def get_distance_between_longlat(position1, position2):
    sin_delta_lat = math.sin(math.radians(position1.latitude - position2.latitude)/2)
    sin_delta_long = math.sin(math.radians(position1.longitude - position2.longitude)/2)
    cos_lat1 = math.cos(math.radians(position1.latitude))
    cos_lat2 = math.cos(math.radians(position2.latitude))
    a = sin_delta_lat**2 + cos_lat1*cos_lat2*(sin_delta_long**2)
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    R = 6371e3 # average Earth radius in meters
    return R*c

def angle_between_xy_vectors(v1, v2):
    dot = v1.y*v2.y + v1.x*v2.x
    mag1 = math.sqrt(v1.y**2 + v1.x**2)
    mag2 = math.sqrt(v2.y**2 + v2.x**2)
    return math.acos(dot/(mag1*mag2))