import math

class position:

    def __init__(self, longitude, latitude):
        self.longitude = longitude
        self.latitude = latitude
        
    
# Returns the clockwise angle from the north axis of position1 to position 2
# (azimuth)
def get_angle_between(position1, position2):
        atan = math.atan2(position2.latitude - position1.latitude,
                          position2.longitude - position1.longitude)
        azimuth = -atan + math.pi/2
        if azimuth > math.pi:
            azimuth -= 2*math.pi
        return azimuth

# Returns the distance in meters between two longitude, latitude points
def get_distance_between(position1, position2):
    sin_delta_lat = math.sin(math.radians(position1.latitude - position2.latitude)/2)
    sin_delta_long = math.sin(math.radians(position1.longitude - position2.longitude)/2)
    cos_lat1 = math.cos(math.radians(position1.latitude))
    cos_lat2 = math.cos(math.radians(position2.latitude))
    a = sin_delta_lat**2 + cos_lat1*cos_lat2*(sin_delta_long**2)
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    R = 6371e3 # average Earth radius in meters
    return R*c

def angle_between_vectors(v1, v2):
    dot = v1.latitude*v2.latitude + v1.longitude*v2.longitude
    mag1 = math.sqrt(v1.latitude**2 + v1.longitude**2)
    mag2 = math.sqrt(v2.latitude**2 + v2.longitude**2)
    return math.acos(dot/(mag1*mag2))