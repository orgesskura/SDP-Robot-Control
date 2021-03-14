import utils
import pandas as pd
import numpy as np 
import robot_movement
import main_controller

# Extra stuff Evripidis needs to import 
import centroidUpdate
import firstJourney

# All of Evripidis functions for the robot use utilxy classes 
def mock_init(self, lochEdge, n, startingLng, startingLat, loch):
    # path planning
    self.lochEdge = lochEdge
    self.loch = loch
    self.n = n
    self.startingLng = startingLng
    self.startingLat = startingLat
    self.itter = 0 # This is the number of itterations of the lake that EdVarka has done 
    self.autonomous_mode = True # default should be True?
    self.firstPath = firstJourney(self.lochEdge, self.n, self.startingLng, self.startingLat, self.loch) # This will give us a set of points to visit
    self.startingPos = xy_position(self.startingLng, self.startingLat) # Starting position
    self.trashFound = []

    self.path = [self.startingPos] # Takes list of centroid coordinates and converts to utils xy_position
    for cent in self.firstPath.centroidList:
        self.path.append(xy_position(round(cent[0],1) round(cent[1],1)))
    self.path_pos = 0

def main_loop(self):

    next_position = self.path[self.path_pos]
    if self.is_object_detected:
        if self.rm.is_scanning:
            self.rm.is_scanning = False
        self.rm.goto_object(self.object_dist_from_center, self.object_size)
        self.rm.open_arms()

        # if is_at(object_position): ASK EVRIPIDIS ABOUT HOW DO GET GUESS X,Y OF THE TARGET
            #trashFound.append(get_own_position().x, get_own_position().y)
    else:
        self.rm.close_arms()
        if self.rm.is_at(next_position) or self.rm.is_scanning:
            if self.rm.scan():
                pass
            else:
                self.path_pos += 1
                if self.path_pos >= len(self.path):
                    self.path_pos = 0
                    self.itter+=1 # Update the itteration
                    update = centroidUpdate(self.lochEdge, self.loch, 10, self.itter, self.trashFound) # Updates the centroids to visit for the next journey
                    self.path = [startingPos]
                    for pt in update.centroidList: # Makes list of xy_positions
                        self.path.append(xy_position(round(pt[0],1) round(pt[1],1)))

                    # Set back to empty list for next round of finding trash
                    self.trashFound = []

        else:
            self.rm.goto_xy(self.path[self.path_pos])

    # rubbish_pos = utils.xy_position(3,3)
    # if self.rm.is_in_arms_open_distance(rubbish_pos):
    #     self.rm.open_arms()
    # else:
    #     self.rm.close_arms()
    # self.rm.goto_xy(rubbish_pos)
    self.robot.step(self.timestep)
    self.send_sensor_readings_to_localization()

