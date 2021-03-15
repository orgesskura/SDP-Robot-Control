from geojson import Point as geoPt
from geojson import Polygon as GeoPolygon
from geojson import Feature, FeatureCollection, dump, LineString
import json
from shapely.geometry import Polygon, Point
import random
from sklearn.cluster import KMeans
import numpy as np 
import pandas as pd
# 2Opt for finding path solution 
from numpy.linalg import norm

import sys
import math
pi = math.pi

# DONE - Import prd-drawn polygon which represents the edge of the body of water. Note Argument must be GEOJSON file containing only polygon lake edge.
def importLakeCords(path):
    with open(path, 'r') as f:
        data = f.read()
    
    lakeStr = json.loads(data)
    coords = lakeStr['features'][0]['geometry']['coordinates'][0]

    return coords

def genCirclePts(r, n):
    return [[math.cos(2*pi/n*x)*r,math.sin(2*pi/n*x)*r] for x in range(0,n+1)]

# DONE - Generate random points which lay within the polygon. 
# The code online uses shapely polygons and points. Therefor we must:
# 1. Create Shapely polygon, 2. Convert Shapely points to GeoJson points
def genPts(PolygonCords, NumPts):
    listPts = []
    for cord in PolygonCords:
        listPts.append((cord[0], cord[1]))

    # Shapely Polygon
    lakePoly = Polygon(listPts)

    # Point generator 
    points = []
    minx, miny, maxx, maxy = lakePoly.bounds
    while len(points) < NumPts:
        pnt = Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
        if lakePoly.contains(pnt):
            points.append(pnt)

    return points
    


# TODO - Find area of polygon. CAN DO LATER 

# TODO - K  = floor(How many 20m^2 goes into this area?). CAN DO LATER

# DONE IN MAIN FUNCTION - Run K-means clustering on our randomly generated points. 
# Create a numpy array containing the geojson points
# Run Sklearn.KMeans

# DONE IN MAIN FUNCTION - Create GeoJson points of the centroids of each cluster.

# DONE PARTLY IN MAIN FUNCTION - Create distance matrix and run 2-opt to find path.
def euc_dist(pt1, pt2):
    np_pt1 = np.array(pt1)
    np_pt2 = np.array(pt2)

    c = np_pt1 - np_pt2

    return np.linalg.norm(c)

def createDistTable(centroidDict):
    distTable = []
    size = len(centroidDict.keys())
    for x in range(size):
        distTable.append([])
    # Distances are euclidean distance between points 
    for x in centroidDict.keys():
        for y in centroidDict.keys():
            distTable[x].append(euc_dist(centroidDict[x], centroidDict[y]))
    
    return distTable

# TODO - Run TSP(2-opt) algorithm on this graph to find the optimal path.
def cost_change(cost_mat, n1, n2, n3, n4):
    return cost_mat[n1][n3] + cost_mat[n2][n4] - cost_mat[n1][n2] - cost_mat[n3][n4]


def two_opt(route, cost_mat):
    best = route
    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route)):
                if j - i == 1: continue
                if cost_change(cost_mat, best[i - 1], best[i], best[j - 1], best[j]) < 0:
                    best[i:j] = best[j - 1:i - 1:-1]
                    improved = True
        route = best
    return best

# DONE IN MAIN - Create geojson line string object which represents path around the lake.

# DONE IN MAIN - OUTPUT geojson file containing the first path plan

# DONE IN MAIN - Write csv file containing the randomly generated points and centroid centers, the will be required for running the algorithm again...

class firstJourney:

    def __init__(self, lochEdge, n, startingLng, startingLat, loch):

        self.lochEdge = lochEdge
        self.n = n
        self.startingLng = startingLng
        self.startingLat = startingLat
        self.loch = loch

        # Task 1 + Make GeoJson polygon from list of pts.
        startingLoc = (float(self.startingLng), float(self.startingLat))

        #LakeCords = importLakeCords(self.lochEdge)
        LakeCords = genCirclePts(0.0001, 2000)

        geojsonLakePts = [[]]
        for cord in LakeCords: 
            geojsonLakePts[0].append((cord[0], cord[1]))

        # GeoJson lake as polygon
        geoJsonLakePoly = GeoPolygon(geojsonLakePts)

        # Task 2 - Note genPts creates shapely pts so we convert to geojson pts
        PointsInLake = genPts(LakeCords, 1000)
        # Convert to GEOJSON point
        ListGeoJsonPts = []
        for pts in PointsInLake:
            ListGeoJsonPts.append(geoPt((pts.x, pts.y)))
        

        # Task 5 - Take geojson points, convert to numpy array, run KMeans, obtain centroids
        data = []
        for pts in PointsInLake:
            data.append([pts.x, pts.y])

        # Fitting the random points using KMeans 
        X = np.array(data)
        kmeans = KMeans(n_clusters = int(self.n), random_state = 0, max_iter = 1000).fit(X)

        # Task 6 - Get centroids and create geojson points from them...
        centroids = kmeans.cluster_centers_
        # Creating geojson points
        geojsonCentroidList = []
        centroidList = centroids.tolist()
        for centroid in centroidList:
            geojsonCentroidList.append(geoPt((centroid[0], centroid[1])))

        # Task 7 - Dictionary to identify each centroid. Note we have to do this to run 2-opt heuristic. 
        counter = 0
        centroid_dict = {}
        for c in centroidList:
            centroid_dict[counter] = c
            counter +=1

        names = []
        for num in centroid_dict.keys():
            names.append(num)
        table = createDistTable(centroid_dict)
        
        #route_finder = RouteFinder(table, names, iterations=20)
        # Route we will take on journey 1. 
        #best_distance, best_route = route_finder.solve()
        best_route = two_opt(names, table)

        print("The best route for EdVarka to take is: ", best_route)

        # We now want to make GeoJson line string representing the path EdVarka should be taking 
        linestrList = []
        linestrList.append(startingLoc)
        for node in best_route:
            linestrList.append((centroid_dict[int(node)][0], centroid_dict[int(node)][1]))
        linestrGeoJson = GeoPolygon([linestrList])
        linestrList.append(startingLoc)

        # For testing we write our list of randomly generated points and lake polygon to GeoJson file.
        features = []
        features.append(Feature(geometry=geoJsonLakePoly))
        # Adding line string path 
        features.append(Feature(geometry=linestrGeoJson, properties={'fill-opacity':0.0}))

        #Add starting location as red marker
        features.append(Feature(geometry=geoPt(startingLoc), properties={'marker-color':'#ff0000'}))

        #KEEP THIS TO SHOW THE MARKERS WHICH INITIALISE THE CENTROIDS
        #for pt in listGeoJsonPts:
        #   features.append(Feature(geometry=pt))

        for pts in geojsonCentroidList:
            features.append(Feature(geometry=pts, properties={'marker-color':'#0000ff'}))

        feature_collection = FeatureCollection(features)

        # We need to add a single polygon representing the points we want to visit on our path plan 
        path_plan = []
        path_plan.append(Feature(geometry=linestrGeoJson, properties={'fill-opacity':0.0}))
        path_plan_fc = FeatureCollection(path_plan)

        # Standard file writing for our geojson path to follow
        #file_to_write = '1stPathPlan' + self.loch + '.geojson'
        #with open(file_to_write, 'w') as f:
        #    f.seek(0)
        #    dump(path_plan_fc, f)

        # Standard file writing for our geojson
        #file_to_write = '1stJourney' + self.loch + '.geojson'
        #with open(file_to_write, 'w') as f:
        #    f.seek(0)
        #    dump(feature_collection, f)

        # Task 10 - Standar file writing for saving our centroid centers, data set X and exact path taken(Not needed as path probably will change).
        random_df = pd.DataFrame(data = X)
        centroids_df = pd.DataFrame(data = centroids, columns=['lng', 'lat'])
        # Add pandas df of starting loc...
        cordStartLoc = np.array([startingLoc[0], startingLoc[1]]).reshape(1,2)
        startingLoc_df = pd.DataFrame(data = cordStartLoc)

        random_df.to_csv(self.loch + '_rand.csv')
        centroids_df.to_csv(self.loch + '_centroids.csv')
        startingLoc_df.to_csv(self.loch + '_startingLoc.csv')

        self.centroidList = centroidList # Used to integrate with Evripidis stuff


if __name__ == '__main__':
    #main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])

    x = firstJourney("StMargaretsEdge.geojson", 10, 0, 0, "StMargarets")
    for cent in x.centroidList:
        print(round(cent[0],1))
        print(round(cent[1],1))