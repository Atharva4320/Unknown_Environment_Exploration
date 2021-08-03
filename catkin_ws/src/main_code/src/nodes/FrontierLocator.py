#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path, OccupancyGrid, GridCells
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped

class FrontierLocator:

    def __init__(self):
        rospy.init_node("frontier_locator", anonymous=True)
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        rospy.Service("frontier_point", GetPlan, self.locateFrontier)

        ## Initialize the request counter
        self.getMap = rospy.Subscriber("/cspace_map", OccupancyGrid,self.updateMap)

        self.frontier = rospy.Publisher('/frontier', GridCells,queue_size=5)
        self.pubPoint = rospy.Publisher('/centroid', PointStamped, queue_size=5)

        print("Initialized frontier locator")

    def updateMap(self,msg):
        self.mapdata=msg

    def locateFrontier(self,msg):
        """
        Find a path and return it - msg contains the cspace map
        """
        robotLocation = msg.start.pose.position # has x,y,z of robot
        print("Callback runs")

        # Locate frontiers
        edgecells = self.frontierLocate()
        print("Have published edge cells")
        frontiers = self.getFrontiers(edgecells)
        centroidPoint = self.bestCentroid(self.mapdata, frontiers,(msg.start.pose.position.x,msg.start.pose.position.y))

        centroid = PoseStamped()
        centroid.header.frame_id = "map"
        centroid.pose.position.x = centroidPoint.x
        centroid.pose.position.y = centroidPoint.y

        path_msg = Path()
        path_msg.poses.append(centroid)
        return path_msg

    def frontierLocate(self):
        # Locate frontiers
        gridData = self.mapdata.data
        pointsList = []
        ## Go through each cell in the occupancy grid
        for i in range(len(gridData)):
            ## Inflate the obstacles where necessary
            
            if gridData[i]== -1:
                coordinates = self.indexToGrid(self.mapdata, i)
                aroundCells = self.neighborsOf8(self.mapdata, coordinates[0], coordinates[1], 0)

                for j in aroundCells:
                    indJ = self.gridToIndex(self.mapdata, j[0], j[1])
                    if indJ < len(gridData) and gridData[indJ] == 0:
                        coord = self.indexToGrid(self.mapdata, indJ)
                        wrldCoord = self.gridToWorld(
                            self.mapdata, coord[0], coord[1])
                        pointsList.append(wrldCoord)

        message = GridCells()

        message.header = self.mapdata.header
        message.cell_width = self.mapdata.info.resolution
        message.cell_height = self.mapdata.info.resolution
        message.cells = pointsList

        self.frontier.publish(message)

        return message

    def getFrontiers(self, edgecells):
        frontierList = []
        cells = edgecells.cells
        cellGrid = []
        for cell in cells:
            cellGrid.append(FrontierLocator.worldToGrid(self.mapdata,cell))

        listOfFrontiers = []
        while len(cellGrid) > 0:
            chain = []
            chainList = self.startChain(self.mapdata,cellGrid,chain,cellGrid[0])
            cellGrid = self.removeDuplicates(cellGrid,chainList)
            listOfFrontiers.append(chainList)

        print("Number of Frontiers Found", len(listOfFrontiers))
        return listOfFrontiers
        # Find Biggest Frontier and set it as one to go to.. this should be closest maybe? 

    def bestCentroid(self, mapdata, listOfFrontiers, loc):
        goodFrontiers = []
        allCentroids = []
        minLengthTolerance = 20
        # Remove Short Frontiers
        for frontier in listOfFrontiers:
            if len(frontier) >= minLengthTolerance:
                goodFrontiers.append(frontier)

        #Calculate Center Frontier
        allPoints = []
        for frontier in goodFrontiers:
            for cell in frontier:
                allPoints.append(cell)

        centroidOfAll = self.selectCentroid(allPoints)        

        #Calculate Biggest Frontier
        biggestFrontier = goodFrontiers[0]
        for frontier in goodFrontiers:
            if len(frontier) > len(biggestFrontier):
                biggestFrontier = frontier
        biggestCentroid = self.selectCentroid(biggestFrontier)
    

        #Get Robot Location
        robotLocation = Point()
        robotLocation.x = loc[0]
        robotLocation.y = loc[1]
        robotGrid = FrontierLocator.worldToGrid(mapdata,robotLocation)

        #Calculate Closest Frontier
        closestFrontier = goodFrontiers[0]
        closestFrontierCentroid = self.selectCentroid(closestFrontier)
        centroidValue = mapdata.data[self.gridToIndex(mapdata,closestFrontierCentroid.x, closestFrontierCentroid.y)]
        closestFrontierDistance = FrontierLocator.euclideanDistance(robotGrid[0], robotGrid[1], closestFrontierCentroid.x, closestFrontierCentroid.y)

        for frontier in goodFrontiers:
            centroid = self.selectCentroid(frontier)
            distance = FrontierLocator.euclideanDistance(robotGrid[0], robotGrid[1], centroid.x, centroid.y)
            centroidValue = mapdata.data[self.gridToIndex(mapdata,centroid.x, centroid.y)]
            allCentroids.append(centroid)
            if distance < closestFrontierDistance and centroidValue < 60:
                closestFrontier = frontier
                closestFrontierDistance = distance
        closestFrontierCentroid = self.selectCentroid(closestFrontier)
        

        # Select a Centroid
        # Convert Centroid to World Coordinates
        closestFrontierValue = mapdata.data[self.gridToIndex(mapdata,closestFrontierCentroid.x, closestFrontierCentroid.y)]
        centroidOfAllValue = mapdata.data[self.gridToIndex(mapdata,centroidOfAll.x, centroidOfAll.y)]
        biggestCentroidValue = mapdata.data[self.gridToIndex(mapdata,biggestCentroid.x, biggestCentroid.y)]
        print("Center Value",centroidOfAllValue )

        if centroidOfAllValue < 60 and centroidOfAllValue > 0 and len(goodFrontiers) <=2:
            bestCentroid = FrontierLocator.gridToWorld(mapdata, centroidOfAll.x, centroidOfAll.y)
        elif biggestCentroidValue < 60:
            bestCentroid = FrontierLocator.gridToWorld(mapdata, biggestCentroid.x, biggestCentroid.y)
            print("Going to Center Centroid")
        
        elif closestFrontierValue < 60:
            bestCentroid = FrontierLocator.gridToWorld(mapdata, closestFrontierCentroid.x, closestFrontierCentroid.y)
            print("Going to Closest Centroid")
        else:
            bestCentroid = FrontierLocator.gridToWorld(mapdata, allCentroids[0].x, allCentroids[0].y)
            for centroid in allCentroids:
                if mapdata.data[self.gridToIndex(mapdata,centroid.x, centroid.y)] < 60:
                    bestCentroid = FrontierLocator.gridToWorld(mapdata, centroid.x, centroid.y)


        return bestCentroid

    def selectCentroid(self,frontier):        
        counter = 0

        sumx = sumy = 0

        for cell in frontier:
            sumx += cell[0]
            sumy += cell[1]
            counter +=1

        point = Point()
        point.x = sumx/counter
        point.y = sumy/counter

        return point

    @staticmethod
    def euclideanDistance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """

        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
    def startChain(self, mapdata, cellGrid, chain, initalCell):

        #Returns list of all cells that are neighbors
        cell = initalCell
        chain.append(cell)
        cellGrid.remove(cell)
        building = True
        cellCheck = []
        
        while building:
            
            neighborCells =  FrontierLocator.neighborsOf8(mapdata, cell[0], cell[1], -1)

            building = False 
            

            for n in neighborCells:
                if n in cellGrid:
                    chain.append(n)
                    cellGrid.remove(n)
                    cellCheck.append(n)
                    
                    building = True

            if len(cellCheck) > 0:    
                cell = cellCheck[0]
                cellCheck.remove(cell)
                building = True
            

        return chain

    def removeDuplicates(self, list1, list2):
        for cell1 in list1:
            if cell1 in list2:
                list1.remove(cell1)
        return list1

    @staticmethod
    def indexToGrid(mapdata, index):
        """
        Returns the x and y  corresponding to the given index in the occupancy grid.
        :param index [int] The cell index.
        :return  [int,int] X and Y coordinates.
        """

        x = index % mapdata.info.width
        y = index // mapdata.info.width
        return (x, y)

    @staticmethod
    def gridToIndex(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """

        return y*mapdata.info.width+x

    @staticmethod
    def worldToGrid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """

        resolution = mapdata.info.resolution
        ox = mapdata.info.origin.position.x
        oy = mapdata.info.origin.position.y

        gcx = int((wp.x - ox) / resolution)
        gcy = int((wp.y - oy) / resolution)

        return (gcx, gcy) 

    @staticmethod
    def gridToWorld(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """

        wc = Point()

        resolution = mapdata.info.resolution
        ox = mapdata.info.origin.position.x
        oy = mapdata.info.origin.position.y

        wc.x = (x + 0.5) * resolution + ox
        wc.y = (y + 0.5) * resolution + oy

        return wc

    @staticmethod
    def neighborsOf8(mapdata, x, y, minimumThreashold):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """

        pointList = [((x-1), y), ((x+1), y), (x, (y-1)), (x, (y+1)),
                     ((x-1), (y+1)), ((x-1), (y-1)), ((x+1), (y+1)), ((x+1), (y-1))]
        walkableList = []
        for point in pointList:
            if FrontierLocator.isCellWalkable(mapdata, point[0], point[1], minimumThreashold):
                walkableList.append(point)

        return walkableList

    @staticmethod
    def isCellWalkable(mapdata, x, y, minimumThreashold):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """

        # Check for boundries:
        x_boundary = mapdata.info.width
        y_boundary = mapdata.info.height
        gridIndex = FrontierLocator.gridToIndex(mapdata, x, y)

        walkable = False

        if x >= 0 and x < x_boundary:
            if y >= 0 and y < y_boundary:
                # Taking 60 as threshold probability that an obstacle might be present
                if mapdata.data[gridIndex] >= minimumThreashold and mapdata.data[gridIndex] < 60:
                    walkable = True

        return walkable

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

if __name__ == '__main__':
    FrontierLocator().run()