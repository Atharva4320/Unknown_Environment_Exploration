#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped
import numpy as np


class PathPlanner:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner", anonymous=True)
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        rospy.Service("path_plan", GetPlan, self.plan_path)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.cspace = rospy.Publisher(
            "/path_planner/cspace", GridCells, queue_size=5)
        ## Create publishers for A* (expanded cells, frontier, ...)
        self.expandCells = rospy.Publisher("/path_planner/expanded_cells", GridCells, queue_size=5)
        ## Choose a the topic names, the message type is GridCells
        self.a_star_pub = rospy.Publisher(
            "/path_planner/aStar", Path, queue_size=5)
        ## Subscribing to gmapping
        self.frontCells = rospy.Subscriber(
            "/nav_msgs/OccupancyGrid", GridCells)
        ## Initialize the request counter
        self.getMap = rospy.Subscriber("/cspace_map", OccupancyGrid,self.updateMap)
        ## Initialize the request counter for the "grayspace" that is unoccupied but we would prefer to avoid
        self.getGray = rospy.Subscriber("/gray_map", OccupancyGrid,self.updateGrayMap)
        self.getGray2 = rospy.Subscriber("/gray_map2", OccupancyGrid,self.updateGrayMap2)


        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

    def updateMap(self,msg):
        self.map=msg
    def updateGrayMap(self,msg):
        self.grayMap=msg
    def updateGrayMap2(self,msg):
        self.grayMap2=msg

    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """

        return y*mapdata.info.width+x

    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """

        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    @staticmethod
    def grid_to_world(mapdata, x, y):
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
    def world_to_grid(mapdata, wp):
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
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """

        poseStampedList = []

        for data in path:
            poseMessage = PoseStamped()
            world = PathPlanner.grid_to_world(mapdata, data[0], data[1])
            poseMessage.pose.position = world
            poseStampedList.append(poseMessage)

        return poseStampedList

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
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
        gridIndex = PathPlanner.grid_to_index(mapdata, x, y)

        walkable = False

        if x >= 0 and x < x_boundary:
            if y >= 0 and y < y_boundary:
                # Taking 60 as threshold probability that an obstacle might be present
                if mapdata.data[gridIndex] < 60:
                    walkable = True

        return walkable

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """

        pointList = [((x-1), y), ((x+1), y), (x, (y-1)), (x, (y+1))]
        walkableList = []
        for point in pointList:
            if PathPlanner.is_cell_walkable(mapdata, point[0], point[1]):
                walkableList.append(point)

        return walkableList

    @staticmethod
    def neighbors_of_8(mapdata, x, y):
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
            if PathPlanner.is_cell_walkable(mapdata, point[0], point[1]):
                walkableList.append(point)

        return walkableList

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """

        rospy.loginfo("Requesting the map")
        # Connect to the map server - this is done here because the method is static
        rospy.wait_for_service('/static_map')
        ##Initialize service client that can be called to provide the map
        map_srv = rospy.ServiceProxy('/static_map', GetMap)
        #Get the map from map_srv
        grid = map_srv().map
        return grid

    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """

        rospy.loginfo("Calculating C-Space")
        gridData = mapdata.data
        updatedList = [0] * len(gridData)
        pointsList = []
        ## Go through each cell in the occupancy grid
        for i in range(len(gridData)):
            ## Inflate the obstacles where necessary
            if gridData[i] >= 60:
                coordinates = self.index_to_grid(mapdata, i)
                aroundCells = self.neighbors_of_8(
                    mapdata, coordinates[0], coordinates[1])
                updatedList[i] = gridData[i]

                for j in aroundCells:

                    indJ = self.grid_to_index(mapdata, j[0], j[1])
                    if indJ < len(gridData) and gridData[indJ] < 60:
                        coord = self.index_to_grid(mapdata, indJ)
                        wrldCoord = self.grid_to_world(
                            mapdata, coord[0], coord[1])
                        pointsList.append(wrldCoord)
                        updatedList[indJ] = gridData[i]
            else:
                if updatedList[i] == 0:
                    updatedList[i] = gridData[i]

        cSpace = mapdata
        cSpace.data = updatedList

        message = GridCells()

        message.header = mapdata.header
        message.cell_width = mapdata.info.resolution
        message.cell_height = mapdata.info.resolution
        message.cells = pointsList
        self.cspace.publish(message)

        return cSpace

    @staticmethod
    def index_to_grid(mapdata, index):
        """
        Returns the x and y  corresponding to the given index in the occupancy grid.
        :param index [int] The cell index.
        :return  [int,int] X and Y coordinates.
        """

        x = index % mapdata.info.width
        y = index // mapdata.info.width
        return (x, y)

    def a_star(self, mapdata, start, goal):
        """
        Using A* algorithm to calculate an path.
        :param mapdata  [OccupancyGrid] The map data.
        :param start    [(int, int)]    The initial point (a grid coordinate).
        :param goal     [(int, int)]    The goal point (a grid coordinate).
        :return         [[(int, int)]]  The path as a list of tuples (grid coordinates)
        """

        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" %
                      (start[0], start[1], goal[0], goal[1]))
        startPath = PathCalc()
        startPath.coord = start

        openList = [startPath]
        closedList = []
        dispList = []

        while(len(openList) > 0):
            #Remove smallest element
            openList = sorted(openList, key=getFn)
            q = openList[0]
            openList = openList[1:]

            # Get neighbors and append them as paths in Q:
            neighbors = self.neighbors_of_8(
                mapdata, q.coord[0], q.coord[1])  # List of neighbor tuples

            for node in neighbors:
                if(node == goal):
                    pathList = [q.coord, node]

                    tempNode = q.parent
                    while(tempNode.coord != start):
                        pathList.insert(0, tempNode.coord)
                        tempNode = tempNode.parent

                    return pathList
                #Calculate G
                if ((node == (q.coord[0]-1, q.coord[1])) or (node == (q.coord[0]+1, q.coord[1])) or (node == (q.coord[0], q.coord[1]-1)) or (node == (q.coord[0], q.coord[1]+1))):
                    g = 1
                if ((node == (q.coord[0]-1, q.coord[1]+1)) or (node == (q.coord[0]-1, q.coord[1]-1)) or (node == (q.coord[0]+1, q.coord[1]+1)) or (node == (q.coord[0]+1, q.coord[1]-1))):
                    g = math.sqrt(2)

                #Double cost if in the "grayspace" near a wall
                if(not PathPlanner.is_cell_walkable(self.grayMap,node[0],node[1])):
                    g = 4*g
                elif(not PathPlanner.is_cell_walkable(self.grayMap2,node[0],node[1])):
                    g = 2*g

                h = self.euclidean_distance(node[0], node[1], goal[0], goal[1])
                
                # Angle heuristic actively makes things worse

                f = g + h

                addNode = True
                for otherNode in openList:
                    if(otherNode.coord == node and otherNode.f <= f):
                        addNode = False  # Better path to position
                for otherNode in closedList:
                    if(otherNode.coord == node and otherNode.f <= f):
                        addNode = False  # We've already been here

                if(addNode):
                    newNode = PathCalc()
                    newNode.coord = node
                    newNode.parent = q
                    newNode.f = f
                    openList.append(newNode)

                    item_coord = newNode.coord
                    worldCoord = self.grid_to_world(mapdata,item_coord[0],item_coord[1])
             
                    dispList.append(worldCoord)
                    
            closedList.append(q)

            ### Publishes the cells as a* expands
            # ERROR: The robot moves really slow while it publishes it
            msg = GridCells()
            msg.header = mapdata.header
            msg.cell_height = mapdata.info.resolution
            msg.cell_width = mapdata.info.resolution
            msg.cells = dispList
            self.expandCells.publish(msg)
            # Uncomment to view A* step by step
            # rospy.sleep(.25)
            

        return "ERROR"

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")

        optimizedPath = [path[0]]

        i = 1
        while(i < len(path)-1):
            prev = path[i-1]
            node = path[i]
            nextNode = path[i+1]

            angle = PathPlanner.getAngle(prev,node,nextNode)

            #If the robot doesn't turn, the point is not necessary
            if(abs(angle) > .05):
                optimizedPath.append(node)
            else:
                pass
            i = i+1

        optimizedPath.append(path[len(path)-1])

        return optimizedPath

    @staticmethod
    def getAngle(prev,node,nextNode):

        thetaprev = math.atan2(node[1]-prev[1], node[0]-prev[0])
        thetanext = math.atan2(nextNode[1]-node[1], nextNode[0]-node[0])

        return thetanext-thetaprev


    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """

        path_message = Path()
        path_message.poses = PathPlanner.path_to_poses(mapdata, path)

        path_message.header.frame_id = "map"

        rospy.loginfo("Returning a Path message")
        return path_message

    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """

        ## Request the map
        ## In case of error, return an empty path
        mapdata = self.map

        if mapdata is None:
            return Path()
       
        ## Execute A*

        print("Start A*")
        print(msg.start.pose.position,msg.goal.pose.position)

        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path = self.a_star(mapdata, start, goal)

        print("Did A*")
        print(path)
        print(mapdata.header)

        ## Optimize waypoints - PURE PURSUIT BENEFITS FROM ADDITIONAL POINTS
        waypoints = path

        ## Return a Path message
        path_msg = self.path_to_message(mapdata, waypoints)
        self.a_star_pub.publish(path_msg)
        return path_msg
    

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


class PathCalc:
	def __init__(self):

            self.parent = None
            self.coord = (0, 0)
            self.f = 0.0


def getFn(path):
	return path.f


if __name__ == '__main__':
    PathPlanner().run()
