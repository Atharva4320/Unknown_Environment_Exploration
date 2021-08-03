#!/usr/bin/env python2

import rospy
import math

from nav_msgs.msg import Odometry, Path, OccupancyGrid,GridCells
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Quaternion
from tf.transformations import euler_from_quaternion
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from operator import itemgetter

#TODO
# - Divide up frontiers
# - A* penalize turns - DONE
# - Fix driving
#    - Turn smarter
# - Arc drive?

class Lab3Drive:

    def __init__(self):
        """
        Class constructor
        """
        #Initialize pose variables
        self.px = 0.0 #x-position
        self.py = 0.0 #y-position
        self.pz = 0.0 # angle

        self.isMoving = False
        self.initStage = True
        self.backOff = False
        self.state = 1
        self.end_goal = PoseStamped()



        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node("lab3drive")
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.cspace = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=5)
        self.pubMap = rospy.Publisher('/cspace_map', OccupancyGrid, queue_size=5)
        self.pubGraySpace = rospy.Publisher('/gray_map', OccupancyGrid, queue_size=5)
        self.pubGraySpace2 = rospy.Publisher('/gray_map2', OccupancyGrid, queue_size=5)
        self.pubPoint = rospy.Publisher('/centroid', PointStamped, queue_size=5)
        self.frontier = rospy.Publisher('/frontier', GridCells,queue_size=5)

        self.pubCurr = rospy.Publisher('/pursuit_curr', PointStamped, queue_size=5)
        self.pubTarg = rospy.Publisher('/pursuit_targ', PointStamped, queue_size=5)

        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry,self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        rospy.Subscriber('/move_base_simple/goal', PoseStamped,self.path_to)

        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        rospy.Subscriber('/map', OccupancyGrid,self.update_map)

        # Connect to the pathing server - this is done here because the method is static
        rospy.wait_for_service('/path_plan')
        ##Initialize service client that can be called to provide the map
        self.path_srv = rospy.ServiceProxy('/path_plan', GetPlan)
        print("A* Server running")

        # Connect to the pathing server - this is done here because the method is static
        rospy.wait_for_service('/frontier_point')
        ##Initialize service client that can be called to provide the map
        self.frontier_srv = rospy.ServiceProxy('/frontier_point', GetPlan)
        print("Frontier Server running")

        ### Tell ROS that this node subscribes to LaserScan  messages on the '/scan' topic
        ### When a message is received, call self.updateScan
        rospy.Subscriber('/scan', LaserScan,self.updateScan)

        rospy.sleep(2)
        self.make_map()

    def updateScan(self,msg):
        self.lasScan = msg

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
 
        ### Make a new Twist message
        msg_cmd_vel = Twist()
        ### Publish the message
        #Linear velocity
        msg_cmd_vel.linear.x =linear_speed
        msg_cmd_vel.linear.y=0.0
        msg_cmd_vel.linear.z=0.0 
        #Angular velocity
        msg_cmd_vel.angular.x =0.0
        msg_cmd_vel.angular.y=0.0
        msg_cmd_vel.angular.z=angular_speed
        #Send command

        self.pub.publish(msg_cmd_vel)

    
        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """

        #Save position
        initPoseX = self.px
        initPoseY = self.py

        correction = -.012

        #Start driving
        self.send_speed(linear_speed,correction)

        #Threshold to stop driving, in meters
        THRESHOLD = .005

        #Plan acceleration, if drive distance is low, accelerate to top speed then slow down. If it is a longer distance, accerate over 1/2 meter
        short_drive = distance<1
        
        if short_drive:
            factor = math.pi/distance
        else:
            factor = math.pi/1

        while (True):
            distToGo = math.sqrt((self.px-initPoseX) ** 2 + (self.py-initPoseY) ** 2)
        
            distTraveled = distance-distToGo

            
            #Accelerate in first half, slow down in second. If more than .5 m distance, only accelerate for the first and last half meter
            if(short_drive):
                if(distTraveled<distance/2):
                    speed = (0.8*linear_speed*math.sin(factor*distTraveled))+.2*linear_speed
                else:
                    speed = (0.8*linear_speed*math.sin(factor*distToGo))+.2*linear_speed
            else:
                #accelerate
                if(distTraveled<.5):
                    speed = (0.8*linear_speed*math.sin(factor*distTraveled))+.2*linear_speed
                #decelerate
                elif(distToGo<.5):
                    speed = (0.8*linear_speed*math.sin(factor*distToGo))+.2*linear_speed
                #Stay at full speed
                else:
                    speed = linear_speed

            #Send speed, and reverse if overshooting target
            if(distToGo >= distance+THRESHOLD):
                self.send_speed(-speed/2,correction)
            elif(distToGo <= distance-THRESHOLD):
                self.send_speed(speed/2,correction)
            elif(distToGo >= distance-THRESHOLD):
                self.send_speed(0.0,0.0)
                break

            rospy.sleep(.05)


    def brake(self,  linear_speed):
        """
        Stops the robot over 0.25 seconds
        :param linear_speed [float] [m/s] The forward linear speed at the start of the motion
        """
        print("stopping")
        
        STOP_TIME = 0.5

        #Start slowing down
        iteration = 25
        while (iteration>=0):
            self.send_speed(linear_speed*(iteration/25),0)

            rospy.sleep(STOP_TIME/25)

            iteration -=1


    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        
        #Save position
        initPoseT = self.pt
        target = initPoseT+angle

        #Start driving
        self.send_speed(0.0,aspeed)

        #Threshold to stop turning, in radians
        THRESHOLD = .0035
        #Factor used in sin based acceleration
        factor = math.pi/angle

        while (True):
            #Calculate how far is left to turn, and how much distance has already been covered
            distToGo = target - self.pt
            distTraveled = self.pt-initPoseT

            #Accelerate in first half, slow down in second, minimum speed is 1/4 target speed
            if(abs(distTraveled)<abs(angle/2)):
                speed = (0.7*aspeed*math.sin(factor*distTraveled))+.3*aspeed
            else:
                speed = (0.7*aspeed*math.sin(factor*distToGo))+.3*aspeed

            #Set turn speed, if the robot overshoots it reverses the turn direction
            if(distToGo < -THRESHOLD):
                self.send_speed(0.0,-speed)
            elif(distToGo > THRESHOLD): #Switch back to going forward if needed
                self.send_speed(0.0, speed)
            else:
                self.send_speed(0.0,0.0)
                break

            rospy.sleep(.05)

    #A high speed/low accuracy version of the turn function - optimal for use with pure pursuit
    def rotateQuick(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
       
        #Save position
        initPoseT = self.pt
        target = initPoseT+angle

        #Threshold to stop turning, in radians
        # In this case, about 15 degrees
        THRESHOLD = .26
        #Factor used in sin based acceleration
        factor = abs(math.pi/(angle*.25))

        while (True):
            #Calculate how far is left to turn, and how much distance has already been covered
            distToGo = target - self.pt
            distTraveled = self.pt-initPoseT

            while(distToGo>math.pi):
                distToGo -= 2*math.pi
                
            while(distToGo < -math.pi):
                distToGo += 2*math.pi
                

            #Accelerate in first quarter, slow down in last, minimum speed is 1/4 target speed
            if(abs(distTraveled)<abs(angle/4)):
                speed = (0.7*aspeed*abs(math.sin(factor*abs(distTraveled))))+.3*aspeed
                
            elif(abs(distTraveled)>abs(3*angle/4)):
                speed = (0.7*aspeed*abs(math.sin(factor*abs(distToGo))))+.3*aspeed
                
            else:
                speed = aspeed

            #Set turn speed, if the robot overshoots it reverses the turn direction
            if(distToGo < -THRESHOLD):
                self.send_speed(0.0,-speed)
            elif(distToGo > THRESHOLD): #Switch back to going forward if needed
                self.send_speed(0.0, speed)
            else:
                print("Complete")
                self.send_speed(0.0,0.0)
                break
            

            rospy.sleep(.05)



    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """

        newPose = msg.pose

        #Calculate x and y distances to travel from current position to target
        x=newPose.position.x-self.px
        y=newPose.position.y-self.py
        
        #Get usable yaw angle from the quaternion
        quat_orig=newPose.orientation
        quat_list=[quat_orig.x,quat_orig.y,quat_orig.z,quat_orig.w]
        (roll,pitch,yaw)=euler_from_quaternion(quat_list)
        theta2=yaw

        #Calculate angle to turn to point at the target x,y position
        theta1 = math.atan2(y,x)
        #Calculate the distance to travel
        dist = math.sqrt(x ** 2+y ** 2)

        #Turn to face target
        self.rotate(theta1-self.pt,.15)
        #Drive distance
        self.drive(dist,.2)
        #Rotate to face final direction - Currently disabled because our path does not provide meaninful final direction
        #self.rotate(theta2-theta1,.15)

    def path_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        ### REQUIRED CREDIT
        start = PoseStamped()
        start.header = msg.header
        start.pose.position.x = self.px
        start.pose.position.y = self.py
        start.pose.orientation = self.quat

        goal = msg
        #Have the path calculated
        path = self.path_srv(start,goal,.001)

        poseList = path.plan.poses
        for pose in poseList:
            print("Moving to pose: ")
            print(pose.pose)
            self.go_to(pose)

        print("Motion Complete")

    def purePursuit(self, msg):
        print("Begin Pure Pursuit")
        LOOKAHEAD_DISTANCE = 3 #Number of points to look ahead - more is more steady and less precise
        LIN_VEL = 0.18

        trajectoryPoses = msg.plan.poses #list of poseStamped

        #Simplify into a list of x,y coordinates
        trajectory = []

        for stampedPose in trajectoryPoses:
            trajectory.append((stampedPose.pose.position.x,stampedPose.pose.position.y))

        #Aim at 2nd pose, otherwise can act weird
        x=trajectory[1][0]-self.px
        y=trajectory[1][1]-self.py

        print(y,x)
        #Calculate angle to turn to point at the target x,y position
        theta1 = math.atan2(y,x)

        #Turn to face target if angle greater than ~45 degrees
        print("THETA",theta1)

        if(abs(theta1-self.pt)>0.8):
            print("Rotate")
            self.rotateQuick(theta1-self.pt,1)

        while(True):
            #Check scanner to avoid wall collisions
            dist = self.lasScan.ranges[0]

            #Cancel motion, back off, and try again
            if(dist != "inf" and dist<.25):
                self.brake
                while(dist<.3):
                    dist = self.lasScan.ranges[0]
                    print("Backing off")
                    self.send_speed(-0.1,0)
                    rospy.sleep(0.01)

                self.backOff = True
                self.send_speed(0,0)   
                return

            #Find closest point
            #------------------
            target = -1
            dist = 10000
            myPoint = (0,0)
            for point in trajectory:
                pointDist = math.sqrt((point[0]-self.px)**2 + (point[1]-self.py)**2)
                if(pointDist<dist):
                    dist = pointDist
                    target = trajectory.index(point)
                    myPoint = point
            if(target+LOOKAHEAD_DISTANCE >= len(trajectory)):
                #Target reached - we stop early because the frontier itself could be unknown
                self.brake(LIN_VEL)
                #Give time for map to update
                rospy.sleep(1)
                break

            currPointMessage = PointStamped()
            currPointMessage.header.frame_id = "map"
            currPointMessage.point.x = myPoint[0]
            currPointMessage.point.y = myPoint[1]
            self.pubCurr.publish(currPointMessage)

            #Find goal point
            #------------------
            goalPoint = trajectory[target+LOOKAHEAD_DISTANCE]

            targPointMessage = PointStamped()
            targPointMessage.header.frame_id = "map"
            targPointMessage.point.x = goalPoint[0]
            targPointMessage.point.y = goalPoint[1]
            self.pubTarg.publish(targPointMessage)

            #Calculate curvature
            #------------------
            dx = goalPoint[0]-self.px
            dy = goalPoint[1]-self.py

            #Convert from world coordinates to distance(x) and offset(y) from robot
            distanceToPoint = math.cos(self.pt)*dx + math.sin(self.pt)*dy
            offsetToPoint = -math.sin(self.pt)*dx + math.cos(self.pt)*dy

            angle = math.atan2(offsetToPoint,distanceToPoint)
        
            w = angle*(LIN_VEL/distanceToPoint)
            #Send motion
            twist = Twist()
            twist.linear.x = LIN_VEL
            twist.angular.z = w*3.0
            self.pub.publish(twist)

            rospy.sleep(.01)


    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """

        self.px = msg.pose.pose.position.x
        self.py=msg.pose.pose.position.y
        
        #Convert to more readable euler angles from quaternions
        quat_orig=msg.pose.pose.orientation
        quat_list=[quat_orig.x,quat_orig.y,quat_orig.z,quat_orig.w]
        (roll,pitch,yaw)=euler_from_quaternion(quat_list)
        self.pt=yaw
        self.quat = quat_orig

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
            # TODO
            if gridData[i] >= 60:
                #updatedList.append(cell)
                coordinates = self.indexToGrid(mapdata, i)
                aroundCells = self.neighborsOf8(
                    mapdata, coordinates[0], coordinates[1], 0)
                updatedList[i] = gridData[i]

                for j in aroundCells:

                    indJ = self.grid_to_index(mapdata, j[0], j[1])
                    #data = self.grid_to_index(mapdata,j[0],j[1])
                    if indJ < len(gridData) and gridData[indJ] < 60:
                        coord = self.indexToGrid(mapdata, indJ)
                        wrldCoord = self.gridToWorld(
                            mapdata, coord[0], coord[1])
                        pointsList.append(wrldCoord)
                        updatedList[indJ] = gridData[i]
            else:
                # for items in range(len(updatedList)):
                if updatedList[i] == 0:
                    updatedList[i] = gridData[i]

        cSpace = OccupancyGrid()#mapdata
        cSpace.info = mapdata.info
        cSpace.header=mapdata.header
        cSpace.data = updatedList

        message = GridCells()

        message.header = mapdata.header
        message.cell_width = mapdata.info.resolution
        message.cell_height = mapdata.info.resolution
        message.cells = pointsList

        return cSpace

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
            if Lab3Drive.is_cell_walkable(mapdata, point[0], point[1], minimumThreashold):
                walkableList.append(point)

        return walkableList

    @staticmethod
    def is_cell_walkable(mapdata, x, y, minimumThreashold):
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
        gridIndex = Lab3Drive.grid_to_index(mapdata, x, y)

        walkable = False

        if x >= 0 and x < x_boundary:
            if y >= 0 and y < y_boundary:
                # Taking 65 as threshold probability that an obstacle might be present
                if mapdata.data[gridIndex] >= minimumThreashold and mapdata.data[gridIndex] < 60:
                    walkable = True

        return walkable

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

    def update_map(self,message):
        self.map = message

    def make_map(self):
        """
        Drive to locations needed to develop the map
        """
        message = self.map
        
        if (self.initStage): 
            # Getting the initial position of the robot for Phase 2
            self.end_goal.pose.position.x = self.px
            self.end_goal.pose.position.y = self.py
            self.end_goal.pose.orientation = self.quat
            self.initStage = False
        print("Make Map")

        if(self.state == 1 and not self.isMoving): # Execute Phase 1 
            try:
                self.isMoving = True
                #Calc cspace for a number of dilations - cspace 2 is the "true" cspace, while a larger one is used for the "grayspace"
                cspace1 = self.calc_cspace(message,1)
                cspace2 = self.calc_cspace(cspace1,1)
                cspace3 = self.calc_cspace(cspace2,1)
                cspace4 = self.calc_cspace(cspace3,1)
                cspace5 = self.calc_cspace(cspace4,1)

                print("Publish")
                self.pubMap.publish(cspace2)
                #Todo: combine cleaner
                self.pubGraySpace.publish(cspace3)
                self.pubGraySpace2.publish(cspace4)

                #Choose target postion on frontier
                startPoint = PoseStamped()
                endPoint = PoseStamped()

                startPoint.header.frame_id = "map"
                endPoint.header.frame_id = "map"

                startPoint.pose.position.x = self.px
                startPoint.pose.position.y = self.py

                resp = self.frontier_srv(startPoint,endPoint,0.001)

                centroid = resp.plan.poses[0].pose.position

                stampPoint = PointStamped()
                stampPoint.header.frame_id = "map"
                stampPoint.point = centroid
                self.pubPoint.publish(stampPoint)

                #Navigate to target
                start = PoseStamped()
                start.header.frame_id = "map"
                start.pose.position.x = self.px
                start.pose.position.y = self.py
                start.pose.orientation = self.quat

                goal = PoseStamped()
                goal.pose.position = centroid


                #Have the path calculated
                rospy.sleep(1)
                try:
                    path = self.path_srv(start,goal,.001)
                    self.purePursuit(path)
                except:
                    # If the robot is in cspace:
                    print("Path in cspace")
                    initPoint = Point() # Record the current pose of the robot
                    initPoint.x = self.px 
                    initPoint.y = self.py
                    initGrid = self.world_to_grid(cspace2,initPoint) # Converting world points to grid coordinates
                    initgridInd = self.grid_to_index(cspace2,initGrid[0],initGrid[1]) # Calculating the index of the cell
                    gridData = cspace2.data # Extracting data from map

                    # Executed till the robot gets out of the cspace: 
                    while (gridData[initgridInd]) > 60:
                        self.send_speed(-0.15,0.0)
                        updatePoint = Point() # Updating the robot's pose
                        updatePoint.x = self.px
                        updatePoint.y = self.py
                        updateGrid = self.world_to_grid(cspace2,updatePoint)
                        initgridInd = self.grid_to_index(cspace2,updateGrid[0],updateGrid[1])
                    self.send_speed(0.0,0.0)
                    self.make_map()

                print("Motion complete")
                rospy.sleep(1)
                self.isMoving = False
                # self.backOff = False
                self.make_map()

            except:
                self.state = 2
        
        if(self.state == 2): # Execute Phase 2
            # Getting the current pose of the robot:
             
            start2 = PoseStamped()
            start2.header.frame_id = "map"
            start2.pose.position.x = self.px
            start2.pose.position.y = self.py
            start2.pose.orientation = self.quat

            rospy.sleep(1)

            # Calculating the path from the current posiiton to the starting position of the robot
            path2 = self.path_srv(start2,self.end_goal,.001)

            poseList2 = path2.plan.poses
       
            self.purePursuit(path2)
            if (self.backOff):
                self.backOff = False
                self.make_map()
            print("Motion complete")
            rospy.sleep(1)
            self.state = 3

        if (self.state == 3): # Execute Phase 3
            print("This is where we execute the phase 3") 
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    Lab3Drive().run()
