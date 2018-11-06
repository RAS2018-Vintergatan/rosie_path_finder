#from feature import *
import rospy
from std_msgs.msg import String, Float32, Header

from tf.msg import transform_broadcaster
from geometry_msgs.msg import Point, Twist, PoseStamped, Pose
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from visulalization_msgs.msg import MarkerArray, Marker


import sys, random, math, pygame, time, threading, datetime

from math import sqrt ,cos ,sin ,atan2 ,exp
from array import *
from lineIntersect import *
#from Scheduler import *

XDIM = 480
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 50.0
NUMNODES = 2000
RADIUS = 50
j = 0
alpha = 0.6
beta = 0.4
distance = 0
resolution = 0.01
# f=0
# t=0
finalpathx = array('d', [])
finalpathy = array('d', [])
OBS= []
posBat = []
posObj = []
startx = 0
starty = 0

xpoint = 0
ypoint = 0

isGrabbed = 0

MarkerArray objects = []
MarkerArray batteries = []
Odometry pose

def angle(p1, p2):
    return atan2(p2[1] - p1[1], p2[0] - p1[0])

def dist(p1, p2):
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

def step_from_to(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return int(p1[0] + EPSILON * cos(theta)), int(p1[1] + EPSILON * sin(theta))

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def checkIntersect(nodeA,nodeB,OBS):
    A=(nodeA.x,nodeA.y)
    B=(nodeB.x,nodeB.y)
    for o in OBS:
      o = o*100
      dist = sqrt((o[2]-o[0])^2+(o[3]-o[1])^2)
      slope1 = (o[2]-o[0])/dist;
      slope2 = (o[3]-o[0])/dist;
      safety = 0.2/resolution;

      #expand wall in direction of the line
      if m1 >= 0:
          x1 = o[0] - slope1*safety
          y1 = o[1] - slope2*safety
          x2 = o[2] + slope1*safety
          y2 = o[3] + slope2*safety
      else:
          x1 = o[0] + slope1*safety
          y1 = o[1] + slope2*safety
          x2 = o[2] - slope1*safety
          y2 = o[3] - slope2*safety

      #expand wall in width
      slope1 = 1-slope1;
      slope2 = 1-slope2;
      P1 = (x1-slope1*safety y1+slope2*safety)
      P2 = (x1+slope1*safety y1-slope2*safety)
      P3 = (x2-slope1*safety y2+slope2*safety)
      P4 = (x2+slope1*safety y2-slope2*safety)

      C1=P1
      D1=P4
      C2=P1
      D2=P2
      C3=P3
      D3=P2
      C4=P3
      D4=P4
      inst1= ccw(A,C1,D1) != ccw(B,C1,D1) and ccw(A,B,C1) != ccw(A,B,D1)
      inst2= ccw(A,C2,D2) != ccw(B,C2,D2) and ccw(A,B,C2) != ccw(A,B,D2)
      inst3= ccw(A,C3,D3) != ccw(B,C3,D3) and ccw(A,B,C3) != ccw(A,B,D3)
      inst4= ccw(A,C4,D4) != ccw(B,C4,D4) and ccw(A,B,C4) != ccw(A,B,D4)
      if inst1==False and inst2==False and inst3==False and inst4==False:
        #print(A,B)
        #input("Press Enter to continue...")
        continue
      else:
         return False
    return True

def chooseParent(nn, newnode, nodes):
    for p in nodes:
        if checkIntersect(p, newnode, OBS) and dist([p.x, p.y], [newnode.x, newnode.y]) < RADIUS and p.cost + dist(
                [p.x, p.y], [newnode.x, newnode.y]) < nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y]):
            nn = p
    newnode.cost = nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y])
    newnode.parent = nn
    return newnode, nn


def reWire(nodes, newnode, pygame, screen):
    white = 255, 240, 200
    black = 20, 20, 40
    for i in range(len(nodes)):
        p = nodes[i]
        if checkIntersect(p, newnode, OBS) and p != newnode.parent and dist([p.x, p.y], [newnode.x, newnode.y]) < RADIUS and newnode.cost + dist([p.x, p.y], [newnode.x, newnode.y]) < p.cost:
            p.parent = newnode
            p.cost = newnode.cost + dist([p.x, p.y], [newnode.x, newnode.y])
            nodes[i] = p

    return nodes


class Cost:
    x = 0
    y = 0
    cost = 0
    parent = None

    def __init__(self, xcoord, ycoord):
        self.x = xcoord
        self.y = ycoord


class Node:
    x = 0
    y = 0
    cost = 0
    parent = None

    def __init__(self, xcoord, ycoord):
        self.x = xcoord
        self.y = ycoord

def trySolutionPath(start, goal, nodes, pygame, screen):
    path = [[goal.x, goal.y]]
    while nodes[goalind].parent is not None:
        node = nodes[goalind]
        path.append([node.x, node.y])
        goalind = node.parent
    path.append([start.x, start.y])
    return path

def objListCallback(data):
		int posx[] = { 0.10, 0.50, 1.00, 4.00, 0.35, 0.80, 2.50, 3.00, 2.00, 0.10 }
		int posy[] = { 0.80, 2.00, 1.00, 0.80, 3.00, 4.00, 4.50, 2.50, 0.10, 4.00} #assume already sorted markers
		int orient[] = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 120.0}
		for i in range(10):
			visualization_msgs::Marker testObj
			testObj.header.seq = 0
			testObj.header.stamp = ros::Time::now()
			testObj.header.frame_id = 0
			testObj.ns = "objects"
			testObj.id = i
			testObj.type = "some"
			testObj.action = 0 #or 2 for deleting an object
			testObj.pose.position.x = posx[i]*100
			testObj.pose.position.y = posy[i]*100
			testObj.pose.position.z = 0.0
			testObj.pose.orientation.x = 0.0
			testObj.pose.orientation.y = 0.0
			testObj.pose.orientation.z = orient[i]
			testObj.pose.orientation.w = 1
			testObj.scale.x = 0.05
			testObj.scale.y = 0.05
			testObj.scale.z = 0.05
			testObj.color.r = 1.0
			testObj.color.b = 0.0
			testObj.color.g = 0.0
			testObj.color.a = 1.0
			testObj.lifetime = 0
			testObj.frame_locked = false
			objects[i] = testObj[i]
        #skip some steps here
        for i in range(len(posx)):
            posObj.append((posx(i),posy(i)))

def batListCallback(data):
	int posx[] = { 0.80, 2.50}
	int posy[] = { 0.80, 2.00} #assume already sorted markers

	int orient[] = {10.0, 20.0}
	for i in range(5):
		visualization_msgs::Marker testObj
		testObj.header.seq = 0
		testObj.header.stamp = ros::Time::now()
		testObj.header.frame_id = 0
		testObj.ns = "battery"
		testObj.id = i
		testObj.type = "battery"
		testObj.action = 0; #or 2 for deleting an object
		testObj.pose.position.x = posx[i]*100
		testObj.pose.position.y = posy[i]*100
		testObj.pose.position.z = 0.0
		testObj.pose.orientation.x = 0.0
		testObj.pose.orientation.y = 0.0
		testObj.pose.orientation.z = orient[i]
		testObj.pose.orientation.w = 1
		testObj.scale.x = 0.05
		testObj.scale.y = 0.05
		testObj.scale.z = 0.05
		testObj.color.r = 1.0
		testObj.color.b = 0.0
		testObj.color.g = 0.0
		testObj.color.a = 1.0
		testObj.lifetime = 0
		testObj.frame_locked = false
		batteries[i] = testObj[i]
    #skip some steps here
    for i in range(len(posx)):
        rx = batteries[i].scale.x
        ry = batteries[i].scale.y
        posBat.append((posx(i),posy(i)))
        OBS.append((posx(i)-rx,posy(i)-ry,posx(i)+rx,posy(i)+ry))

def odomCallback(data):
    pose = data
    if startx == 0 and starty == 0:
        startx = pose.pose.pose.position.x
        starty = pose.pose.pose.position.y

    if pose.pose.pose.position.x == xpoint and pose.pose.pose.position.y == ypoint:
        complete = True
        self.grab_pub.publish(1)

def publishPath(pathx, pathy):
    Path path
    PoseStamped poses = []

    Time now = rospy.get_rostime()
    path.header.seq=0
    path.header.time = now
    path.header.frame_id = "path"
    seq = 0
    for i in range(len(pathx)):
        PoseStamped pose
        pose.header.seq= seq
        pose.header.time = now
        pose.header.frame_id ="path_pose"
        pose.pose.position.x = pathx(i)
        pose.pose.position.y = pathy(i)
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        if i < (len(pathx)-1):
            pose.pose.orientation.z = atan((pathy(i+1)-pathy(i))/(pathx(i+1)-pathx(i)))
        else if i == (len(pathx)-1):
            pose.pose.orientation.z = atan(pathy(i)-pathy(i-1))/(pathx(i)-pathx(i-1)))
        pose.pose.orientation.z = 1
        poses.append(pose)
        seq = seq+1
    path.poses = poses
    self.path_pub.publish(path)

    #publish path as grid
    OccupancyGrid pathGrid
    pathGrid.header.seq = seq
    pathGrid.header.stamp = now
    pathGrid.header.frame_id = "pathGrid"
    pathGrid.info.map_load_time = now
    pathGrid.info.resolution = resolution
    pathGrid.info.width = width
    pathGrid.info.height = height
    pathGrid.info.origin.position.x = 0
    pathGrid.info.origin.position.y = 0
    pathGrid.info.origin.position.z = 0
    pathGrid.info.origin.orientation.x = 0
    pathGrid.info.origin.orientation.y = 0
    pathGrid.info.origin.orientation.z = 0
    pathGrid.info.origin.orientation.w = 1
    for i in range(width*height):
        pathGrid.data[i] = 0 #eventually we have to use .append()
    for i in range(len(pathx)):
        pathGrid.data[pathy(i)*width+pathx(i)]=125
    this.grid_pub.publish(pathGrid)


def grabCallback(data):
    isGrabbed = data

def wallCallback(data):
    for d in range(len(data)):
        OBS.append((data(d),data(d+1),data(d+2),data(d+3)))
        d = d+4

def mainpfast():
    print(__file__ + " start!!")
    rospy.init_node('rosie_rrt', anonymous=True)

	grid_pub = rospy.Publisher('rosie_path_grid',OccupancyGrid, queue_size=10) # visualize object grid
	wall_sub = rospy.Subscriber('walls', Float32, wallCallback) # get obstacles
    objList_sub = rospy.Subscriber('visualization_marker', MarkerArray, objListCallback) # get objects / obstacles
    bat_sub = rospy.Subscriber('visualization_marker_battery', MarkerArray, batListCallback) # get bat obstacles
	odom_sub = rospy.Subscriber('odom', Odometry, odomCallback) # for check position and end of path
    grab_sub = rospy.Subscriber('grabState',Bool, grabCallback)
    grab_pub = rospy.Publisher('grabState',Bool, queue_size=10)
    global path_pub = rospy.Publisher('rosie_path', Path, queue_size=100)

    # xpoint, ypoint, complete
    # initialize and prepare screen

    mode = "goto" #others are "home" or "explore"
    if mode == "goto":
        #sorting eventually
        xpoint = posObj[0][0]/resolution
        ypoint = posObj[0][1]/resolution
    else if mode == "home":
        xpoint = 0.2/resolution
        ypoint = 0.2/resolution
    else if mode == "explore":
        for i in posObj:
            OBS.append(i)
        #extract nodes from map - Random generated points

    global startx, starty, distance, finalpathx, finalpathy
    start_time = rospy.get_rostime()#time.time()
    while(complete):

        nodes = []
        nodes.append(Node(startx, starty))  # Start in the corner
        start = nodes[0]
        goal = Node(xpoint, ypoint)
        slope = angle([start.x, start.y], [goal.x, goal.y])
        i = 1
        ctr_neargoal = 0
        ctr = 1
        while ctr:
            lower = slope - cons
            higher = slope + cons
            if (i % 20 == 0 or ctr_neargoal == 1):
                rand = Node(goal.x, goal.y)
                i = i + 1
            else:
                rand = Node(random.randint(0, XDIM), random.randint(0, YDIM))
                slope1 = angle([start.x, start.y], [rand.x, rand.y])
                while not(slope1 >= lower and slope1 <= higher):
                    rand = Node(random.randint(0, XDIM), random.randint(0, YDIM))
                    slope1 = angle([start.x, start.y], [rand.x, rand.y])
                i = i + 1
            if i > 300:     #no. of iterations have become way too high means angle is not enough
                cons = cons + 0.25
                i = i - 300  #so that i > 300 works again in case of failed angles
            nn = nodes[0]
            for p in nodes:
                if dist([p.x, p.y], [rand.x, rand.y]) < dist([nn.x, nn.y], [rand.x, rand.y]):
                    nn = p
            interpolatedNode = step_from_to([nn.x, nn.y], [rand.x, rand.y])
            newnode = Node(interpolatedNode[0], interpolatedNode[1])
            if checkIntersect(nn, rand, OBS):
                [newnode, nn] = chooseParent(nn, newnode, nodes)
                nodes.append(newnode)
                nodes = reWire(nodes, newnode, pygame, screen)

            if (dist([newnode.x, newnode.y], [goal.x, goal.y]) < 25.0):
                ctr_neargoal = 1
            if (dist([newnode.x, newnode.y], [goal.x, goal.y]) == 0):
                ctr = 0
        finalpathx = []
        finalpathy = []

        print("The time taken to plot the request is", time.time() - start_time)
        time.sleep(10)
        startx = xpoint
        starty = ypoint
        print("The distance traversed is", distance)

        publishPath(finalpathx, finalpathy)

        complete = 0

if __name__ == '__main__':

    mainpfast()
