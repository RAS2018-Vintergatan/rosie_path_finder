#from feature import *
import rospy
from std_msgs.msg import String, Float32, Header, Int8

from tf.msg import transform_broadcaster
from geometry_msgs.msg import Point, Twist, PoseStamped, Pose
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from visulalization_msgs.msg import MarkerArray, Marker


import sys, random, math, time, threading, datetime

from math import sqrt ,cos ,sin ,atan2 ,exp
from array import *
from lineIntersect import *
#from Scheduler import *

XDIM = 480
YDIM = 480
WINSIZE = [XDIM, YDIM]
HOME = [20 20]
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
OBS= []
obsList = []
batList = []
objList = []
startx = 0
starty = 0

xpoint = 0
ypoint = 0

isGrabbed = 0

MarkerArray objects = []
MarkerArray batteries = []
Odometry pose

lastMode = "home"

#def angle(p1, p2):
#    return atan2(p2[1] - p1[1], p2[0] - p1[0])

#def dist(p1, p2):
#    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

#def step_from_to(p1, p2):
#    if dist(p1, p2) < EPSILON:
#        return p2
#    else:
#        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
#        return int(p1[0] + EPSILON * cos(theta)), int(p1[1] + EPSILON * sin(theta))

#def ccw(A,B,C):
#    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

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


def reWire(nodes, newnode):
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

def trySolutionPath(start, goal, nodes):
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
            objList.append((posx(i),posy(i)))

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
#    for i in range(len(posx)):
#        rx = batteries[i].scale.x
#        ry = batteries[i].scale.y
#        batList.append((posx(i),posy(i)))
#        OBS.append((posx(i)-rx,posy(i)-ry,posx(i)+rx,posy(i)+ry))

def odomCallback(data):
    pose = data
    if startx == 0 and starty == 0:
        startx = pose.pose.pose.position.x
        starty = pose.pose.pose.position.y

def checkEndpoint(pose, state):

    if pose.pose.pose.position.x == xpoint and pose.pose.pose.position.y == ypoint:
        complete = True
        self.grab_pub.publish(state)


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

def wallCallback(data): # assumes message float32 with [widht, height, x1_1 y1_1, x2_1, y2_1, x1_2,.....]
    width = data(0)
    height = data(1)
    data = data(2:)
    for d in range(len(data)):
        OBS.append((data(d),data(d+1),data(d+2),data(d+3)))
        d = d+4

def calculatePath(startx, starty, OBS):
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
                nodes = reWire(nodes, newnode)

            if (dist([newnode.x, newnode.y], [goal.x, goal.y]) < 25.0):
                ctr_neargoal = 1
            if (dist([newnode.x, newnode.y], [goal.x, goal.y]) == 0):
                ctr = 0
        finalpathx = []
        finalpathy = []
        return finalpath = [finalpathx, finalpathy]

def generate_random_path(OBS):
    randPathX = [50, 380, 380, 450, 450, 450, 120, 120, 140, 120, 120, 50]
    randPathY = [400, 400, 200, 200, 400, 50, 50, 100, 100, 100, 50, 50]
    genPathX = []
    genPathY = []
    for i in range(len(randPathX)):
        [tempPathX, tempPathY] = calculatePath(randPathX[i], randPathY[i], OBS)
        for i in tempPathX:
            genPathX.append(i)
        for i in tempPathY:
            genPathY.append(i)
    publishPath(genPathX, genPathY)

def controlGate(state): #0 is close, 1 is open
    rospy.wait_for_service('control_gates')
    try:
        control_gates = rospy.ServiceProxy('control_gates', controlGates)
        change_gate = control_gates(state)
        return change_gate.result #if 1 change performed, if 0 error occured
    except rospy.ServiceException, e:
        print "Service call failed: %s"%# -*- coding: utf-8 -*-

def mainpfast():
    print(__file__ + " start!!")
    rospy.init_node('rosie_rrt', anonymous=True)

	map_sub = rospy.Subscriber('/maze_map', MarkerArray, wmapCallback) # get obstacles
    objList_sub = rospy.Subscriber('visualization_marker', Marker, objListCallback) # get objects / obstacles
    bat_sub = rospy.Subscriber('visualization_marker_battery', Marker, batListCallback) # get bat obstacles
	odom_sub = rospy.Subscriber('odom', Odometry, odomCallback) # for check position and end of path
	global grid_pub = rospy.Publisher('rosie_path_grid',OccupancyGrid, queue_size=10) # visualize object grid
    global path_pub = rospy.Publisher('rosie_path', Path, queue_size=100)

    rate = rospy.Rate(10)
    # xpoint, ypoint, complete
    # initialize and prepare screen
    while not rospy.is_shutdown():
        global startx, starty, distance, finalpathx, finalpathy
        global mode = "goto" #others are "home" or "explore"
        if mode == "goto": # goes to first element, rest is obstacle
            #sorting eventually
            xpoint = objList[0][0]/resolution
            ypoint = objList[0][1]/resolution
            OBS = obsList + batList + objList[1:]
            [finalpathx, finalpathy] = calculatePath(OBS)
            publishPath()
            mode = "wait"
            lastMode = "goto"
            startx = xpoint
            starty = ypoint
        else if mode == "home":
            xpoint = 0.2/resolution
            ypoint = 0.2/resolution
            OBS = obsList + batList + objList[1:]
            [finalpathx, finalpathy] = calculatePath(OBS)
            publishPath()
            mode = "wait"
            lastMode = "home"
            startx = xpoint
            starty = ypoint
            complete = 0
        else if mode == "explore":
            OBS = obsList + batList + objList
            generate_random_path()
            #extract nodes from map - Random generated points
            lastMode = "explore"
            mode = "wait"
            startx = xpoint
            starty = ypoint
        else if mode == "wait":
            rospy.wait_for_service('new_Path', 1)
            try:
                new_path = rospy.ServiceProxy('new_Path', CalNewPath)
                state = new_path(s)
            except rospy.ServiceException, e:
                print "Path Serice call failed: %s"%e
            if state == 1:
                startx = pose.pose.pose.position.x
                starty = pose.pose.pose.position.y
                mode = "goto"
                lastMode = "wait"
        if isGrabbed == 1 and state == "wait":
            checkEndpoint(1)
        else if isGrabbed == 0 and (state == "goto" or state == "home"):
            checkEndpoint(0)


        Rate.sleep(10)


if __name__ == '__main__':

mainpfast()


**********************************************

   nodes[0] = first;
    bool break_status = false;


    for(int i=0; i<NUMNODES; ++i){
        std::vector<float> q_rand(1,0);
		q_rand = { (XDIM*randZO()), (YDIM*randZO())};

        for(int j=0; j<=i; ++j){
            int r=50;
            std::vector<float> d(1,0);
			d = nodes[j].coord - q_goal.coord;
            if(sqrt(d[0]^2+d[1]^2) <= r){
                break_status = true;
            }
        }
        if(break_status == true){
            break;
        }

        float ndist[i] = {};
        for(int j=0;j<=i;++j){
            node n = nodes[j];
            float tmp = dist(n.coord, q_rand);
            ndist[j] = tmp;
        }
        int it = std::min_element(std::begin(ndist), std::end(ndist));
        std::size_t index = std::distance(std::begin(playerSums), it);
        int min = *it;

        node q_near = nodes(it);
        node q_new;
        q_new.coord = steer(q_rand, q_near.coord, val, EPS);

        if (checkIntersect(q_rand, q_near.coord, obstacle)){

            q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;

            // Within a radius of r, find all existing nodes
            node q_nearest[i];
            int r = 50;
            int neighbor_count = 1;
            for ( int j = 0; j<=i; ++j){
                if( checkIntersect(nodes[j].coord, q_new.coord, obstacle) && dist(nodes[j].coord, q_new.coord) <= r){
                    q_nearest[neighbor_count].coord = nodes[j].coord;
                    q_nearest[neighbor_count].cost = nodes[j].cost;
                    neighbor_count = neighbor_count+1;
                }
            }

            // Initialize cost to currently known value
            node q_min = q_near;
            double C_min = q_new.cost;

            // Iterate through all nearest neighbors to find alternate lower
            // cost paths

            for( int k = 0; k<=i; k++){
                if (checkIntersect(&q_nearest[k].coord, &q_new.coord, obstacle) && &q_nearest[k].cost + dist(&q_nearest[k].coord, &q_new.coord) < C_min){
                    q_min = q_nearest[k];
                    C_min = &q_nearest[k].cost + dist(q_nearest[k].coord, q_new.coord);

                }
            }

            // Update parent to least cost-from node
            for (int j=0; j<=i; ++j){
                if (nodes[j].coord == q_min.coord){
                    q_new.parent = j;
                }
            }

            // Append to nodes
            nodes.push_back(q_new);

        }

    }
