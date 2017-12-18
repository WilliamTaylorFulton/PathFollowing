#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import Image, ImageDraw
import math
import Queue
import os.path
from discretemap import *

#####################
# BEGIN Global Variable Definitions
robot = [0,0,0] # This is the robot's actual location
dmap = None # This is our discrete map
path = None #Path in map coordinates (to be loaded from a file)
# END Global Variable Definitions
#####################

def robotCallback(data):
    #This function updates the robots position and yaw, based on the ground truth (no localization for this lab)
    global robot
    [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    robot = [data.pose.pose.position.x,data.pose.pose.position.y,yaw]

def nextWaypoint(waypoint):
    #Figure out which waypoint in the path should be the next one.  Set it in waypoint.x and waypoint.y and return
    
    global robot
    global discrete_map
    global path
    global path_index

    
    for p in path:
        if discrete_map.clear_path((robot[0],robot[1]),p):
            waypoint.x = p[0]
            waypoint.y = p[1]


    #Determine which point in path should be the robot's next waypoint
    #Set waypoint.x and waypoint.y to the waypoint (in map coordinates)
    
    #You can use discrete_map.clear_path(point_a,point_b) 
    #which will return true if there are no obstacles 
    #between point_a and point_b on the map, and will 
    #return false if there is an obstacle between them

    #LAB 5 PART C CODE END

    #Return the waypoint
    return waypoint

if __name__ == '__main__':
    #ROS Initialization
    rospy.init_node('waypointmanager', anonymous=True) #Initialize the ros node
    pub = rospy.Publisher('next_waypoint', Point) #Create our publisher to send waypoints to the driver
    rospy.Subscriber("base_pose_ground_truth", Odometry, robotCallback) #Subscribe to the robot pose topic

    global discrete_map
    global path
    global path_index

    path_index = 0

    rate = rospy.Rate(2)

    #Setup things that we need to
    #Get info from .world file
    args = rospy.myargv(argv=sys.argv)
    discrete_map = DiscreteMap(args[1],5)

    # Get the path to the world file
    world_dir = os.path.dirname(args[1])

    #Load the path from the file
    path_name = args[1][0:-6] + "_astarpath.pkl"
    path_file = open(path_name,'rb')
    path = pickle.load(path_file)
    path_file.close()

    print 'Loaded path from file: ', path_name

    #Main loop to publish waypoints for the robot to follow
    while not rospy.is_shutdown():
        #Publish waypoints to navigate to goal
        waypoint = Point()
        waypoint = nextWaypoint(waypoint)
        if waypoint is not None:
            pub.publish(waypoint)
        rate.sleep()

    
        
