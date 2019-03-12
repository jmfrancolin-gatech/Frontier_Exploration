#!/usr/bin/env python

import rospy
import actionlib
import tf
import math
import numpy as np
import union_find
import matplotlib.pyplot as plt

from skimage.measure import find_contours
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header, ColorRGBA

def get_frontiers():

    print 'Inside get_frontiers()'

    global map_raw
    global map_origin
    global map_resolution
    global cluster_trashhole

    # compute contours
    # wait until Ocuupancy grid output features
    # temporalUpdate = 30s (max wait time) see launch file
    while True:

        # save current occupancy grid for reliable computations
        saved_map = np.copy(map_raw)

        contours_negative = find_contours(saved_map, -1.0, fully_connected='high')
        contours_positive = find_contours(saved_map,  1.0, fully_connected='high')
        
        if len(contours_negative) != 0:
            break
        rospy.sleep(5)

    # translate contours to map frame
    contours_negative = np.concatenate(contours_negative, axis=0)
    for index in range(len(contours_negative)):
        contours_negative[index][0] = round(contours_negative[index][0]\
                                    * map_resolution + map_origin[0], 2)
        contours_negative[index][1] = round(contours_negative[index][1]\
                                    * map_resolution + map_origin[1], 2)

    # translate contours to map frame
    contours_positive = np.concatenate(contours_positive, axis=0)
    for index in range(len(contours_positive)):
        contours_positive[index][0] = round(contours_positive[index][0]\
                                    * map_resolution + map_origin[0], 2)
        contours_positive[index][1] = round(contours_positive[index][1]\
                                    * map_resolution + map_origin[1], 2)

    # convert contour np arrays into sets
    set_negative = set([tuple(x) for x in contours_negative])
    set_positive = set([tuple(x) for x in contours_positive])

    # perform set difference operation to find candidates
    candidates = set_negative.difference(set_positive)

    # convert set of frotiers into a list (hasable type data structre)
    candidates = [x for x in candidates]

    # group candidates points into clusters based on distance
    candidates = groupTPL(candidates, cluster_trashhole)

    # make list of np arrays of clustered frontier points
    frontiers = []
    for i in range(len(candidates)):
        frontiers.append(np.array(candidates[i]))

    # return frontiers as list of np arays
    return frontiers

def groupTPL(TPL, distance=1):

    # TO-DO:
    # Rethink ways to cluster points
    # K-d tree may be an alternative
    # Currently algorithm runs on O(n^2)

    print 'Inside groupTPL()'
    U = union_find.UnionFind()

    for (i, x) in enumerate(TPL):
        for j in range(i + 1, len(TPL)):
            y = TPL[j]
            if max(abs(x[0] - y[0]), abs(x[1] - y[1])) <= distance:
                U.union(x, y)

    disjSets = {}
    for x in TPL:
        s = disjSets.get(U[x], set())
        s.add(x)
        disjSets[U[x]] = s

    return [list(x) for x in disjSets.values()]

def compute_centroids(list_of_arrays):

    print 'Inside compute_centroids()'
    global map_resolution

    centroids = []
    for index in range(len(list_of_arrays)):
        # compute num of elements in the array
        length = list_of_arrays[index].shape[0]
        # compute real world length of frontier in cm
        real_length = np.round(length * map_resolution * 100)
        # compute x coordanate of centroid
        sum_x = np.sum(list_of_arrays[index][:, 0])
        x = np.round(sum_x / length, 2)
        # compute y coodenate of centroid
        sum_y = np.sum(list_of_arrays[index][:, 1])
        y = np.round(sum_y / length, 2)
        # append coordanate in the form [x, y, real_length]
        centroids.append([x, y, real_length])

    # convert list of centroids into np array
    centroids = np.array(centroids)
    # return centroids as np array
    return centroids

def utility_function(centroids, turtlebot_goals):

    # TO-DO:
    # Check if current_position for tf listener is also inverted
    # Check accuracy of manhattan distance calculation

    print 'Inside utility_function()'

    # concatenate
    turtlebot_goals = np.vstack([turtlebot_goals, centroids])
    turtlebot_goals = np.unique(turtlebot_goals, axis=0)

    # sort
    index = np.argsort(turtlebot_goals[:, 2])
    turtlebot_goals[:] = turtlebot_goals[index]
    turtlebot_goals = turtlebot_goals[::-1]

    # filter
    turtlebot_goals = turtlebot_goals[turtlebot_goals[:,2] >= 50]

    # return
    return turtlebot_goals

    # # chosen utility function : length / distance

    # # pre allocate utility_array
    # utility_array = np.zeros((centroids.shape[0], centroids.shape[1]))

    # # make a copy of centroids and use for loop to
    # # substitute length atribute with utility of point
    # utility_array = np.copy(centroids)

    # # compute current position on the map
    # current_position, current_quaternion = get_current_pose('/map', '/odom')

    # for index in range(len(centroids)):

    #     # compute manhattan distance
    #     man_dist = abs(current_position[0] - centroids[index][0])\
    #              + abs(current_position[1] - centroids[index][1])

    #     # compute length / distance
    #     utility = centroids[index][2] / man_dist

    #     # substitute length atribute with utility of point
    #     utility_array[index][2] = utility

    # # sort utility_array based on utility
    # index = np.argsort(utility_array[:, 2])
    # utility_array[:] = utility_array[index]

    # # reverse utility_array to have greatest utility as index 0
    # utility_array = utility_array[::-1]

    # for i in range(3):
    #     coordanate = []

    #     if i < len(utility_array):
    #         coordanate = [utility_array[i][0], utility_array[i][1], ]
    #         turtlebot_goals = np.vstack([turtlebot_goals, coordanate])

    # return goal as np array
    # return np.array(goals)

def rviz_and_graph(frontiers, centroids, goals):

    print 'Inside rviz_and_graph()'

    # global graph_id

    # # print out graph every 200 points and update counters
    # if rviz_id > 1000 or rviz_id == 0:

    #     # graph frontiers
    #     for i in range(len(frontiers)):
    #         plt.plot(frontiers[i][:, 1], frontiers[i][:, 0], '.')

    #     # graph centroids
    #     plt.plot(centroids[:, 1], centroids[:, 0], 'ro')

    #     # graph goals
    #     plt.plot(goals[:, 1], goals[:, 0], 'gX')

    #     plt.grid(True)
    #     plt.savefig('graph_' + str(graph_id) + '.png')

    #     graph_id = graph_id + 1
    #     rviz_id = 0

    global rviz_id

    color = ColorRGBA()
    dimention = map_resolution * 2

    # # frontiers
    # scale needs [x, y, z] atributes
    scale = Point(dimention, dimention, dimention)
    # color nneds [r, g, b, a] atributes
    color = ColorRGBA(0.0, 0.0, 1.0, 0.50)
    # concatenate frontiers because output_to_rviz recives a single array
    frontiers = np.concatenate(frontiers)
    # publish to rviz
    output_to_rviz(frontiers, scale, color)

    # centroids
    # scale needs [x, y, z] atributes
    scale = Point(dimention, dimention, dimention)
    # color nneds [r, g, b, a] atributes
    color = ColorRGBA(1.0, 1.0, 0.0, 0.75)
    # publish to rviz
    output_to_rviz(centroids, scale, color)

    # goals
    # scale needs [x, y, z] atributes
    scale = Point(dimention * 2, dimention * 2, dimention * 2)
    # color nneds [r, g, b, a] atributes
    color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
    # publish to rviz
    output_to_rviz(goals, scale, color)

def get_current_pose(target_frame, source_frame):

    print 'Inside get_current_pose()'

    # rospy.Time(0) in the context of tf returns the latest available transform
    position, quaternion = tflistener.lookupTransform(
                            target_frame, source_frame, rospy.Time(0))
    return position, quaternion

def callback_map(OccupancyGrid):

    print 'Inside callback_map()'

    global map_raw
    global map_origin
    global map_resolution

    # get meta-data
    info = OccupancyGrid.info
    data = OccupancyGrid.data

    # make numpy array from OccupancyGrid
    map_raw = np.array(data)
    map_raw = map_raw.reshape(info.height, info.width)

    np.savetxt('map_raw.txt', map_raw, fmt = '%d');

    # get initial position
    map_origin = np.array([info.origin.position.x, info.origin.position.y,\
                 info.origin.position.z])

    # get map resolution
    map_resolution = info.resolution

    # gice time to update the OccupancyGrid
    rospy.sleep(1)

def go_to_point(x_target, y_target, theta_target = 0):
    """ Move to a location relative to the indicated frame """

    print 'Inside go_to_point()'

    rospy.loginfo("navigating to: ({},{},{})".format(x_target, y_target, theta_target))

    goal = create_goal_message(x_target, y_target, theta_target, '/map')

    #start moving
    move_base.send_goal(goal)

    #allow TurtleBot up to 60 seconds to complete task
    success = move_base.wait_for_result(rospy.Duration(30))

    print success

    # if not successfull, cancel goal
    if not success:
        move_base.cancel_goal()

    # output status
    state = move_base.get_state()
    rospy.loginfo("State      : {}".format(state))

    # return boolean type
    return success

def create_goal_message(x_target, y_target, theta_target, frame = '/map'):
    """Create a goal message in the indicated frame"""

    print 'Inside create_goal_message()'

    quat = tf.transformations.quaternion_from_euler(0, 0, theta_target)
    # Create a goal message ...
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.get_rostime()
    # position
    goal.target_pose.pose.position.x = x_target
    goal.target_pose.pose.position.y = y_target
    # orientation
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    return goal

def output_to_rviz(array, scale, color):

    print 'Inside output_to_rviz()'

    global publisher
    global rviz_id

    # make MarkerArray message
    markerArray = MarkerArray()

    # loop throgh all instances of the array
    for index in range(len(array)):
        marker = Marker()
        marker.id = rviz_id
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(30.0)
        marker.scale = scale
        marker.color = color
        # x and y are inverted due to nature of the map
        marker.pose.position.x = array[index][1]
        marker.pose.position.y = array[index][0]
        markerArray.markers.append(marker)
        # incremment rviz_id
        rviz_id = rviz_id + 1

    # Publish the MarkerArray
    publisher.publish(markerArray)

def shutdown():

    print 'Inside shutdown()'

    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
    rospy.sleep(1)

def controller():

    # initialyze goal queues
    # [x, y, priority]
    turtlebot_goals = np.array([np.nan, np.nan, np.nan]);
    snake_goals = np.array([np.nan, np.nan, np.nan]);

    while not rospy.is_shutdown():

        print 'Inside controller()'

        frontiers = get_frontiers()
        centroids = compute_centroids(frontiers)
        turtlebot_goals = utility_function(centroids, turtlebot_goals)

        # x and y are inverted due to conflicting frames
        # of recerence from Occupancy grid & /map

        # convert contour np arrays into sets
        set_turtlebot = set([tuple(x) for x in turtlebot_goals])
        set_snake = set([tuple(x) for x in snake_goals])

        # perform set difference operation to find candidates
        turtlebot_goals = set_turtlebot.difference(set_snake)

        # convert set back into numpy array
        turtlebot_goals = [x for x in candidates]
        turtlebot_goals = np.array(turtlebot_goals);

        if len(turtlebot_goals) > 0:

            print 'navigating to turtlebot_goals'

            rviz_and_graph(frontiers, centroids, [turtlebot_goals[0]])
            success = go_to_point(turtlebot_goals[0][1], turtlebot_goals[0][0])

            print turtlebot_goals
            
            # if can't reach goal, put goal into snake queue
            if not success:
                snake_goals = np.vstack([snake_goals, turtlebot_goals[0]])
                snake_goals = np.unique(snake_goals, axis=0)
            
            turtlebot_goals = np.delete(turtlebot_goals, (0), axis=0)

        elif len(snake_goals) > 0:

            print 'navigating to snake_goals'

            rviz_and_graph(frontiers, centroids, [snake_goal])
            success = go_to_point(snake_goal[i][1], snake_goal[i][0])

        else:
            shutdown()

        # for goal in goals:
        #     go_to_point(goal[1], goal[0])
        #     rospy.sleep(1)

## Init ########################################################################

# set priting options
np.set_printoptions(suppress=True)

# initialyze node
rospy.init_node('nav_node')

# subscribe to topics of interest
rospy.Subscriber('/map', OccupancyGrid, callback_map)
rospy.Subscriber('/odom', Odometry)

# publish frontier markers rviz
publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

# initialyze listener
tflistener = tf.TransformListener()

# initialyze action client
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
move_base.wait_for_server()
print "Server connection stablished"

# tell user how to stop TurtleBot
rospy.loginfo("To stop TurtleBot CTRL + C")
# what function to call when you ctrl + c
rospy.on_shutdown(shutdown)

rospy.sleep(10)

# initialyze counter to give a unique id for rviz features
rviz_id = 0
graph_id = 0
# parameters dependent on map_resolution & necessary to cluster points
cluster_trashhole = 0.1

# call controller
controller()