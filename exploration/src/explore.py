#!/usr/bin/env python

import rospy
import actionlib
import tf
import math
import numpy as np
import union_find
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from skimage.measure import find_contours
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Pose, Point
# from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


def get_candidates():

    global map_raw
    global map_origin
    global map_resolution

    # save current occupancy grid for reliable computations
    saved_map = map_raw

    # compute contours
    contours_negative = find_contours(saved_map, -1.0, fully_connected='high')
    contours_positive = find_contours(saved_map,  1.0, fully_connected='high')

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
        contours_positive[index][0] = round(contours_positive[index][0] * map_resolution + map_origin[0], 2)
        contours_positive[index][1] = round(contours_positive[index][1] * map_resolution + map_origin[1], 2)

    # convert contour np arrays into sets
    set_negative = set([tuple(x) for x in contours_negative])
    set_positive = set([tuple(x) for x in contours_positive])

    # perform set difference operation to find frontiers
    frontier = set_negative.difference(set_positive)

    # convert set of frotiers into a list (hasable type data structre)
    frontier = [x for x in frontier]

    # group frontier points into clusters based on distance
    frontier = groupTPL(frontier, 0.02)

    # make list of np arrays of clustered frontier points
    candidates = []
    for i in range(len(frontier)):
        candidates.append(np.array(frontier[i]))

    # return candidates as np array of np arays
    return candidates

def groupTPL(TPL, distance=1):

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

def utility_function(centroids):

    # chosen utility function : length / distance

    # pre allocate utility_array
    utility_array = np.zeros((centroids.shape[0], centroids.shape[1]))

    # make a copy of centroids and use for loop to
    # substitute length atribute with utility of point
    utility_array = np.copy(centroids)

    # compute current position on the map
    current_position, current_quaternion = get_current_pose('/map', '/odom')

    for index in range(len(centroids)):

        # compute manhattan distance
        man_dist = abs(current_position[0] - centroids[index][0])\
                 + abs(current_position[1] - centroids[index][1])

        # compute length / distance
        utility = centroids[index][2] / man_dist

        # substitute length atribute with utility of point
        utility_array[index][2] = utility

    # sort utility_array based on utility
    index = np.argsort(utility_array[:, 2])
    utility_array[:] = utility_array[index]

    # reverse utility_array to have greatest utility as index 0
    utility_array = utility_array[::-1]

    # define goal [x, y]
    goal = np.array([utility_array[0][0], utility_array[0][1]])

    # return goal as np array
    return goal

def rviz_and_graph(candidates, centroids, goal):

    global rviz_id

    # print out graph
    if rviz_id % 500 == 0:

        for i in range(len(candidates)):
            plt.plot(candidates[i][:, 1], candidates[i][:, 0], '.')

        plt.plot(centroids[:, 1], centroids[:, 0], 'ro')
        plt.plot(goal[1], goal[0], 'gX')

        plt.grid(True)
        plt.savefig('graph_' + str(rviz_id / 500) + '.png')


    color = ColorRGBA()

    # # candidates
    # scale needs [x, y, z] atributes
    scale = Point(map_resolution * 2, map_resolution * 2, map_resolution * 2)
    # color nneds [r, g, b, a] atributes
    color = ColorRGBA(0.0, 0.0, 1.0, 0.50)
    # concatenate candidates because output_to_rviz recives a single array
    candidates = np.concatenate(candidates)
    # publish to rviz
    output_to_rviz(candidates, scale, color)

    # centroids
    # scale needs [x, y, z] atributes
    scale = Point(map_resolution * 3, map_resolution * 3, map_resolution * 3)
    # color nneds [r, g, b, a] atributes
    color = ColorRGBA(1.0, 1.0, 0.0, 0.75)
    # publish to rviz
    output_to_rviz(centroids, scale, color)

    # goal
    # scale needs [x, y, z] atributes
    scale = Point(map_resolution * 4, map_resolution * 4, map_resolution * 4)
    # color nneds [r, g, b, a] atributes
    color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
    # publish to rviz
    output_to_rviz([goal], scale, color)


def get_current_pose(target_frame, source_frame):

    # rospy.Time(0) in the context of tf returns the latest available transform
    position, quaternion = tflistener.lookupTransform(target_frame, source_frame, rospy.Time(0))
    return position, quaternion


def callback_map(OccupancyGrid):

    global start_flag
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
    map_origin = np.array([info.origin.position.x, info.origin.position.y, info.origin.position.z])

    # get map resolution
    map_resolution = info.resolution

    # set start_flag to true
    start_flag = True

    # gice time to update the OccupancyGrid
    rospy.sleep(1)


def go_to_point(x_target, y_target, theta_target = 0):
    """ Move to a location relative to the indicated frame """

    rospy.loginfo("navigating to: ({},{},{})".format(x_target, y_target, theta_target))

    goal = create_goal_message(x_target, y_target, theta_target, '/map')

    rospy.loginfo("Waiting for server.")
    action.wait_for_server()

    rospy.loginfo("Sending goal.")
    action.send_goal(goal)
    rospy.loginfo("Goal Sent.")

    # Check in after a while to see how things are going.
    rospy.sleep(1)
    rospy.loginfo("Status Text: {}".format(action.get_goal_status_text()))

    # Should be either "ACTIVE", "SUCCEEDED" or "ABORTED"
    state_name = actionlib.get_name_of_constant(GoalStatus, action.get_state())
    rospy.loginfo("State      : {}".format(state_name))

    # Wait until the server reports a result.
    action.wait_for_result()
    rospy.loginfo("Status Text: {}".format(action.get_goal_status_text()))

    # Should be either "SUCCEEDED" or "ABORTED"
    state_name = actionlib.get_name_of_constant(GoalStatus, action.get_state())
    rospy.loginfo("State      : {}".format(state_name))

def create_goal_message(x_target, y_target, theta_target, frame = '/map'):
    """Create a goal message in the indicated frame"""

    quat = tf.transformations.quaternion_from_euler(0, 0, theta_target)
    # Create a goal message ...
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.get_rostime()

    goal.target_pose.pose.position.x = x_target
    goal.target_pose.pose.position.y = y_target

    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]
    return goal

def output_to_rviz(array, scale, color):

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
        marker.lifetime = rospy.Duration(5.0)
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
    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
    rospy.sleep(1)


## Main ########################################################################

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
action = actionlib.SimpleActionClient("move_base", MoveBaseAction)
action.wait_for_server()
print "Server connection stablished"

# tell user how to stop TurtleBot
rospy.loginfo("To stop TurtleBot CTRL + C")
# what function to call when you ctrl + c
rospy.on_shutdown(shutdown)

# set start flag to false -- wait until map is published
start_flag = False

# initialyze counter to give a unique id for rviz features
rviz_id = 0


while not rospy.is_shutdown():
    if start_flag:

        candidates = get_candidates()
        centroids = compute_centroids(candidates)
        goal = utility_function(centroids)
        rviz_and_graph(candidates, centroids, goal)
        # x and y are inverted due to confliction frames
        # of recerence from Occupancy grid & /map
        go_to_point(goal[1], goal[0])
        rospy.sleep(1)