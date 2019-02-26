#!/usr/bin/env python

import sys
import rospy
import actionlib
import tf
import math
import numpy as np
import sys

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point
from actionlib_msgs.msg import GoalStatus

from skimage.measure import find_contours, approximate_polygon, subdivide_polygon
import matplotlib.pyplot as plt

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def get_frontier_candidates():

    global map_raw
    global map_origin
    global map_resolution

    saved_map = map_raw


    contours_negative = find_contours(saved_map, -1.0, fully_connected='high')
    contours_positive = find_contours(saved_map,  1.0, fully_connected='high')


    contours_negative = np.concatenate(contours_negative, axis = 0)
    for index in range(len(contours_negative)):
        contours_negative[index][0] = round(contours_negative[index][0] * map_resolution + map_origin[0], 2)
        contours_negative[index][1] = round(contours_negative[index][1] * map_resolution + map_origin[1], 2)

    # contours_negative = np.unique(contours_negative, axis = 0)
    # output_to_rviz(contours_negative)



    contours_positive = np.concatenate(contours_positive, axis = 0)
    for index in range(len(contours_positive)):
        contours_positive[index][0] = round(contours_positive[index][0] * map_resolution + map_origin[0], 2)
        contours_positive[index][1] = round(contours_positive[index][1] * map_resolution + map_origin[1], 2)

    # contours_positive = np.unique(contours_positive, axis = 0)
    # output_to_rviz(contours_positive)





    set_negative = set([tuple(x) for x in contours_negative])
    set_positive = set([tuple(x) for x in contours_positive])



    frontier = set_negative.difference(set_positive)

    frontier = np.array([list(x) for x in frontier])
    print frontier





    fig, (ax1, ax2) = plt.subplots(ncols=2, figsize=(10, 5))



    ax1.plot(contours_negative[:, 1], contours_negative[:, 0], 'bo')
    ax1.plot(contours_positive[:, 1], contours_positive[:, 0], 'rx')

    ax2.plot(frontier[:, 1], frontier[:, 0], 'go')



    plt.grid(True)
    plt.savefig("test.png")
    plt.show()


def translate_to_map(frontiers_candidates_map):

    global map_raw
    global map_origin
    global map_resolution


    # get current pose om /odom frame
    # current_position, current_quaternion = get_current_pose('/map', '/odom')
    # current_x = current_position[0]
    # current_y = current_position[1]
    # print 'current_x = ' + str(current_x) + ' current_y = ' + str(current_y) + '\n'


    # current_position, current_quaternion = get_current_pose('/map', '/map')
    # print '/map to /map'
    # print current_position

    # current_position, current_quaternion = get_current_pose('/odom', '/odom')
    # print '/odom to /odom'
    # print current_position

    # current_position, current_quaternion = get_current_pose('/map', '/odom')
    # print '/map to /odom'
    # print current_position

    # current_position, current_quaternion = get_current_pose('/odom', '/map')
    # print '/odom to /map'
    # print current_position

    # print '\n'

    # go_to_point(0.5, 0.0)



    # initialyze frontier candidate list
    frontiers_candidates_odom = []

    for ele in frontiers_candidates_map:

        # initialyze coodinate structure
        # coordinate = [x, y, length(m), dist(m)]
        coordinate = [np.nan, np.nan]

        # compute x and y
        coordinate[0] = ele[0] * map_resolution + map_origin[0]
        coordinate[1] = ele[1] * map_resolution + map_origin[1]

        # compute legth in m
        # coordinate[2] = ele[2] * map_resolution

        # compute distance in m
        # manhattan_dist = abs(coordinate[0] - map_origin[0]) + abs(coordinate[1] - map_origin[1])
        # coordinate[3] = manhattan_dist

        # for i in range(len(coordinate) - 1):
        #     coordinate[i] = int(coordinate[i])
        # print coordinate
        # print '\n'

        # append data
        frontiers_candidates_odom.append(coordinate)

    # frontiers_candidates_odom = np.unique(frontiers_candidates_odom, axis = 0)
    # frontiers_candidates_odom = filter(lambda x: x[2] > 1.0 and x[3] > 1.0, frontiers_candidates_odom)

    # print frontiers_candidates_odom
    # print '\n'

    # goal = Point(frontiers_candidates_odom[0][0], frontiers_candidates_odom[0][1], 0.0)

    return frontiers_candidates_odom

    # apply utility function



def compute_centroid(array):

    length = array.shape[0]
    sum_x = np.sum(array[:, 0])
    sum_y = np.sum(array[:, 1])
    return sum_x / length, sum_y / length, length


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

    out_name = 'catkin_ws/src/Frontier_Exploration/exploration/maps/map_raw.txt'
    np.savetxt(out_name, map_raw, fmt = '%f');

    # get initial position
    map_origin = np.array([info.origin.position.x, info.origin.position.y, info.origin.position.z])

    # get map resolution
    map_resolution = info.resolution

    # set start_flag to true
    start_flag = True


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

def output_to_rviz(candidates):

    global publisher

    markerArray = MarkerArray()

    # # mark map origin
    # origin_marker = Marker()
    # origin_marker.id = 0
    # origin_marker.header.frame_id = "/map"
    # origin_marker.type = origin_marker.SPHERE
    # origin_marker.action = origin_marker.ADD
    # origin_marker.lifetime = rospy.Duration(10.0)
    # origin_marker.scale.x = 0.2
    # origin_marker.scale.y = 0.2
    # origin_marker.scale.z = 0.2
    # origin_marker.color.a = 1.0
    # origin_marker.color.r = 0.0
    # origin_marker.color.g = 0.0
    # origin_marker.color.b = 1.0
    # origin_marker.pose.position.x = 0.0
    # origin_marker.pose.position.y = 1.0
    # markerArray.markers.append(origin_marker)

    for i in range(len(candidates)):

        marker = Marker()
        marker.id = i + 1
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(10.0)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.pose.position.x = candidates[i][1]
        marker.pose.position.y = candidates[i][0]
        markerArray.markers.append(marker)

    # Publish the MarkerArray
    publisher.publish(markerArray)
    rospy.sleep(1)


def shutdown():
    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
    rospy.sleep(1)



## Main ########################################################################

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

while not rospy.is_shutdown():
    if start_flag:
        # # get initial position
        # init_position, init_quaternion = tflistener.lookupTransform('/odom', '/map', rospy.Time(0))

        # run algorithm



        get_frontier_candidates()

        # frontiers_candidates_map = get_frontier_candidates()
        # goal = select_frontier(frontiers_candidates_map)
        # output_to_rviz(marker_publisher, goal)

        rospy.sleep(1)
