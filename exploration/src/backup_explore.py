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

    contours_negative = find_contours(map_raw, -1.0, fully_connected='low')
    contours_positive = find_contours(map_raw, +1.0, fully_connected='low')

    vertices_negative = []
    for i in range(len(contours_negative)):
        vertices_negative.append(approximate_polygon(contours_negative[i], tolerance=1.0))
    vertices_negative = np.concatenate(vertices_negative, axis=0)
    vertices_negative = np.unique(vertices_negative, axis=0)

    vertices_positive = []
    for i in range(len(contours_positive)):
        vertices_positive.append(approximate_polygon(contours_positive[i], tolerance=1.0))
    vertices_positive = np.concatenate(vertices_positive, axis=0)
    vertices_positive = np.unique(vertices_positive, axis=0)


    vertices_negative = select_frontier(vertices_negative)
    vertices_positive = select_frontier(vertices_positive)

    line_seg_neg = []
    line_seg_neg.append(tuple([vertices_negative[-1], vertices_negative[0]]))
    for i in range(len(vertices_negative) - 1):
        line_seg_neg.append(tuple([vertices_negative[i], vertices_negative[i+1]]))
    print line_seg_neg
    print '\n'

    line_seg_pos = []
    line_seg_pos.append(tuple([vertices_positive[-1], vertices_positive[0]]))
    for i in range(len(vertices_positive) - 1):
        line_seg_pos.append(tuple([vertices_positive[i], vertices_positive[i+1]]))
    print line_seg_pos
    print '------------------------\n'


    # function_x = lambda x: x * map_resolution + map_origin[0]
    # function_y = lambda y: y * map_resolution + map_origin[1]

    # vertices_negative = np.apply_along_axis(function_x, 0, vertices_negative)
    # vertices_negative = np.apply_along_axis(function_y, 1, vertices_negative)

    # print vertices_positive
    # print '\n'
    # print vertices_positive
    # print '\n'

    output_to_rviz(vertices_positive)
    rospy.sleep(1)
    output_to_rviz(vertices_negative)


    # vertices_negative = vertices_negative.astype(int)



    # contours_unknown_cells = contours_populated_cell + contours_oppen_cells
    # contours_unknown_cells = find_contours(map_raw, 1.0, fully_connected='low')
    # print contours_unknown_cells
    # print '\n'

    # frontier = subdivide_polygon(contours_unknown_cells[0], degree=1, preserve_ends=False)
    # frontier = approximate_polygon(contours_unknown_cells[0], tolerance=1.0)

    # fig, (ax1, ax2) = plt.subplots(ncols=2, figsize=(9, 4))

    # print frontier
    # print '\n'

    # frontier_0 = np.delete(frontier, 0, 0)
    # frontier_1 = np.delete(frontier, 1, 0)
    # # frontier = np.unique(frontier, axis = 0)
    # print frontier
    # print '\n'


    # ax1.plot(frontier_0[:, 1], frontier_0[:, 0])
    # ax2.plot(frontier_1[:, 1], frontier_1[:, 0])


    # plt.grid(True)
    # plt.savefig("test.png")
    # plt.show()




    # # get centroids
    # centroids_unknown_cells = []
    # for i in range(len(contours_unknown_cells)):
    #     centroids_unknown_cells.append(compute_centroid(contours_unknown_cells[i]))
    # # make into a set
    # # centroids_unknown_cells = set(centroids_unknown_cells)

    # print 'centroids_unknown_cells:'
    # print centroids_unknown_cells
    # print '\n'


    # candidates = select_frontier(centroids_unknown_cells)
    # print 'candidates:'
    # print candidates
    # print '\n'


    # candidates = select_frontier(frontier_0)
    # print candidates
    # output_to_rviz(candidates)


    # x_target = candidates[0][0]
    # y_target = candidates[0][1]
    # theta_target = 0

    # go_to_point(y_target, x_target, theta_target)


    # # contours_populated_cells
    # contours_populated_cells = measure.find_contours(map_raw, 0.5)

    # # get centroids
    # centroids_unknown_cells = []
    # for i in range(len(contours_unknown_cells)):
    #     centroids_unknown_cells.append(compute_centroid(contours_unknown_cells[i]))
    # # make into a set
    # centroids_unknown_cells = set(centroids_unknown_cells)

    # # get centroids
    # centroids_populated_cells = []
    # for i in range(len(contours_populated_cells)):
    #     centroids_populated_cells.append(compute_centroid(contours_populated_cells[i]))
    # # make into a set
    # centroids_populated_cells = set(centroids_populated_cells)

    # # get candidate_frontiers
    # frontiers_candidates_map = centroids_unknown_cells.difference(centroids_populated_cells)

    # # if len(frontiers_candidates_map) == 0:
    # #     go_to_point(0.5, 0.0)
    # #     get_frontier_centroid()

    # return frontiers_candidates_map

    #---------------------------------------------------------------------


        global map_raw
    global map_origin
    global map_resolution

    saved_map = map_raw

    contours_negative = find_contours(map_raw, -1.0, fully_connected='low')
    contours_positive = find_contours(map_raw, +1.0, fully_connected='low')

    vertices_negative = []
    for i in range(len(contours_negative)):
        vertices_negative.append(approximate_polygon(contours_negative[i], tolerance=1.0))
    vertices_negative = np.concatenate(vertices_negative, axis=0)
    vertices_negative = np.unique(vertices_negative, axis=0)

    vertices_positive = []
    for i in range(len(contours_positive)):
        vertices_positive.append(approximate_polygon(contours_positive[i], tolerance=1.0))
    vertices_positive = np.concatenate(vertices_positive, axis=0)
    vertices_positive = np.unique(vertices_positive, axis=0)


    vertices_negative = select_frontier(vertices_negative)
    vertices_positive = select_frontier(vertices_positive)

    # line_seg_neg = []
    # line_seg_neg.append(tuple([vertices_negative[-1], vertices_negative[0]]))
    # for i in range(len(vertices_negative) - 1):
    #     line_seg_neg.append(tuple([(int)vertices_negative[i], (int)vertices_negative[i+1]]))
    # print line_seg_neg
    # print '\n'

    line_seg_pos = []
    # line_seg_pos.append(tuple([(int)vertices_positive[-1], (int)vertices_positive[0]]))
    for i in range(len(vertices_positive) - 1):
        print vertices_positive[i]
    #     line_seg_pos.append(tuple([(int)vertices_positive[i], (int)vertices_positive[i+1]]))
    # print line_seg_pos
    # print '\n------------------------'



    # saved_map[saved_map >   0] =  -1
    # saved_map[saved_map == -1] =   1
    # saved_map[saved_map ==  0] =   0

    # out_name = 'catkin_ws/src/Frontier_Exploration/exploration/maps/saved_map.txt'
    # np.savetxt(out_name, saved_map, fmt = '%d');

    # contours = find_contours(saved_map, 0.5, fully_connected='low')
    # print contours
    # print '\n'
    # # contours_positive = find_contours(saved_map, +1.0, fully_connected='high')


    # vertices = []
    # for i in range(len(contours)):
    #     vertices.append(translate_to_map(contours[i]))
    #     print vertices[i]
    #     output_to_rviz(vertices[i])
    #     rospy.sleep(1)

    # print vertices[0]
    # print '\n'

    # vertices = np.concatenate(vertices, axis=0)
    # vertices = np.unique(vertices, axis=0)
    # vertices = translate_to_map(vertices)
    # vertices_negative = tuple(tuple(x) for x in vertices_negative)
    # vertices_negative = set(vertices_negative)

    # contours_positive = find_contours(saved_map, 0.5, fully_connected='low')
    # vertices_positive = []
    # for i in range(len(contours_positive)):
    #     vertices_positive.append(approximate_polygon(contours_positive[i], tolerance=2.0))
    # vertices_positive = np.concatenate(vertices_positive, axis=0)
    # vertices_positive = np.unique(vertices_positive, axis=0)
    # vertices_positive = translate_to_map(vertices_positive)
    # vertices_positive = tuple(tuple(x) for x in vertices_positive)
    # vertices_positive = set(vertices_positive)

    # contours_negative = find_contours(saved_map, -0.5, fully_connected='low')
    # vertices_negative = []
    # for i in range(len(contours_negative)):
    #     vertices_negative.append(approximate_polygon(contours_negative[i], tolerance=2.0))
    # vertices_negative = np.concatenate(vertices_negative, axis=0)
    # vertices_negative = np.unique(vertices_negative, axis=0)
    # vertices_negative = translate_to_map(vertices_negative)
    # vertices_negative = tuple(tuple(x) for x in vertices_negative)
    # vertices_negative = set(vertices_negative)

    # frontier = vertices_negative.intersection(vertices_positive)
    # frontier = list(frontier)
    # output_to_rviz(frontier)
    # print frontier

    # vertices_negative = list(vertices_negative)
    # vertices_positive = list(vertices_positive)



    # print contours
    # print '\n'

    # # print contours[0]
    # # print '\n'

    # frontier = translate_to_map(contours[0])

    # print vertices_positive
    # print '\n'
    # print frontier
    # print '\n'
    # print '\n'


    # output_to_rviz(vertices[0])
    # rospy.sleep(5)


    # vertices_negative = vertices_negative.astype(int)


    # # contours_unknown_cells = contours_populated_cell + contours_oppen_cells
    # contours_unknown_cells = find_contours(saved_map, 1.0, fully_connected='low')

    # frontier = approximate_polygon(contours_unknown_cells[0], tolerance=1.0)
    # frontier_0 = np.delete(frontier, 0, 0)

    # fig, (ax1, ax2) = plt.subplots(ncols=2, figsize=(9, 4))
    # ax1.plot(frontier_0[:, 1], frontier_0[:, 0])
    # plt.grid(True)
    # plt.savefig("test.png")
    # plt.show()


def select_frontier(frontiers_candidates_map):

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
        coordinate[0] = round(ele[0] * map_resolution + map_origin[0], 2)
        coordinate[1] = round(ele[1] * map_resolution + map_origin[1], 2)

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
    np.savetxt(out_name, map_raw, fmt = '%d');

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
        marker.lifetime = rospy.Duration(1.0)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
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
publisher = rospy.Publisher('visualization_marker_array', MarkerArray)

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
