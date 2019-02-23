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
from skimage import measure

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

from tf.transformations import euler_from_quaternion, quaternion_from_euler


def get_frontier_candidates():

    global map_raw
    global map_origin
    global map_resolution

    # contours_unknown_cells = contours_populated_cell + contours_oppen_cells
    contours_unknown_cells = measure.find_contours(map_raw, -0.5)
    print contours_unknown_cells
    print '\n'


    # get centroids
    centroids_unknown_cells = []
    for i in range(len(contours_unknown_cells)):
        centroids_unknown_cells.append(compute_centroid(contours_unknown_cells[i]))
    # make into a set
    centroids_unknown_cells = set(centroids_unknown_cells)
    print centroids_unknown_cells
    print '\n'


    candidates = select_frontier(centroids_unknown_cells)
    print candidates
    print '\n'



    show_text_in_rviz(candidates)


    x_target = candidates[0][0]
    y_target = candidates[0][1]
    theta_target = 0

    go_to_point(y_target, x_target, theta_target)


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
        coordinate = [np.nan, np.nan, np.nan, np.nan]

        # compute x and y
        coordinate[0] = ele[0] * map_resolution + map_origin[0]
        coordinate[1] = ele[1] * map_resolution + map_origin[1]

        # compute legth in m
        coordinate[2] = ele[2] * map_resolution

        # compute distance in m
        manhattan_dist = abs(coordinate[0] - map_origin[0]) + abs(coordinate[1] - map_origin[1])
        coordinate[3] = manhattan_dist

        # for i in range(len(coordinate) - 1):
        #     coordinate[i] = int(coordinate[i])
        # print coordinate
        # print '\n'

        # append data
        frontiers_candidates_odom.append(coordinate)
#

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
    return (int) (sum_x / length), (int) (sum_y / length), (int) (length)


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

    # print info

    # make numpy array from OccupancyGrid
    map_raw = np.array(data)
    map_raw = map_raw.reshape(info.height, info.width)
    np.savetxt("map_raw.txt", map_raw, fmt = '%d');

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
    rospy.sleep(1.0)
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

def show_text_in_rviz(candidates):

    global publisher

    # for i in range(len(list_pose)):

    # marker = Marker(
    #             type = 7,
    #             id = 0,
    #             lifetime=rospy.Duration(1.5),
    #             pose = list_pose,
    #             scale = Vector3(1.1, 1.1, 1.1),
    #             header = Header(frame_id = '/map'),
    #             color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
    #             )
    # marker_publisher.publish(marker)
    # rospy.sleep(1)

    markerArray = MarkerArray()


    theta_target = 0
    quat = tf.transformations.quaternion_from_euler(0, 0, theta_target)


    for i in range(len(candidates)):

        marker = Marker()
        marker.id = i
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(5.0)

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.pose.position.x = candidates[i][1]
        marker.pose.position.y = candidates[i][0]

        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        markerArray.markers.append(marker)

    # Publish the MarkerArray
    publisher.publish(markerArray)
    rospy.sleep(1)


def shutdown():
    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
    update_rate.sleep()



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

# Define updare rate to 10 HZ
update_rate = rospy.Rate(10);

start_flag = False

rospy.sleep(10)

while not rospy.is_shutdown():
    if start_flag:
        # # get initial position
        # init_position, init_quaternion = tflistener.lookupTransform('/odom', '/map', rospy.Time(0))

        # run algorithm



        get_frontier_candidates()

        # frontiers_candidates_map = get_frontier_candidates()
        # goal = select_frontier(frontiers_candidates_map)
        # show_text_in_rviz(marker_publisher, goal)

        update_rate.sleep()
