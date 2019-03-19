#!/usr/bin/env python

import rospy
import actionlib
import tf
import math
import numpy as np
import union_find
import matplotlib.pyplot as plt

from std_srvs.srv import Empty
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Quaternion

from skimage.measure import find_contours
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
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

    # filter
    turtlebot_goals = turtlebot_goals[turtlebot_goals[:,2] > 50]

    # sort
    index = np.argsort(turtlebot_goals[:, 2])
    turtlebot_goals[:] = turtlebot_goals[index]
    turtlebot_goals = turtlebot_goals[::-1]

    # return
    return turtlebot_goals

def rviz_and_graph(frontiers, centroids, goals):

    print 'Inside rviz_and_graph()'

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

    #allow TurtleBot up to 30 seconds to complete task
    success = move_base.wait_for_result(rospy.Duration(20))

    # if not successfull, cancel goal
    if not success:
        move_base.cancel_goal()
        rospy.sleep(1)

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

    global clear_costmaps

    # initialyze goal queues
    # [x, y, priority]
    turtlebot_goals = np.array([np.nan, np.nan, np.nan]);
    snake_goals = np.array([np.nan, np.nan, np.nan]);

    # concatenate
    snake_goals = np.vstack([snake_goals, snake_goals])
    snake_goals = np.unique(snake_goals, axis=0)

    #rospy.set_param('/move_base/DWAPlannerROS/xy_goal_tolerance', 0.1)

    while not rospy.is_shutdown():

        print 'Inside controller()'

        frontiers = get_frontiers()
        centroids = compute_centroids(frontiers)

        if len(turtlebot_goals) > 0:

            turtlebot_goals = utility_function(centroids, turtlebot_goals)

            for i in range(3):

                if len(turtlebot_goals) == 1:
                    current_position, current_quaternion = get_current_pose('/map', '/odom')
                    roll, pitch, yaw = tf.transformations.euler_from_quaternion(current_quaternion)
                    
                    print 'trying to turn'
                    print 'current angle: ' + str(yaw)
                    go_to_point(current_position[1], current_position[0], yaw+3.14)
                    rospy.sleep(1)
                    go_to_point(current_position[1], current_position[0], yaw+3.14)

                    frontiers = get_frontiers()
                    centroids = compute_centroids(frontiers)
                    turtlebot_goals = utility_function(centroids, turtlebot_goals)

                # temporary solution for difference between
                # turtlebot_goals and snake_goals
                set_turtlebot = set([tuple(x) for x in turtlebot_goals])
                set_snake = set([tuple(x) for x in snake_goals])
                turtlebot_goals = set_turtlebot.difference(set_snake)
                turtlebot_goals = [x for x in turtlebot_goals]
                turtlebot_goals = np.array(turtlebot_goals);

                # x and y are inverted due to conflicting frames
                # of recerence from Occupancy grid & /map
                print 'turtlebot_goals: '
                print turtlebot_goals
                print len(turtlebot_goals)

                if i < len(turtlebot_goals):

                    quat = tf.transformations.quaternion_from_euler(0, 0, 0)
                    current_position, current_quaternion = get_current_pose('/map', '/odom')
            

                    start = PoseStamped()
                    start.header.frame_id = "map"
                    start_point = Point(current_position[1], current_position[0],0)                    
                    start_orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])
                    start_pose = Pose(start_point, start_orientation)
                    start.pose = start_pose

                    #print 'start:'
                    #print start

                    goal = PoseStamped()
                    goal.header.frame_id = "map"
                    goal_point = Point(turtlebot_goals[i][1], turtlebot_goals[i][0],0)
                    goal_orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])
                    goal_pose = Pose(goal_point, goal_orientation)
                    goal.pose = goal_pose

                    #print 'goal:'
                    #print goal

                    tolerance = 0.1

                    make_plan.wait_for_service()
                    state = move_base.get_state()
                    rospy.loginfo("State      : {}".format(state))

                    while state == 0 or state == 1:
                        rospy.sleep(1)
                        state = move_base.get_state()

                    plan_response = make_plan(start = start, goal = goal, tolerance = tolerance)
                    #print 'plan:'
                    #print plan_response.plan.poses


                    print 'navigating to turtlebot_goals'
                    rviz_and_graph(frontiers, centroids, [turtlebot_goals[i]])
                    
                    success = go_to_point(turtlebot_goals[i][1], turtlebot_goals[i][0])
                    
                    # if can't reach goal, put goal into snake queue
                    if not success:
                        snake_goals = np.vstack([snake_goals, turtlebot_goals[0]])
                        snake_goals = np.unique(snake_goals, axis=0)
                    
                    turtlebot_goals = np.delete(turtlebot_goals, (i), axis=0)

        elif len(snake_goals) > 0:

            # clear_costmaps = rospy.ServiceProxy('clear_costmaps', Empty)
            # rospy.wait_for_service('clear_costmaps')
            # clear_costmaps (std_srvs/Empty)

            clear_costmaps()

            # override xy_goal_tolerance
            rospy.set_param('/move_base/DWAPlannerROS/xy_goal_tolerance', 0.1)
            rospy.set_param('/move_base/global_costmap/robot_radius', 0.2)
            rospy.set_param('/move_base/local_costmap/inflation_layer/inflation_radius' , 0.1)

            # filter
            snake_goals = snake_goals[snake_goals[:,2] > 50]

            # sort
            index = np.argsort(snake_goals[:, 2])
            snake_goals[:] = snake_goals[index]
            snake_goals = snake_goals[::-1]

            print 'snake_goals:'
            print snake_goals

            for snake_goal in snake_goals:

                print 'navigating to snake_goals'
                print 'snake_goals :'
                print snake_goals

                quat = tf.transformations.quaternion_from_euler(0, 0, 0)
                current_position, current_quaternion = get_current_pose('/map', '/odom')
        

                start = PoseStamped()
                start.header.frame_id = "map"
                start_point = Point(current_position[1], current_position[0],0)                    
                start_orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])
                start_pose = Pose(start_point, start_orientation)
                start.pose = start_pose

                #print 'start:'
                #print start

                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal_point = Point(snake_goal[1], snake_goal[0],0)
                goal_orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])
                goal_pose = Pose(goal_point, goal_orientation)
                goal.pose = goal_pose

                #print 'goal:'
                #print goal

                tolerance = 0.1

                make_plan.wait_for_service()
                state = move_base.get_state()
                rospy.loginfo("State      : {}".format(state))

                while state == 0 or state == 1:
                    rospy.sleep(1)
                    state = move_base.get_state()

                plan_response = make_plan(start = start, goal = goal, tolerance = tolerance)
                #print 'plan:'
                #print plan_response.plan.poses
                snake_plan = plan_response.plan.poses

                success = False
                while len(snake_plan) > 0 and not success:

                    goal = MoveBaseGoal()
                    goal_pose_stamped = snake_plan[-1]

                    goal.target_pose.header.frame_id = '/map'
                    goal.target_pose.header.stamp = rospy.get_rostime()
                    goal.target_pose.pose = goal_pose_stamped.pose
                    move_base.send_goal(goal)

                    #allow TurtleBot up to 30 seconds to complete task
                    success = move_base.wait_for_result(rospy.Duration(20))

                    # if not successfull, cancel goal
                    if not success:
                        move_base.cancel_goal()
                        rospy.sleep(1)
                        snake_plan = snake_plan[0:len(snake_plan)-10]

                    if success:
                        print 'SUCCESS!'
                        rospy.sleep(20)


                rospy.sleep(5)

                #rviz_and_graph(frontiers, centroids, [snake_goal])
                #success = go_to_point(snake_goal[1], snake_goal[0])

                # # implemment check if still frontier

                # if success:
                #     snake_goals = np.delete(snake_goals, (0), axis=0)
        else:
            shutdown()



        np.savetxt('snake_goals.txt', snake_goals, fmt = '%.2f');
        np.savetxt('turtlebot_goals.txt', turtlebot_goals, fmt = '%.2f');

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


###
clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)

make_plan = rospy.ServiceProxy('move_base/make_plan', GetPlan)
make_plan.wait_for_service()


# clear_costmaps = rospy.ServiceProxy('clear_costmaps', Empty)
# # rospy.wait_for_service('clear_costmaps')
# clear_costmaps (Empty) 

# call controller
controller()